#include "sim_sd.h"

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "fatfs/ff.h"

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#define host_mkdir(p)          _mkdir(p)
#define host_truncate(fp, len) _chsize_s(_fileno(fp), (long long)(len))
#else
#include <unistd.h>
#define host_mkdir(p)          mkdir((p), 0777)
#define host_truncate(fp, len) ftruncate(fileno(fp), (off_t)(len))
#endif

#define SIM_SD_MAX_TRACKED 32
#define SIM_SD_CLUSTER     32768u

sim_sd_t sim_sd;

static FATFS *registered_fs;

/* Open-file state lives here, not in the caller's FIL, so a FIL that goes out
 * of scope without f_close can never leave a dangling pointer behind. */
typedef struct {
    bool used;
    FILE *f;
    FSIZE_t size;
    FSIZE_t synced_size; /* what survives a power cut */
    char path[FF_MAX_LFN + 1];
} slot_t;

static slot_t slots[SIM_SD_MAX_TRACKED];

/* ---------------------------------------------------------------- helpers */

static sim_time_ns_t op_start;

static void op_begin(void)
{
    op_start = sim_now_ns();
}

static FRESULT op_end(FRESULT r)
{
    sim_time_ns_t dt = sim_now_ns() - op_start;
    if (dt > sim_sd.longest_op_ns) {
        sim_sd.longest_op_ns = dt;
    }
    return r;
}

static bool is_dir(const char *p)
{
    struct stat st;
    return stat(p, &st) == 0 && S_ISDIR(st.st_mode);
}

static bool host_file_size(const char *p, uint64_t *size)
{
    struct stat st;
    if (stat(p, &st) != 0 || !S_ISREG(st.st_mode)) {
        return false;
    }
    *size = (uint64_t)st.st_size;
    return true;
}

static bool mkdirs(const char *path)
{
    char buf[1024];
    size_t n = strlen(path);
    if (n == 0 || n >= sizeof(buf)) {
        return false;
    }
    memcpy(buf, path, n + 1);
    for (size_t i = 1; i <= n; i++) {
        if (buf[i] == '/' || buf[i] == '\\' || buf[i] == '\0') {
            char c = buf[i];
            buf[i] = '\0';
            if (!is_dir(buf) && host_mkdir(buf) != 0 && errno != EEXIST) {
                return false;
            }
            buf[i] = c;
        }
    }
    return is_dir(path);
}

static void remove_tree_contents(const char *dir)
{
    DIR *d = opendir(dir);
    if (!d) {
        return;
    }
    struct dirent *e;
    char child[1024];
    while ((e = readdir(d)) != NULL) {
        if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0) {
            continue;
        }
        snprintf(child, sizeof(child), "%s/%s", dir, e->d_name);
        if (is_dir(child)) {
            remove_tree_contents(child);
            rmdir(child);
        } else {
            remove(child);
        }
    }
    closedir(d);
}

static uint64_t tree_size(const char *dir)
{
    DIR *d = opendir(dir);
    if (!d) {
        return 0;
    }
    uint64_t total = 0;
    struct dirent *e;
    char child[1024];
    while ((e = readdir(d)) != NULL) {
        if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0) {
            continue;
        }
        snprintf(child, sizeof(child), "%s/%s", dir, e->d_name);
        uint64_t sz;
        if (is_dir(child)) {
            total += tree_size(child);
        } else if (host_file_size(child, &sz)) {
            total += sz;
        }
    }
    closedir(d);
    return total;
}

/* Map a FatFs path ("0:/logs/a.bin", "/a.bin", "a.bin") to a host path. */
static FRESULT map_path(const char *card_path, char *out, size_t out_len)
{
    if (!card_path) {
        return FR_INVALID_NAME;
    }
    const char *p = card_path;
    if (p[0] >= '0' && p[0] <= '9' && p[1] == ':') {
        if (p[0] != '0') {
            return FR_INVALID_DRIVE;
        }
        p += 2;
    }
    while (*p == '/' || *p == '\\') {
        p++;
    }
    /* No escaping the card folder. */
    if (strstr(p, "..") != NULL) {
        return FR_INVALID_NAME;
    }
    int n = snprintf(out, out_len, "%s/%s", sim_sd.cfg.root_dir, p);
    if (n < 0 || (size_t)n >= out_len) {
        return FR_INVALID_NAME;
    }
    return FR_OK;
}

static FRESULT check_card(void)
{
    if (!registered_fs) {
        return FR_NOT_ENABLED;
    }
    if (!sim_sd.inserted) {
        return FR_NOT_READY;
    }
    switch (sim_device_begin(&sim_sd.base)) {
    case SIM_FAULT_OFF:
        return FR_NOT_READY;
    case SIM_FAULT_BUS_ERROR:
        return FR_DISK_ERR;
    default:
        return FR_OK;
    }
}

/* Returns the slot behind fp, or NULL if the handle is closed or stale. */
static slot_t *slot_of(const FIL *fp)
{
    if (!fp || fp->generation != sim_sd.generation || fp->slot < 0 ||
        fp->slot >= SIM_SD_MAX_TRACKED || !slots[fp->slot].used) {
        return NULL;
    }
    return &slots[fp->slot];
}

static void close_all_slots(bool truncate_unsynced)
{
    for (int i = 0; i < SIM_SD_MAX_TRACKED; i++) {
        slot_t *s = &slots[i];
        if (!s->used) {
            continue;
        }
        fflush(s->f);
        if (truncate_unsynced && s->size > s->synced_size) {
            sim_sd.used_bytes -= s->size - s->synced_size;
            (void)host_truncate(s->f, s->synced_size);
        }
        fclose(s->f);
        memset(s, 0, sizeof(*s));
    }
    sim_sd.open_files = 0;
}

/* ----------------------------------------------------------- control API */

sim_sd_config_t sim_sd_default_config(const char *root_dir)
{
    sim_sd_config_t c = {
        .root_dir = root_dir,
        .capacity_bytes = 0,
        .write_bytes_per_s = 0.0,
        .sync_latency_ns = 0,
        .max_open_files = 0,
        .wipe_on_init = true,
    };
    return c;
}

static void sd_power_on(sim_device_t *dev)
{
    (void)dev;
    sim_sd_insert();
}

bool sim_sd_init(const sim_sd_config_t *cfg)
{
    /* Drop anything left open by a previous test. */
    close_all_slots(false);
    registered_fs = NULL;

    memset(&sim_sd, 0, sizeof(sim_sd));
    sim_device_init(&sim_sd.base, "sd");
    sim_sd.base.on_power_on = sd_power_on;
    sim_sd.cfg = *cfg;
    sim_sd.inserted = true;
    sim_sd.generation = 1;

    const char *root = cfg->root_dir;
    if (!root || root[0] == '\0' || strcmp(root, "/") == 0 || strcmp(root, ".") == 0) {
        return false;
    }
    if (!mkdirs(root)) {
        return false;
    }
    if (cfg->wipe_on_init) {
        remove_tree_contents(root);
    }
    sim_sd.used_bytes = tree_size(root);
    return true;
}

void sim_sd_power_cut(void)
{
    close_all_slots(true);
    sim_sd.inserted = false;
    sim_sd.generation++;
    sim_sd.power_cuts++;
    sim_device_trace(&sim_sd.base, "power cut: unsynced data dropped");
}

void sim_sd_insert(void)
{
    sim_sd.inserted = true;
}

const char *sim_sd_host_path(const char *card_path)
{
    static char buf[1024];
    if (map_path(card_path, buf, sizeof(buf)) != FR_OK) {
        buf[0] = '\0';
    }
    return buf;
}

/* --------------------------------------------------------------- FatFs API */

FRESULT f_mount(FATFS *fs, const TCHAR *path, BYTE opt)
{
    (void)path;
    if (!fs) {
        registered_fs = NULL; /* unmount */
        return FR_OK;
    }
    registered_fs = fs;
    fs->csize = SIM_SD_CLUSTER / FF_MIN_SS;
    fs->n_fatent = sim_sd.cfg.capacity_bytes
                       ? (DWORD)(sim_sd.cfg.capacity_bytes / SIM_SD_CLUSTER) + 2
                       : 0xFFFFFFF0u;
    fs->mounted = 0;
    if (opt == 0) {
        return FR_OK; /* lazy mount, like FatFs */
    }
    FRESULT r = check_card();
    if (r == FR_OK) {
        fs->mounted = 1;
    }
    sim_device_trace(&sim_sd.base, "f_mount -> %d", r);
    return r;
}

FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode)
{
    op_begin();
    if (!fp) {
        return op_end(FR_INVALID_OBJECT);
    }
    memset(fp, 0, sizeof(*fp));
    fp->slot = -1;
    FRESULT r = check_card();
    if (r != FR_OK) {
        return op_end(r);
    }
    int slot = -1;
    for (int i = 0; i < SIM_SD_MAX_TRACKED; i++) {
        if (!slots[i].used) {
            slot = i;
            break;
        }
    }
    if (slot < 0 ||
        (sim_sd.cfg.max_open_files && sim_sd.open_files >= sim_sd.cfg.max_open_files)) {
        return op_end(FR_TOO_MANY_OPEN_FILES);
    }

    char host[1024];
    r = map_path(path, host, sizeof(host));
    if (r != FR_OK) {
        return op_end(r);
    }
    if (is_dir(host)) {
        return op_end(FR_DENIED);
    }
    for (int i = 0; i < SIM_SD_MAX_TRACKED; i++) {
        if (slots[i].used && strcmp(slots[i].path, path) == 0 && (mode & FA_WRITE)) {
            return op_end(FR_LOCKED); /* FatFs FF_FS_LOCK: one writer per file */
        }
    }

    /* Parent folder must exist. */
    char parent[1024];
    snprintf(parent, sizeof(parent), "%s", host);
    char *slash = strrchr(parent, '/');
    if (slash) {
        *slash = '\0';
        if (!is_dir(parent)) {
            return op_end(FR_NO_PATH);
        }
    }

    uint64_t old_size = 0;
    bool exists = host_file_size(host, &old_size);
    BYTE how = mode & (FA_CREATE_NEW | FA_CREATE_ALWAYS | FA_OPEN_ALWAYS);

    if (how == FA_OPEN_EXISTING && !exists) {
        return op_end(FR_NO_FILE);
    }
    if ((how & FA_CREATE_NEW) && exists) {
        return op_end(FR_EXIST);
    }

    const char *fmode;
    if ((how & FA_CREATE_ALWAYS) || !exists) {
        if (!(mode & FA_WRITE)) {
            return op_end(exists ? FR_DENIED : FR_NO_FILE);
        }
        fmode = "w+b";
        if (exists) {
            sim_sd.used_bytes -= old_size;
        }
        old_size = 0;
    } else {
        fmode = (mode & FA_WRITE) ? "r+b" : "rb";
    }

    FILE *f = fopen(host, fmode);
    if (!f) {
        return op_end(FR_DENIED);
    }

    slot_t *s = &slots[slot];
    s->used = true;
    s->f = f;
    s->size = (FSIZE_t)old_size;
    s->synced_size = s->size;
    snprintf(s->path, sizeof(s->path), "%s", path);

    fp->slot = slot;
    fp->mode = mode;
    fp->generation = sim_sd.generation;
    fp->size = s->size;
    fp->fptr = ((mode & FA_OPEN_APPEND) == FA_OPEN_APPEND) ? s->size : 0;

    sim_sd.open_files++;
    sim_device_trace(&sim_sd.base, "f_open %s (mode 0x%02X, %lu bytes)", path, mode,
                     (unsigned long)fp->size);
    return op_end(FR_OK);
}

FRESULT f_write(FIL *fp, const void *buff, UINT btw, UINT *bw)
{
    op_begin();
    if (bw) {
        *bw = 0;
    }
    slot_t *s = slot_of(fp);
    if (!s) {
        return op_end(FR_INVALID_OBJECT);
    }
    if (!(fp->mode & FA_WRITE)) {
        return op_end(FR_DENIED);
    }

    sim_fault_mode_t fault = sim_device_begin(&sim_sd.base);
    if (fault == SIM_FAULT_OFF || fault == SIM_FAULT_BUS_ERROR) {
        sim_device_trace(&sim_sd.base, "f_write %s: disk error (injected)", s->path);
        return op_end(FR_DISK_ERR);
    }

    /* Capacity: FatFs reports a full disk as a short write, not an error. */
    UINT n = btw;
    if (sim_sd.cfg.capacity_bytes) {
        uint64_t end = (uint64_t)fp->fptr + n;
        uint64_t growth = end > s->size ? end - s->size : 0;
        uint64_t room = sim_sd.cfg.capacity_bytes > sim_sd.used_bytes
                            ? sim_sd.cfg.capacity_bytes - sim_sd.used_bytes
                            : 0;
        if (growth > room) {
            n -= (UINT)(growth - room);
        }
    }

    const uint8_t *src = buff;
    uint8_t *copy = NULL;
    if (fault == SIM_FAULT_CORRUPT && n > 0) {
        copy = malloc(n);
        if (copy) {
            memcpy(copy, buff, n);
            sim_corrupt(copy, n);
            src = copy;
        }
    }

    fseek(s->f, (long)fp->fptr, SEEK_SET);
    size_t written = fwrite(src, 1, n, s->f);
    free(copy);
    if (written != n) {
        return op_end(FR_DISK_ERR);
    }

    FSIZE_t end = fp->fptr + (FSIZE_t)written;
    if (end > s->size) {
        sim_sd.used_bytes += end - s->size;
        s->size = end;
    }
    fp->size = s->size;
    fp->fptr = end;

    if (sim_sd.cfg.write_bytes_per_s > 0.0) {
        sim_advance_ns((sim_time_ns_t)((double)written * 1e9 / sim_sd.cfg.write_bytes_per_s));
    }
    sim_sd.bytes_written += written;
    sim_sd.writes++;
    if (bw) {
        *bw = (UINT)written;
    }
    sim_device_trace(&sim_sd.base, "f_write %s: %u/%u bytes", s->path, (unsigned)written, btw);
    return op_end(FR_OK);
}

FRESULT f_read(FIL *fp, void *buff, UINT btr, UINT *br)
{
    op_begin();
    if (br) {
        *br = 0;
    }
    slot_t *s = slot_of(fp);
    if (!s) {
        return op_end(FR_INVALID_OBJECT);
    }
    if (!(fp->mode & FA_READ)) {
        return op_end(FR_DENIED);
    }
    sim_fault_mode_t fault = sim_device_begin(&sim_sd.base);
    if (fault == SIM_FAULT_OFF || fault == SIM_FAULT_BUS_ERROR) {
        return op_end(FR_DISK_ERR);
    }

    fflush(s->f);
    fseek(s->f, (long)fp->fptr, SEEK_SET);
    size_t got = fread(buff, 1, btr, s->f);
    if (fault == SIM_FAULT_CORRUPT) {
        sim_corrupt(buff, got);
    }
    fp->fptr += (FSIZE_t)got;
    if (br) {
        *br = (UINT)got;
    }
    return op_end(FR_OK);
}

FRESULT f_lseek(FIL *fp, FSIZE_t ofs)
{
    if (!slot_of(fp)) {
        return FR_INVALID_OBJECT;
    }
    if (ofs > fp->size && !(fp->mode & FA_WRITE)) {
        ofs = fp->size; /* read-only: clipped to the end, like FatFs */
    }
    fp->fptr = ofs;
    return FR_OK;
}

FRESULT f_sync(FIL *fp)
{
    op_begin();
    slot_t *s = slot_of(fp);
    if (!s) {
        return op_end(FR_INVALID_OBJECT);
    }
    sim_fault_mode_t fault = sim_device_begin(&sim_sd.base);
    if (fault == SIM_FAULT_OFF || fault == SIM_FAULT_BUS_ERROR) {
        sim_device_trace(&sim_sd.base, "f_sync %s: disk error (injected)", s->path);
        return op_end(FR_DISK_ERR);
    }
    fflush(s->f);
    s->synced_size = s->size;
    sim_advance_ns(sim_sd.cfg.sync_latency_ns);
    sim_sd.syncs++;
    sim_device_trace(&sim_sd.base, "f_sync %s (%lu bytes safe)", s->path,
                     (unsigned long)s->synced_size);
    return op_end(FR_OK);
}

FRESULT f_close(FIL *fp)
{
    slot_t *s = slot_of(fp);
    if (!s) {
        return FR_INVALID_OBJECT;
    }
    if (fp->mode & FA_WRITE) {
        FRESULT r = f_sync(fp);
        if (r != FR_OK) {
            return r; /* like FatFs: handle stays open if the sync failed */
        }
    }
    fclose(s->f);
    memset(s, 0, sizeof(*s));
    fp->slot = -1;
    if (sim_sd.open_files > 0) {
        sim_sd.open_files--;
    }
    return FR_OK;
}

FRESULT f_stat(const TCHAR *path, FILINFO *fno)
{
    FRESULT r = check_card();
    if (r != FR_OK) {
        return r;
    }
    char host[1024];
    r = map_path(path, host, sizeof(host));
    if (r != FR_OK) {
        return r;
    }
    uint64_t size = 0;
    bool file = host_file_size(host, &size);
    if (!file && !is_dir(host)) {
        return FR_NO_FILE;
    }
    if (fno) {
        fno->fsize = (FSIZE_t)size;
        const char *base = strrchr(path, '/');
        snprintf(fno->fname, sizeof(fno->fname), "%s", base ? base + 1 : path);
    }
    return FR_OK;
}

FRESULT f_unlink(const TCHAR *path)
{
    FRESULT r = check_card();
    if (r != FR_OK) {
        return r;
    }
    char host[1024];
    r = map_path(path, host, sizeof(host));
    if (r != FR_OK) {
        return r;
    }
    for (int i = 0; i < SIM_SD_MAX_TRACKED; i++) {
        if (slots[i].used && strcmp(slots[i].path, path) == 0) {
            return FR_LOCKED;
        }
    }
    uint64_t size = 0;
    if (host_file_size(host, &size)) {
        if (remove(host) != 0) {
            return FR_DENIED;
        }
        sim_sd.used_bytes -= size;
        return FR_OK;
    }
    if (is_dir(host)) {
        return rmdir(host) == 0 ? FR_OK : FR_DENIED;
    }
    return FR_NO_FILE;
}

FRESULT f_mkdir(const TCHAR *path)
{
    FRESULT r = check_card();
    if (r != FR_OK) {
        return r;
    }
    char host[1024];
    r = map_path(path, host, sizeof(host));
    if (r != FR_OK) {
        return r;
    }
    uint64_t size;
    if (is_dir(host) || host_file_size(host, &size)) {
        return FR_EXIST;
    }
    return host_mkdir(host) == 0 ? FR_OK : FR_NO_PATH;
}

FRESULT f_getfree(const TCHAR *path, DWORD *nclst, FATFS **fatfs)
{
    (void)path;
    FRESULT r = check_card();
    if (r != FR_OK) {
        return r;
    }
    if (nclst) {
        if (sim_sd.cfg.capacity_bytes) {
            uint64_t free_bytes = sim_sd.cfg.capacity_bytes > sim_sd.used_bytes
                                      ? sim_sd.cfg.capacity_bytes - sim_sd.used_bytes
                                      : 0;
            *nclst = (DWORD)(free_bytes / SIM_SD_CLUSTER);
        } else {
            *nclst = 0xFFFFFFF0u - 2;
        }
    }
    if (fatfs) {
        *fatfs = registered_fs;
    }
    return FR_OK;
}
