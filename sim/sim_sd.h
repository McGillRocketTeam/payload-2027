#ifndef SIM_SIM_SD_H_
#define SIM_SIM_SD_H_

/*
 * Simulated SD card behind the FatFs API in sim/fatfs/ff.h.
 *
 * Files land in a host folder (cfg.root_dir), so logs written by the real
 * logging code can be opened, diffed, or fed to the utils/ CSV converter.
 *
 * What it models:
 *   - write throughput and per-sync latency, in virtual time
 *   - card capacity (f_write returns FR_OK with bw < btw when full, like FatFs)
 *   - power loss: sim_sd_power_cut() truncates every open file back to its last
 *     f_sync/f_close, invalidates handles, and unmounts. Same as real FatFs,
 *     where unsynced data and size updates are lost.
 *   - faults through sd.base (see sim_device.h):
 *       OFF        card missing / not answering: FR_NOT_READY
 *       BUS_ERROR  FR_DISK_ERR on the faulted operation
 *       CORRUPT    written data gets one bit flipped (tests your CRC)
 *       STALL      operation takes stall_ns longer (default 250 ms)
 *
 * Only one card exists (FatFs drive "0:"), reached through the global sim_sd.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sim_device.h"

typedef struct {
    const char *root_dir;           /* host folder that is the card's root */
    uint64_t capacity_bytes;        /* 0 = unlimited */
    double write_bytes_per_s;       /* 0 = instant */
    sim_time_ns_t sync_latency_ns;  /* extra time per f_sync / f_close */
    unsigned max_open_files;        /* FatFs FF_FS_LOCK-like limit, 0 = unlimited */
    bool wipe_on_init;              /* delete root_dir contents in sim_sd_init */
} sim_sd_config_t;

typedef struct {
    sim_device_t base;
    sim_sd_config_t cfg;
    bool inserted;
    uint32_t generation;
    uint64_t used_bytes;
    unsigned open_files;

    /* Statistics. */
    uint64_t bytes_written;
    uint32_t writes;
    uint32_t syncs;
    uint32_t power_cuts;
    sim_time_ns_t longest_op_ns;
} sim_sd_t;

extern sim_sd_t sim_sd;

sim_sd_config_t sim_sd_default_config(const char *root_dir);
/* Creates root_dir if needed. Returns false if the folder can't be used. */
bool sim_sd_init(const sim_sd_config_t *cfg);

/* Pull power / eject: unsynced data lost, open handles invalid, unmounted. */
void sim_sd_power_cut(void);
/* Card back (after a power cut or removal); firmware must f_mount again. */
void sim_sd_insert(void);

/* Host path of a file on the card, for tests (static buffer). */
const char *sim_sd_host_path(const char *card_path);

#endif /* SIM_SIM_SD_H_ */
