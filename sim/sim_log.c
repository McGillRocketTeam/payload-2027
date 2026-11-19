#include "sim_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

#ifdef _WIN32
#include <direct.h>
#define log_mkdir(p) _mkdir(p)
#else
#define log_mkdir(p) mkdir((p), 0777)
#endif

#include "sim.h"

#define MAX_SOURCES 64

static FILE *log_file;
static bool log_edges;

typedef struct {
    char source[24];
    char kind[12];
    uint32_t count;
} counter_t;

static counter_t counters[MAX_SOURCES];
static int n_counters;

static void count(const char *source, const char *kind)
{
    for (int i = 0; i < n_counters; i++) {
        if (strcmp(counters[i].source, source) == 0 && strcmp(counters[i].kind, kind) == 0) {
            counters[i].count++;
            return;
        }
    }
    if (n_counters < MAX_SOURCES) {
        counter_t *c = &counters[n_counters++];
        snprintf(c->source, sizeof(c->source), "%s", source);
        snprintf(c->kind, sizeof(c->kind), "%s", kind);
        c->count = 1;
    }
}

bool sim_log_open(const char *path)
{
    sim_log_close();
    n_counters = 0;
    /* Create missing parent folders. */
    char dir[512];
    snprintf(dir, sizeof(dir), "%s", path);
    for (char *p = dir + 1; *p; p++) {
        if (*p == '/') {
            *p = '\0';
            log_mkdir(dir);
            *p = '/';
        }
    }
    log_file = fopen(path, "w");
    if (!log_file) {
        return false;
    }
    fprintf(log_file, "t_s,source,kind,name,value,detail\n");
    return true;
}

void sim_log_close(void)
{
    if (!log_file) {
        return;
    }
    fclose(log_file);
    log_file = NULL;
    if (n_counters > 0) {
        printf("sim run summary (rows per source/kind):\n");
        for (int i = 0; i < n_counters; i++) {
            printf("  %-12s %-8s %u\n", counters[i].source, counters[i].kind, counters[i].count);
        }
    }
}

bool sim_log_is_open(void)
{
    return log_file != NULL;
}

void sim_log_edges(bool enable)
{
    log_edges = enable;
}

bool sim_log_edges_enabled(void)
{
    return log_edges && log_file;
}

void sim_log(const char *source, const char *kind, const char *name, double value,
             const char *fmt, ...)
{
    if (!log_file) {
        return;
    }
    source = source ? source : "?";
    count(source, kind);

    char detail[256] = "";
    if (fmt) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(detail, sizeof(detail), fmt, ap);
        va_end(ap);
        /* Keep the CSV parseable. */
        for (char *p = detail; *p; p++) {
            if (*p == ',' || *p == '\n' || *p == '\r') {
                *p = ';';
            }
        }
    }
    sim_time_ps_t t = sim_now_ps();
    fprintf(log_file, "%llu.%012llu,%s,%s,%s,%.12g,%s\n",
            (unsigned long long)(t / SIM_PS_PER_S), (unsigned long long)(t % SIM_PS_PER_S),
            source, kind, name ? name : "", value, detail);
}

uint32_t sim_log_count(const char *source, const char *kind)
{
    for (int i = 0; i < n_counters; i++) {
        if (strcmp(counters[i].source, source) == 0 && strcmp(counters[i].kind, kind) == 0) {
            return counters[i].count;
        }
    }
    return 0;
}
