#include "sim.h"

#include <math.h>
#include <stdio.h>

#include "sim_log.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#define SIM_DEFAULT_SEED 0x2027C0DEu

static sim_time_ps_t now_ps;
static uint32_t rng_state;
static bool trace_all;

/* ---------------------------------------------------------------- events */

typedef struct {
    sim_time_ps_t t;
    uint64_t seq; /* FIFO order for events at the same time */
    sim_event_fn fn;
    void *ctx;
    uint32_t tag;
} event_t;

static event_t heap[SIM_MAX_EVENTS];
static size_t heap_n;
static uint64_t heap_seq;
static bool dispatching;
static sim_time_ps_t event_time;

sim_time_ps_t sim_event_time_ps(void)
{
    return dispatching ? event_time : now_ps;
}

static bool before(const event_t *a, const event_t *b)
{
    return a->t < b->t || (a->t == b->t && a->seq < b->seq);
}

static void heap_swap(size_t a, size_t b)
{
    event_t t = heap[a];
    heap[a] = heap[b];
    heap[b] = t;
}

bool sim_schedule_ps(sim_time_ps_t t, sim_event_fn fn, void *ctx, uint32_t tag)
{
    if (heap_n == SIM_MAX_EVENTS || !fn) {
        sim_log("sim", "error", "event_queue_full", (double)heap_n,
                "event dropped: something schedules too much");
        return false;
    }
    size_t i = heap_n++;
    heap[i] = (event_t){.t = t, .seq = heap_seq++, .fn = fn, .ctx = ctx, .tag = tag};
    while (i > 0) {
        size_t p = (i - 1) / 2;
        if (!before(&heap[i], &heap[p])) {
            break;
        }
        heap_swap(i, p);
        i = p;
    }
    return true;
}

static event_t heap_pop(void)
{
    event_t top = heap[0];
    heap[0] = heap[--heap_n];
    size_t i = 0;
    for (;;) {
        size_t l = 2 * i + 1;
        size_t r = l + 1;
        size_t m = i;
        if (l < heap_n && before(&heap[l], &heap[m])) {
            m = l;
        }
        if (r < heap_n && before(&heap[r], &heap[m])) {
            m = r;
        }
        if (m == i) {
            break;
        }
        heap_swap(i, m);
        i = m;
    }
    return top;
}

size_t sim_pending_events(void)
{
    return heap_n;
}

/* ---------------------------------------------------------------- pacing */

static double pacing_speed;
static double pacing_wall0;
static sim_time_ps_t pacing_sim0;

static double wall_s(void)
{
#ifdef _WIN32
    LARGE_INTEGER f;
    LARGE_INTEGER c;
    QueryPerformanceFrequency(&f);
    QueryPerformanceCounter(&c);
    return (double)c.QuadPart / (double)f.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
#endif
}

static void pace(void)
{
    if (pacing_speed <= 0.0) {
        return;
    }
    double target = pacing_wall0 + (double)(now_ps - pacing_sim0) / 1e12 / pacing_speed;
    double wait = target - wall_s();
    if (wait <= 0.0) {
        return;
    }
#ifdef _WIN32
    Sleep((DWORD)(wait * 1000.0));
#else
    struct timespec ts = {.tv_sec = (time_t)wait, .tv_nsec = (long)((wait - floor(wait)) * 1e9)};
    nanosleep(&ts, NULL);
#endif
}

void sim_set_pacing(double speed)
{
    pacing_speed = speed;
    pacing_wall0 = wall_s();
    pacing_sim0 = now_ps;
}

/* ------------------------------------------------------------------ time */

void sim_reset(void)
{
    now_ps = 0;
    heap_n = 0;
    heap_seq = 0;
    dispatching = false;
    rng_state = SIM_DEFAULT_SEED;
    trace_all = false;
    pacing_speed = 0.0;
}

sim_time_ns_t sim_now_ns(void)
{
    return now_ps / SIM_PS_PER_NS;
}

sim_time_ps_t sim_now_ps(void)
{
    return now_ps;
}

double sim_now_s(void)
{
    return (double)now_ps / 1e12;
}

void sim_run_until_ps(sim_time_ps_t target)
{
    if (dispatching) {
        /* Time spent inside an event handler: later events wait for it. */
        if (target > now_ps) {
            now_ps = target;
        }
        return;
    }
    dispatching = true;
    while (heap_n > 0 && heap[0].t <= target) {
        event_t e = heap_pop();
        if (e.t > now_ps) {
            now_ps = e.t;
            pace();
        }
        event_time = e.t;
        e.fn(e.ctx, e.tag);
    }
    dispatching = false;
    if (target > now_ps) {
        now_ps = target;
        pace();
    }
}

void sim_advance_ps(sim_time_ps_t dt)
{
    sim_run_until_ps(now_ps + dt);
}

void sim_advance_ns(sim_time_ns_t dt)
{
    sim_run_until_ps(now_ps + dt * SIM_PS_PER_NS);
}

void sim_advance_ms(uint32_t ms)
{
    sim_advance_ns(SIM_MS(ms));
}

void sim_delay_ms(void *handle, uint32_t ms)
{
    (void)handle;
    sim_advance_ms(ms);
}

/* ---------------------------------------------------------------- random */

void sim_seed(uint32_t seed)
{
    rng_state = seed ? seed : SIM_DEFAULT_SEED;
}

uint32_t sim_rand_u32(void)
{
    if (rng_state == 0) {
        rng_state = SIM_DEFAULT_SEED;
    }
    uint32_t x = rng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng_state = x;
    return x;
}

double sim_rand_uniform(void)
{
    return (double)sim_rand_u32() / 4294967296.0;
}

static double box_muller(double u1, double u2)
{
    if (u1 < 1e-300) {
        u1 = 1e-300;
    }
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

double sim_rand_gauss(void)
{
    double u1 = sim_rand_uniform();
    double u2 = sim_rand_uniform();
    return box_muller(u1, u2);
}

static uint64_t splitmix64(uint64_t x)
{
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

double sim_hash_uniform(uint64_t seed, uint64_t index)
{
    uint64_t h = splitmix64(seed ^ splitmix64(index));
    return (double)(h >> 11) / 9007199254740992.0; /* 2^53 */
}

double sim_hash_gauss(uint64_t seed, uint64_t index)
{
    double u1 = sim_hash_uniform(seed, 2 * index);
    double u2 = sim_hash_uniform(seed, 2 * index + 1);
    return box_muller(u1, u2);
}

void sim_trace_all(bool enable)
{
    trace_all = enable;
}

bool sim_trace_all_enabled(void)
{
    return trace_all;
}
