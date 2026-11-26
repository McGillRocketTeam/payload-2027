#ifndef SIM_SIM_H_
#define SIM_SIM_H_

/*
 * Payload hardware simulator (host builds only, never on the flight board).
 *
 * Every simulated chip plugs into the same I/O structs the real board code
 * fills in (tmp102_ctx_t, stmdev_ctx_t, gps_io_t, ...), so drivers and the
 * whole pipeline run unchanged against fake hardware.
 *
 * TIME
 * Sim time is true physical time: the perfect reference every oscillator is
 * compared against. GPS seconds land exactly on sim seconds. It is counted in
 * picoseconds (TDC-grade resolution) and only moves when a test calls
 * sim_advance_*() or a driver calls a delay (sim_delay_ms), so runs are
 * deterministic and as fast as the CPU allows. sim_set_pacing() optionally
 * slows it down to the wall clock (for watching live debug output).
 *
 * EVENTS (how interrupts happen)
 * Pin edges and periodic device output are events scheduled at exact times.
 * While time advances, each event fires in order with sim time set to the
 * event's time, so an interrupt handler attached to a pin runs at the edge,
 * between main-loop steps, exactly like on the board. If a handler itself
 * spends time (a blocking SPI read), events falling inside that time wait
 * until it returns: interrupt latency, as on the real MCU.
 *
 * Call sim_reset() in every test's setUp().
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef uint64_t sim_time_ns_t;
typedef uint64_t sim_time_ps_t;

#define SIM_US(x) ((sim_time_ns_t)(x) * 1000ULL)
#define SIM_MS(x) ((sim_time_ns_t)(x) * 1000000ULL)
#define SIM_S(x)  ((sim_time_ns_t)(x) * 1000000000ULL)

#define SIM_PS_PER_NS 1000ULL
#define SIM_PS_PER_S  1000000000000ULL

/* Time 0, empty event queue, default RNG seed, tracing and pacing off. */
void sim_reset(void);

sim_time_ns_t sim_now_ns(void);
sim_time_ps_t sim_now_ps(void);
double sim_now_s(void);

/* Move time forward, firing every event on the way. */
void sim_advance_ns(sim_time_ns_t dt);
void sim_advance_ps(sim_time_ps_t dt);
void sim_advance_ms(uint32_t ms);
void sim_run_until_ps(sim_time_ps_t t);

/*
 * *t = now + s, in picoseconds. Returns false (t untouched) if s is negative,
 * NaN, infinite, or too large to represent: callers treat that as "never".
 */
bool sim_ps_after_s(double s, sim_time_ps_t *t);

/* Drop-in for every io struct's delay_ms(handle, ms) member. */
void sim_delay_ms(void *handle, uint32_t ms);

/*
 * Pace sim time to the wall clock: 1.0 = real time, 10.0 = ten times faster,
 * 0 = off (default, as fast as possible).
 */
void sim_set_pacing(double speed);

/* ---- events ---- */

typedef void (*sim_event_fn)(void *ctx, uint32_t tag);

/*
 * Run fn(ctx, tag) at time t (picoseconds). A time already in the past runs
 * at the next opportunity. tag is free for the caller, typically a generation
 * counter so a rescheduled source can ignore its own stale events.
 * Returns false if the queue is full (SIM_MAX_EVENTS).
 */
#define SIM_MAX_EVENTS 4096
bool sim_schedule_ps(sim_time_ps_t t, sim_event_fn fn, void *ctx, uint32_t tag);
size_t sim_pending_events(void);

/*
 * Inside an event handler: the time the event was scheduled for. Usually
 * equal to sim_now_ps(); later if the event had to wait for a busy handler.
 * Physical edges use this, so their timestamp stays exact (like a hardware
 * input capture) even when the handler runs late.
 */
sim_time_ps_t sim_event_time_ps(void);

/* ---- random numbers ----
 * Used for sensor/oscillator noise, random faults and corruption. Everything
 * is seeded, so the same test gives the same result on every run. */

void sim_seed(uint32_t seed);
uint32_t sim_rand_u32(void);
double sim_rand_uniform(void); /* [0, 1) */
double sim_rand_gauss(void);   /* mean 0, std 1 */

/*
 * Counter-based noise: the same (seed, index) always gives the same value, no
 * matter what else ran before. Oscillators use this so their noise does not
 * depend on how often or in which order they are queried.
 */
double sim_hash_uniform(uint64_t seed, uint64_t index);
double sim_hash_gauss(uint64_t seed, uint64_t index);

/* Bus traffic tracing to stdout. Per device via sim_device_t.trace, or all. */
void sim_trace_all(bool enable);
bool sim_trace_all_enabled(void);

#endif /* SIM_SIM_H_ */
