#ifndef SIM_SIM_PPS_H_
#define SIM_SIM_PPS_H_

/*
 * A 1PPS output derived from an oscillator: a pulse each time the
 * oscillator's own clock crosses a whole second (plus `epoch_s`).
 *
 *   true time of edge n = the t where  t + x(t) = n + epoch_s
 *
 * With an ideal oscillator and epoch 0 the edges land exactly on true seconds
 * (the GPS PPS). Each edge is logged as "truth" with its real offset from the
 * nearest true second, so the run CSV holds the ground truth to compare the
 * firmware's TDC measurements against.
 *
 * Owners (CSAC, MAC, GPS) call sim_pps_reschedule() after anything that moves
 * the oscillator (steer, phase step, sync) so the next edge is recomputed.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sim_osc.h"
#include "sim_pin.h"

typedef struct sim_pps sim_pps_t;

struct sim_pps {
    const char *name;
    sim_osc_t *osc;          /* NULL: ideal, true time */
    sim_pin_t pin;
    double epoch_s;          /* local-time phase of the edges, [0, 1) */
    sim_time_ps_t width_ps;
    double jitter_s;         /* white phase noise per edge, rms */
    uint64_t jitter_seed;
    bool enabled;

    /* Called at every rising edge, with its exact true time. */
    void (*on_edge)(void *ctx, sim_time_ps_t t_ps);
    void *ctx;

    uint32_t gen;
    uint64_t next_n;
    sim_time_ps_t next_ps;
    sim_time_ps_t last_ps;
    uint64_t edges;
};

void sim_pps_init(sim_pps_t *pps, const char *name, sim_osc_t *osc, sim_time_ps_t width_ps);
void sim_pps_start(sim_pps_t *pps);
void sim_pps_stop(sim_pps_t *pps);
void sim_pps_reschedule(sim_pps_t *pps);

/* True time of the edge for local second n (no jitter). */
double sim_pps_edge_s(sim_pps_t *pps, int64_t n);
/* True time of this source's edge nearest to true time t_s (no jitter). */
double sim_pps_nearest_edge_s(sim_pps_t *pps, double t_s);

/*
 * Re-phase the output so its edge lands on the clock cycle (period
 * cycle_s, e.g. 100 ns for 10 MHz) nearest to true time ref_s. This is what a
 * 1PPS sync / JamSync does: the residual error is up to half a cycle.
 */
void sim_pps_align(sim_pps_t *pps, double ref_s, double cycle_s);

#endif /* SIM_SIM_PPS_H_ */
