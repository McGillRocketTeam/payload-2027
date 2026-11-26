#ifndef SIM_SIM_SIGNAL_H_
#define SIM_SIM_SIGNAL_H_

/*
 * A physical quantity as a function of sim time: what a simulated sensor
 * "measures". Every analog input of every device is one of these, so any
 * sensor can be fed a constant, a waveform, a recorded flight, or your own
 * function.
 *
 *   imu.cfg.accel[2] = sim_signal_const(1.0);                 1 g, at rest
 *   tmp.cfg.temperature = sim_signal_ramp(20.0, 0.01);        +0.01 C/s
 *   sim_signal_load_csv(&imu.cfg.accel[0], "blue_raven.csv", 0, 1);   replay
 *   imu.cfg.accel[0].noise_std = 0.02;                        add noise
 */

#include <stddef.h>

#include "sim.h"

typedef enum {
    SIM_SIGNAL_CONST = 0,
    SIM_SIGNAL_RAMP,     /* offset + slope * t */
    SIM_SIGNAL_SINE,     /* offset + amplitude * sin(2 pi freq t + phase) */
    SIM_SIGNAL_STEP,     /* offset before t_step, offset + amplitude after */
    SIM_SIGNAL_TABLE,    /* linear interpolation of (t, value) points, held at the ends */
    SIM_SIGNAL_FUNC,     /* fn(ctx, t) */
} sim_signal_kind_t;

typedef struct {
    sim_signal_kind_t kind;
    double offset;
    double amplitude;
    double slope;        /* per second, RAMP */
    double freq_hz;      /* SINE */
    double phase_rad;    /* SINE */
    double t_step_s;     /* STEP */
    double t0_s;         /* signal time 0 = sim time t0_s (shift a replay) */
    double noise_std;    /* gaussian noise added to every sample, any kind */

    const double *table_t; /* seconds, increasing */
    const double *table_v;
    size_t table_n;

    double (*fn)(void *ctx, double t_s);
    void *ctx;
} sim_signal_t;

sim_signal_t sim_signal_const(double value);
sim_signal_t sim_signal_ramp(double start, double slope_per_s);
sim_signal_t sim_signal_sine(double offset, double amplitude, double freq_hz);
sim_signal_t sim_signal_step(double before, double after, double t_step_s);
sim_signal_t sim_signal_table(const double *t_s, const double *v, size_t n);
sim_signal_t sim_signal_func(double (*fn)(void *ctx, double t_s), void *ctx);

double sim_signal_eval(const sim_signal_t *sig, sim_time_ns_t t);
/* Same, without noise_std: the underlying physical value (oscillators use it). */
double sim_signal_eval_clean(const sim_signal_t *sig, sim_time_ns_t t);

/*
 * Load a TABLE signal from a CSV file: time column (seconds) and value column,
 * 0-based. Lines that do not parse (headers, comments) are skipped. The table
 * is heap allocated; free it with sim_signal_free. Returns the number of
 * points, or 0 on failure.
 */
size_t sim_signal_load_csv(sim_signal_t *sig, const char *path, unsigned t_col, unsigned v_col);
void sim_signal_free(sim_signal_t *sig);

#endif /* SIM_SIM_SIGNAL_H_ */
