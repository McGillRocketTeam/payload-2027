#ifndef SIM_SIM_OSC_H_
#define SIM_SIM_OSC_H_

/*
 * An oscillator's time error against true (sim) time.
 *
 * x(t) = oscillator time - true time, in seconds. It is the integral of the
 * fractional frequency error y(t):
 *
 *   y = y0 + aging * t                      offset and linear drift
 *     + white FM noise                      sets the 1 s Allan deviation
 *     + random-walk FM                      slow wander
 *     + tempco * (T - Tref) + tempco2 * (T - Tref)^2
 *     + sum(g_sens[i] * accel_g[i])         acceleration sensitivity (boost!)
 *     + steer                               set by disciplining / commands
 *
 * Environment and noise are evaluated on a fixed 100 ms grid of true time, and
 * noise comes from a per-oscillator counter-based RNG. So x(t) is the same no
 * matter how often or in what order it is queried, and the same seed always
 * gives the same oscillator. Steering and phase steps take effect at the exact
 * time they are applied.
 *
 * Values are plain fractional frequency (1e-9 = 1 ppb). Fill the config from
 * each part's data sheet.
 */

#include <stdint.h>

#include "sim.h"
#include "sim_signal.h"

#define SIM_OSC_GRID_S 0.1

typedef struct {
    double y0;              /* fractional frequency offset */
    double aging_per_day;   /* change of y per day */
    double adev_1s;         /* white FM level: Allan deviation at tau = 1 s */
    double rw_per_sqrt_s;   /* random-walk FM: y diffuses by this per sqrt(second) */
    double tempco;          /* per degC, around t_ref_c */
    double tempco2;         /* per degC^2 */
    double t_ref_c;
    double g_sens[3];       /* per g, X/Y/Z */
    const sim_signal_t *temperature_c; /* NULL: always t_ref_c */
    const sim_signal_t *accel_g[3];    /* NULL: 0 g on that axis */
    uint64_t seed;          /* different per oscillator */
} sim_osc_config_t;

typedef struct {
    const char *name;
    sim_osc_config_t cfg;
    /* Integrated state at true time t_s. */
    double t_s;
    double x;
    double rw;
    double steer;
} sim_osc_t;

/* An ideal oscillator: every field 0. */
sim_osc_config_t sim_osc_ideal(void);

/*
 * ECS-327TXO 32.768 kHz TCXO (docs/ECS-327TXO.pdf), one unit drawn from the
 * spec limits by seed: +-1.5 ppm at 25 C, +-5 ppm over -40..+85 C (modelled as
 * a parabola around 25 C hitting at most 5 ppm at the extremes), +-3 ppm first
 * year aging. Noise (ADEV) is not in the data sheet: 0 until measured.
 * Point temperature_c at the board temperature signal.
 */
sim_osc_config_t sim_osc_ecs327txo(uint64_t seed);

/*
 * PENDING (MEMS part number): the exact DSC6000B ordering code sets the
 * stability grade and temperature range. Until we have it, callers must pass
 * both explicitly; there is deliberately no default.
 *
 * Microchip DSC60xxB MEMS oscillator (docs/DSC60xxB-...-DS20006133.pdf), one
 * unit drawn from the spec limits by seed. stability_ppm is the ordering
 * code's grade (50, 25 or 20 ppm: total of initial tolerance + temperature +
 * supply) and t_min_c / t_max_c its temperature range (A -40..125, L -40..105,
 * I -40..85, E -20..70). The total is split as up to half initial offset and
 * up to half temperature curve (a parabola around 25 C), so it never exceeds
 * the grade. Aging +-5 ppm in the first year. Jitter: 39 ps RMS period jitter
 * is specified at 27 MHz only, so noise is 0 until measured at 32.768 kHz.
 */
sim_osc_config_t sim_osc_dsc60xxb(uint64_t seed, double stability_ppm, double t_min_c,
                                  double t_max_c);

/*
 * PENDING: placeholder part chosen until the real ordering code is known:
 * DSC6001CI2B-032K768 (OE with pull-up, standard drive, 3.2 x 2.5 mm VDFN,
 * industrial -40..85 C, +-25 ppm, 32.768 kHz). Change it here when we know.
 */
sim_osc_config_t sim_osc_mems_placeholder(uint64_t seed);

/* Starts at the current sim time with time error x0_s. */
void sim_osc_init(sim_osc_t *osc, const char *name, const sim_osc_config_t *cfg, double x0_s);

/* Time error x (seconds) at true time t_ps. */
double sim_osc_error_at(sim_osc_t *osc, sim_time_ps_t t_ps);
/* Fractional frequency error y at true time t_ps. */
double sim_osc_freq_at(sim_osc_t *osc, sim_time_ps_t t_ps);
/* True time (s) at which the oscillator's own clock reads local_s. */
double sim_osc_true_time(sim_osc_t *osc, double local_s);

/* Changes from now on (the state is first brought up to now). */
void sim_osc_set_steer(sim_osc_t *osc, double steer);
void sim_osc_step_phase(sim_osc_t *osc, double dx_s);
void sim_osc_reconfigure(sim_osc_t *osc, const sim_osc_config_t *cfg);

#endif /* SIM_SIM_OSC_H_ */
