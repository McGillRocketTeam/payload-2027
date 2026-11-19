#ifndef SIM_SIM_SCENARIO_H_
#define SIM_SIM_SCENARIO_H_

/*
 * A mission timeline played on a sim_board_t.
 *
 *   static sim_board_t board;
 *   static sim_scenario_t sc;
 *   sim_board_init(&board, &bcfg);
 *   sim_scenario_init(&sc, &board, NULL);            default mission
 *   sim_scenario_add(&sc, 600.0, "antenna blocked", sim_action_gps_sky_off, NULL);
 *   sim_scenario_run(&sc, sc.t_end_s, MRT_loop_once_wrapper, NULL, SIM_MS(1));
 *
 * Timeline (seconds after power-on):
 *   0                  power-on: GPS, CSACs, sensors (MACs off)
 *   macs_on_s          both MACs powered (DR2: >= 10 min before launch)
 *   t_launch_s         boost: thrust_g of specific force for burn_s
 *   t_burnout_s        coast (free fall, drag ignored) to apogee
 *   t_apogee_s         descent under parachute at descent_mps (1 g)
 *   t_landing_s        landed
 *   t_end_s            landing + after_landing_s (DR2: >= 10 min after GPS
 *                      reacquisition)
 * Each phase change is a "state" row in the run log.
 *
 * The flight profile is a simple placeholder. For realistic launch-detection
 * tests, replace board.accel_g[...] with a recorded flight (for example
 * sim_signal_load_csv on the Blue Raven log, with t0_s set to t_launch_s).
 */

#include <stdbool.h>
#include <stddef.h>

#include "sim_board.h"

typedef struct {
    double pad_wait_s;        /* power-on to launch */
    double macs_on_s;         /* power-on to MAC power-up */
    double thrust_g;          /* specific force during boost */
    double burn_s;            /* DR2: boost stays beyond 4 g for ~3.4 s */
    double descent_mps;       /* under parachute */
    double after_landing_s;   /* keep running after landing */
} sim_mission_config_t;

typedef struct sim_scenario sim_scenario_t;
typedef void (*sim_action_fn)(sim_scenario_t *sc, void *arg);

typedef struct {
    double t_s;
    const char *label;
    sim_action_fn fn;
    void *arg;
} sim_action_t;

#define SIM_MAX_ACTIONS 32

struct sim_scenario {
    sim_board_t *board;
    sim_mission_config_t m;
    double t_launch_s, t_burnout_s, t_apogee_s, t_landing_s, t_end_s;
    double v_burnout_mps, h_burnout_m, h_apogee_m;
    double pad_alt_m;
    int axis;
    bool gps_sky;
    sim_action_t actions[SIM_MAX_ACTIONS];
    int n_actions;
};

sim_mission_config_t sim_mission_default(void);

/* Installs the flight profile into the board and schedules the phases.
 * m may be NULL for the default mission. */
void sim_scenario_init(sim_scenario_t *sc, sim_board_t *b, const sim_mission_config_t *m);

/* Run fn(sc, arg) at t_s (seconds after power-on). */
bool sim_scenario_add(sim_scenario_t *sc, double t_s, const char *label, sim_action_fn fn,
                      void *arg);

/*
 * Run until until_s. If loop_once is given it is called every loop_period
 * (the firmware superloop, e.g. a wrapper around MRT_loop_once()); events
 * (interrupts, device output, scenario actions) fire in between.
 */
void sim_scenario_run(sim_scenario_t *sc, double until_s, void (*loop_once)(void *ctx), void *ctx,
                      sim_time_ns_t loop_period);

/* Ready-made actions. */
void sim_action_macs_on(sim_scenario_t *sc, void *arg);
void sim_action_macs_off(sim_scenario_t *sc, void *arg);
void sim_action_gps_sky_off(sim_scenario_t *sc, void *arg);
void sim_action_gps_sky_on(sim_scenario_t *sc, void *arg);
void sim_action_sd_power_cut(sim_scenario_t *sc, void *arg);

/* Flight state at time t (true seconds), for tests and plots. */
double sim_scenario_altitude_m(const sim_scenario_t *sc, double t_s);
double sim_scenario_vertical_speed_mps(const sim_scenario_t *sc, double t_s); /* up positive */

#endif /* SIM_SIM_SCENARIO_H_ */
