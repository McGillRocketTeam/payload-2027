#include "sim_scenario.h"

#include <math.h>
#include <string.h>

#include "sim_log.h"

#define G0 9.80665

sim_mission_config_t sim_mission_default(void)
{
    sim_mission_config_t m = {
        .pad_wait_s = 30.0 * 60.0,
        .macs_on_s = 15.0 * 60.0,    /* 15 min before launch: DR2 asks >= 10 */
        .thrust_g = 6.0,             /* placeholder profile */
        .burn_s = 3.4,               /* DR2: beyond 4 g for ~3.4 s */
        .descent_mps = 6.0,
        .after_landing_s = 15.0 * 60.0,
    };
    return m;
}

/* ---------------------------------------------------------- kinematics */

double sim_scenario_vertical_speed_mps(const sim_scenario_t *sc, double t)
{
    if (t < sc->t_launch_s || t >= sc->t_landing_s) {
        return 0.0;
    }
    if (t < sc->t_burnout_s) {
        return (sc->m.thrust_g - 1.0) * G0 * (t - sc->t_launch_s);
    }
    if (t < sc->t_apogee_s) {
        return sc->v_burnout_mps - G0 * (t - sc->t_burnout_s);
    }
    return -sc->m.descent_mps;
}

double sim_scenario_altitude_m(const sim_scenario_t *sc, double t)
{
    double h;
    if (t < sc->t_launch_s) {
        h = 0.0;
    } else if (t < sc->t_burnout_s) {
        double dt = t - sc->t_launch_s;
        h = 0.5 * (sc->m.thrust_g - 1.0) * G0 * dt * dt;
    } else if (t < sc->t_apogee_s) {
        double dt = t - sc->t_burnout_s;
        h = sc->h_burnout_m + sc->v_burnout_mps * dt - 0.5 * G0 * dt * dt;
    } else if (t < sc->t_landing_s) {
        h = sc->h_apogee_m - sc->m.descent_mps * (t - sc->t_apogee_s);
    } else {
        h = 0.0;
    }
    return sc->pad_alt_m + h;
}

/* Specific force on the thrust axis, g. Free fall reads 0 g. */
static double thrust_axis_g(void *ctx, double t)
{
    const sim_scenario_t *sc = ctx;
    if (t >= sc->t_launch_s && t < sc->t_burnout_s) {
        return sc->m.thrust_g;
    }
    if (t >= sc->t_burnout_s && t < sc->t_apogee_s) {
        return 0.0;
    }
    return 1.0; /* pad, under parachute at terminal speed, landed */
}

static double alt_signal(void *ctx, double t)
{
    return sim_scenario_altitude_m(ctx, t);
}

static double vel_d_signal(void *ctx, double t)
{
    return -sim_scenario_vertical_speed_mps(ctx, t);
}

static double sky_signal(void *ctx, double t)
{
    (void)t;
    return ((const sim_scenario_t *)ctx)->gps_sky ? 1.0 : 0.0;
}

/* -------------------------------------------------------------- actions */

static void action_event(void *ctx, uint32_t tag)
{
    sim_scenario_t *sc = ctx;
    sim_action_t *a = &sc->actions[tag];
    sim_log("scenario", "state", a->label, a->t_s, NULL);
    if (a->fn) {
        a->fn(sc, a->arg);
    }
}

bool sim_scenario_add(sim_scenario_t *sc, double t_s, const char *label, sim_action_fn fn,
                      void *arg)
{
    if (sc->n_actions == SIM_MAX_ACTIONS) {
        return false;
    }
    int i = sc->n_actions++;
    sc->actions[i] = (sim_action_t){.t_s = t_s, .label = label, .fn = fn, .arg = arg};
    return sim_schedule_ps((sim_time_ps_t)llround(t_s * 1e12), action_event, sc, (uint32_t)i);
}

void sim_action_macs_on(sim_scenario_t *sc, void *arg)
{
    (void)arg;
    if (sc->board->cfg.kind == SIM_BOARD_FLIGHT) {
        sim_mac_power(&sc->board->mac[0], true);
        sim_mac_power(&sc->board->mac[1], true);
    }
}

void sim_action_macs_off(sim_scenario_t *sc, void *arg)
{
    (void)arg;
    sim_mac_power(&sc->board->mac[0], false);
    sim_mac_power(&sc->board->mac[1], false);
}

void sim_action_gps_sky_off(sim_scenario_t *sc, void *arg)
{
    (void)arg;
    sc->gps_sky = false;
}

void sim_action_gps_sky_on(sim_scenario_t *sc, void *arg)
{
    (void)arg;
    sc->gps_sky = true;
}

void sim_action_sd_power_cut(sim_scenario_t *sc, void *arg)
{
    (void)sc;
    (void)arg;
    sim_sd_power_cut();
}

/* ----------------------------------------------------------------- setup */

void sim_scenario_init(sim_scenario_t *sc, sim_board_t *b, const sim_mission_config_t *m)
{
    memset(sc, 0, sizeof(*sc));
    sc->board = b;
    sc->m = m ? *m : sim_mission_default();
    sc->axis = b->cfg.imu_thrust_axis;
    sc->gps_sky = true;
    sc->pad_alt_m = sim_signal_eval_clean(&b->gps.cfg.alt_m, 0);

    sc->t_launch_s = sc->m.pad_wait_s;
    sc->t_burnout_s = sc->t_launch_s + sc->m.burn_s;
    sc->v_burnout_mps = (sc->m.thrust_g - 1.0) * G0 * sc->m.burn_s;
    sc->h_burnout_m = 0.5 * (sc->m.thrust_g - 1.0) * G0 * sc->m.burn_s * sc->m.burn_s;
    sc->t_apogee_s = sc->t_burnout_s + sc->v_burnout_mps / G0;
    sc->h_apogee_m = sc->h_burnout_m + sc->v_burnout_mps * sc->v_burnout_mps / (2.0 * G0);
    sc->t_landing_s = sc->t_apogee_s + sc->h_apogee_m / sc->m.descent_mps;
    sc->t_end_s = sc->t_landing_s + sc->m.after_landing_s;

    /* The whole board follows the profile: IMU, GPS dynamics, oscillator g sensitivity. */
    b->accel_g[sc->axis] = sim_signal_func(thrust_axis_g, sc);
    b->gps.cfg.alt_m = sim_signal_func(alt_signal, sc);
    b->gps.cfg.vel_d = sim_signal_func(vel_d_signal, sc);
    b->gps.cfg.sky = sim_signal_func(sky_signal, sc);

    sim_scenario_add(sc, sc->m.macs_on_s, "macs_on", sim_action_macs_on, NULL);
    sim_scenario_add(sc, sc->t_launch_s, "launch", NULL, NULL);
    sim_scenario_add(sc, sc->t_burnout_s, "burnout", NULL, NULL);
    sim_scenario_add(sc, sc->t_apogee_s, "apogee", NULL, NULL);
    sim_scenario_add(sc, sc->t_landing_s, "landing", NULL, NULL);
    sim_log("scenario", "state", "mission", sc->h_apogee_m,
            "launch %.0f s, apogee %.0f m at %.1f s, landing %.0f s, end %.0f s", sc->t_launch_s,
            sc->h_apogee_m, sc->t_apogee_s, sc->t_landing_s, sc->t_end_s);
}

void sim_scenario_run(sim_scenario_t *sc, double until_s, void (*loop_once)(void *ctx), void *ctx,
                      sim_time_ns_t loop_period)
{
    (void)sc;
    sim_time_ps_t until = (sim_time_ps_t)llround(until_s * 1e12);
    if (!loop_once) {
        sim_run_until_ps(until);
        return;
    }
    while (sim_now_ps() < until) {
        loop_once(ctx);
        sim_advance_ns(loop_period);
    }
}
