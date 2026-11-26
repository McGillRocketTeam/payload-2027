#include "sim_clock.h"

#include <math.h>
#include <string.h>

#include "sim_log.h"

void sim_clock_init(sim_clock_t *c, const char *name, const sim_clock_config_t *cfg)
{
    memset(c, 0, sizeof(*c));
    c->name = name;
    c->cfg = *cfg;
    c->tau_s = 100.0;
    c->threshold_s = 20e-9;
    c->ok_since_s = -1.0;
    c->locked_y0_cal = cfg->locked.y0;
    sim_osc_init(&c->osc, name, &cfg->free_run, 0.0);
    sim_pps_init(&c->pps, name, &c->osc, cfg->pps_width_ps);
}

static void apply_steer(sim_clock_t *c)
{
    double s = c->steer_user + c->steer_disc;
    if (c->cfg.steer_range > 0.0 && fabs(s) > c->cfg.steer_range) {
        s = s > 0 ? c->cfg.steer_range : -c->cfg.steer_range;
    }
    if (c->cfg.steer_res > 0.0) {
        s = round(s / c->cfg.steer_res) * c->cfg.steer_res; /* what the hardware applies */
    }
    sim_osc_set_steer(&c->osc, c->locked ? s : 0.0);
    sim_pps_reschedule(&c->pps);
}

void sim_clock_power(sim_clock_t *c, bool on)
{
    if (on == c->powered) {
        return;
    }
    c->powered = on;
    if (!on) {
        sim_pps_stop(&c->pps);
        c->locked = false;
        c->ref_seen = false;
        c->phase_valid = false;
        c->sync_pending = false;
        return;
    }
    /* Fresh start: TCXO free-running, output at an arbitrary phase. After the
     * first power-up the atomic frequency comes back within +-retrace. */
    c->cfg.locked.y0 = c->locked_y0_cal;
    if (c->power_ups > 0 && c->cfg.retrace > 0.0) {
        c->cfg.locked.y0 +=
            c->cfg.retrace * (2.0 * sim_hash_uniform(c->cfg.locked.seed ^ 0x7E7, c->power_ups) - 1.0);
    }
    c->power_ups++;
    c->locked = false;
    c->steer_user = c->steer_disc = c->y_int = 0.0;
    c->disc_started = false;
    c->disc = SIM_DISC_ACQUIRING;
    c->ok_since_s = -1.0;
    sim_osc_init(&c->osc, c->name, &c->cfg.free_run, 0.0);
    c->pps.epoch_s = sim_hash_uniform(c->cfg.free_run.seed ^ 0x5050, sim_now_ps());
    sim_pps_start(&c->pps);
}

void sim_clock_set_locked(sim_clock_t *c, bool locked)
{
    if (locked == c->locked) {
        return;
    }
    c->locked = locked;
    sim_osc_reconfigure(&c->osc, locked ? &c->cfg.locked : &c->cfg.free_run);
    apply_steer(c);
    sim_log(c->name, "state", "atomic_lock", locked, NULL);
}

static void ref_listener(void *ctx, bool rising, sim_time_ps_t t)
{
    if (rising) {
        sim_clock_ref_edge(ctx, t);
    }
}

void sim_clock_attach_ref(sim_clock_t *c, sim_pin_t *ref)
{
    sim_pin_listen(ref, ref_listener, c);
}

bool sim_clock_ref_present(const sim_clock_t *c)
{
    return c->ref_seen && sim_now_ps() - c->last_ref_ps < 1500ULL * 1000000000ULL;
}

static void do_sync(sim_clock_t *c, double t_ref)
{
    sim_pps_align(&c->pps, t_ref, 1.0 / c->cfg.rf_hz);
    c->syncs++;
    sim_log(c->name, "state", "sync", t_ref, "pps re-phased to reference");
}

static void servo(sim_clock_t *c)
{
    /* Positive err = own edge early. Steering y up moves own edges earlier. */
    double err = c->phase_s + c->cable_s;
    /* Poles at -1/tau and -2.6/tau: s^2 + 3.6 s/tau + 2.6/tau^2. */
    double w = 1.0 / c->tau_s;
    c->y_int += -err * 2.6 * w * w;
    double before = c->steer_disc;
    c->steer_disc = c->y_int - 3.6 * err * w;
    double lim = c->cfg.disc_limit;
    c->disc_at_limit = false;
    if (lim > 0.0 && fabs(c->steer_disc) > lim) {
        c->steer_disc = c->steer_disc > 0 ? lim : -lim;
        c->y_int = c->steer_disc; /* anti-windup */
        c->disc_at_limit = true;
    }
    c->last_correction = c->steer_disc - before;
    apply_steer(c);

    double now = sim_now_s();
    sim_disc_state_t old = c->disc;
    if (fabs(err) < c->threshold_s) {
        if (c->ok_since_s < 0.0) {
            c->ok_since_s = now;
        }
        if (now - c->ok_since_s >= 2.0 * c->tau_s) {
            c->disc = SIM_DISC_LOCKED;
        }
    } else {
        c->ok_since_s = -1.0;
        c->disc = SIM_DISC_ACQUIRING;
    }
    if (c->disc != old) {
        sim_log(c->name, "state", "discipline", c->disc, NULL);
    }
}

void sim_clock_ref_edge(sim_clock_t *c, sim_time_ps_t t_ps)
{
    if (!c->powered) {
        return;
    }
    double t_ref = (double)t_ps / 1e12;
    bool was_holdover = c->disc == SIM_DISC_HOLDOVER;
    c->ref_seen = true;
    c->last_ref_ps = t_ps;

    if (c->sync_pending) {
        do_sync(c, t_ref);
        c->sync_pending = false;
        c->sync_result = 1;
    } else if (c->mode == SIM_REF_AUTOSYNC) {
        do_sync(c, t_ref);
    }

    if (c->mode == SIM_REF_OFF) {
        c->phase_valid = false;
        return;
    }

    double own = sim_pps_nearest_edge_s(&c->pps, t_ref);
    double phase = t_ref - own;
    double res = c->cfg.meter_res_s;
    if (c->mode == SIM_REF_MEASURE && c->cfg.meter_fine_range_s > 0.0 &&
        fabs(phase) > c->cfg.meter_fine_range_s) {
        res = c->cfg.meter_coarse_res_s; /* extended-range meter */
    }
    if (res > 0.0) {
        phase = round(phase / res) * res;
    }
    c->phase_s = phase;
    c->phase_valid = true;

    if (c->mode != SIM_REF_DISCIPLINE || !c->locked) {
        return;
    }
    if (!c->disc_started) {
        /* Disciplining starts with a sync, then steers. */
        c->disc_started = true;
        c->y_int = c->steer_disc = 0.0;
        do_sync(c, t_ref);
        c->phase_s = 0.0;
        c->disc = SIM_DISC_ACQUIRING;
        return;
    }
    if (was_holdover) {
        c->disc = SIM_DISC_ACQUIRING;
        sim_log(c->name, "state", "discipline", c->disc, "reference back");
    }
    if (c->cfg.jam_limit_s > 0.0 && fabs(phase) > c->cfg.jam_limit_s) {
        do_sync(c, t_ref);
        c->phase_s = 0.0;
        if (c->cfg.jam_restarts_servo) {
            c->y_int = c->steer_disc = 0.0;
            c->ok_since_s = -1.0;
            c->disc = SIM_DISC_ACQUIRING;
            apply_steer(c);
        }
        return;
    }
    servo(c);
}

void sim_clock_set_mode(sim_clock_t *c, sim_ref_mode_t mode)
{
    if (mode == c->mode) {
        return;
    }
    c->mode = mode;
    if (mode == SIM_REF_DISCIPLINE) {
        c->disc_started = false; /* (re)enabling re-initialises */
        c->ok_since_s = -1.0;
        c->disc = SIM_DISC_ACQUIRING;
    } else {
        c->steer_disc = c->y_int = 0.0;
        apply_steer(c);
    }
}

void sim_clock_request_sync(sim_clock_t *c, double timeout_s)
{
    c->sync_pending = true;
    c->sync_result = 0;
    c->sync_deadline_ps = sim_now_ps() + (sim_time_ps_t)(timeout_s * 1e12);
}

void sim_clock_set_user_steer(sim_clock_t *c, double steer)
{
    c->steer_user = steer;
    apply_steer(c);
}

void sim_clock_latch(sim_clock_t *c)
{
    double total = c->steer_user + c->steer_disc;
    c->cfg.locked.y0 += total;
    c->locked_y0_cal += total;
    c->steer_user = c->steer_disc = c->y_int = 0.0;
    sim_osc_reconfigure(&c->osc, &c->cfg.locked);
    apply_steer(c);
}

void sim_clock_second(sim_clock_t *c)
{
    if (c->sync_pending && sim_now_ps() >= c->sync_deadline_ps) {
        c->sync_pending = false;
        c->sync_result = -1;
        sim_log(c->name, "error", "sync_timeout", 0, "no reference edge");
    }
    if (c->mode == SIM_REF_DISCIPLINE && c->disc_started && !sim_clock_ref_present(c) &&
        c->disc != SIM_DISC_HOLDOVER) {
        c->disc = SIM_DISC_HOLDOVER;
        c->ok_since_s = -1.0;
        sim_log(c->name, "state", "discipline", c->disc, "holdover: no reference");
    }
}
