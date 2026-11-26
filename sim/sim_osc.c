#include "sim_osc.h"

#include <math.h>
#include <string.h>

sim_osc_config_t sim_osc_ideal(void)
{
    sim_osc_config_t c;
    memset(&c, 0, sizeof(c));
    return c;
}

sim_osc_config_t sim_osc_ecs327txo(uint64_t seed)
{
    sim_osc_config_t c = sim_osc_ideal();
    c.y0 = 1.5e-6 * (2.0 * sim_hash_uniform(seed, 1) - 1.0);
    c.t_ref_c = 25.0;
    c.tempco2 = (5e-6 / (65.0 * 65.0)) * (2.0 * sim_hash_uniform(seed, 2) - 1.0);
    c.aging_per_day = (3e-6 / 365.0) * (2.0 * sim_hash_uniform(seed, 3) - 1.0);
    c.seed = seed;
    return c;
}

sim_osc_config_t sim_osc_dsc60xxb(uint64_t seed, double stability_ppm, double t_min_c,
                                  double t_max_c)
{
    double s = stability_ppm * 1e-6;
    double span = fmax(25.0 - t_min_c, t_max_c - 25.0);
    sim_osc_config_t c = sim_osc_ideal();
    c.y0 = 0.5 * s * (2.0 * sim_hash_uniform(seed, 1) - 1.0);
    c.t_ref_c = 25.0;
    c.tempco2 = (0.5 * s / (span * span)) * (2.0 * sim_hash_uniform(seed, 2) - 1.0);
    c.aging_per_day = (5e-6 / 365.0) * (2.0 * sim_hash_uniform(seed, 3) - 1.0);
    c.seed = seed;
    return c;
}

sim_osc_config_t sim_osc_mems_placeholder(uint64_t seed)
{
    return sim_osc_dsc60xxb(seed, 25.0, -40.0, 85.0); /* PENDING: see sim_osc.h */
}

void sim_osc_init(sim_osc_t *osc, const char *name, const sim_osc_config_t *cfg, double x0_s)
{
    memset(osc, 0, sizeof(*osc));
    osc->name = name;
    osc->cfg = *cfg;
    osc->t_s = sim_now_s();
    osc->x = x0_s;
}

static uint64_t grid_index(double t)
{
    return t <= 0.0 ? 0 : (uint64_t)floor(t / SIM_OSC_GRID_S + 1e-9);
}

/* Frequency error over grid interval k, excluding the random-walk part. */
static double interval_y(const sim_osc_t *o, uint64_t k)
{
    const sim_osc_config_t *c = &o->cfg;
    double tk = (double)k * SIM_OSC_GRID_S;
    sim_time_ns_t tk_ns = (sim_time_ns_t)(tk * 1e9);
    double y = c->y0 + c->aging_per_day * tk / 86400.0 + o->steer;

    if (c->adev_1s > 0.0) {
        y += c->adev_1s / sqrt(SIM_OSC_GRID_S) * sim_hash_gauss(c->seed, 2 * k);
    }
    if (c->tempco != 0.0 || c->tempco2 != 0.0) {
        double dt = (c->temperature_c ? sim_signal_eval_clean(c->temperature_c, tk_ns) : c->t_ref_c) -
                    c->t_ref_c;
        y += c->tempco * dt + c->tempco2 * dt * dt;
    }
    for (int i = 0; i < 3; i++) {
        if (c->g_sens[i] != 0.0 && c->accel_g[i]) {
            y += c->g_sens[i] * sim_signal_eval_clean(c->accel_g[i], tk_ns);
        }
    }
    return y;
}

static double rw_step(const sim_osc_t *o, uint64_t k)
{
    if (o->cfg.rw_per_sqrt_s <= 0.0) {
        return 0.0;
    }
    return o->cfg.rw_per_sqrt_s * sqrt(SIM_OSC_GRID_S) * sim_hash_gauss(o->cfg.seed, 2 * k + 1);
}

/* Integrate (t, x, rw) forward to t_to; exact for piecewise-constant y. */
static void integrate(const sim_osc_t *o, double *t, double *x, double *rw, double t_to)
{
    while (*t < t_to) {
        uint64_t k = grid_index(*t);
        double t_end = (double)(k + 1) * SIM_OSC_GRID_S;
        double t_next = t_end < t_to ? t_end : t_to;
        *x += (interval_y(o, k) + *rw) * (t_next - *t);
        if (t_next >= t_end) {
            *rw += rw_step(o, k + 1);
        }
        *t = t_next;
    }
}

static void bring_state_to(sim_osc_t *o, double t)
{
    integrate(o, &o->t_s, &o->x, &o->rw, t);
}

double sim_osc_error_at(sim_osc_t *o, sim_time_ps_t t_ps)
{
    double t = (double)t_ps / 1e12;
    if (t < o->t_s) {
        /* Slightly in the past: linear extrapolation, sub-ps error. */
        return o->x - (interval_y(o, grid_index(o->t_s)) + o->rw) * (o->t_s - t);
    }
    /* Keep the stored state within ~1 s so queries stay cheap, but never past
     * now: a steer applied now must act from now on. */
    double now = sim_now_s();
    double keep = t - 1.0 < now ? t - 1.0 : now;
    if (keep - o->t_s > 1.0) {
        bring_state_to(o, keep);
    }
    double tt = o->t_s;
    double x = o->x;
    double rw = o->rw;
    integrate(o, &tt, &x, &rw, t);
    return x;
}

double sim_osc_freq_at(sim_osc_t *o, sim_time_ps_t t_ps)
{
    double t = (double)t_ps / 1e12;
    (void)sim_osc_error_at(o, t_ps);
    double tt = o->t_s;
    double x = o->x;
    double rw = o->rw;
    if (t > tt) {
        integrate(o, &tt, &x, &rw, t);
    }
    return interval_y(o, grid_index(t)) + rw;
}

double sim_osc_true_time(sim_osc_t *o, double local_s)
{
    double t = local_s;
    for (int i = 0; i < 4; i++) {
        double tp = t < 0.0 ? 0.0 : t;
        t = local_s - sim_osc_error_at(o, (sim_time_ps_t)llround(tp * 1e12));
    }
    return t;
}

void sim_osc_set_steer(sim_osc_t *o, double steer)
{
    bring_state_to(o, sim_now_s());
    o->steer = steer;
}

void sim_osc_step_phase(sim_osc_t *o, double dx_s)
{
    bring_state_to(o, sim_now_s());
    o->x += dx_s;
}

void sim_osc_reconfigure(sim_osc_t *o, const sim_osc_config_t *cfg)
{
    bring_state_to(o, sim_now_s());
    o->cfg = *cfg;
}
