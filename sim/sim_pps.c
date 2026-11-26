#include "sim_pps.h"

#include <math.h>
#include <string.h>

#include "sim_log.h"

void sim_pps_init(sim_pps_t *pps, const char *name, sim_osc_t *osc, sim_time_ps_t width_ps)
{
    memset(pps, 0, sizeof(*pps));
    pps->name = name;
    pps->osc = osc;
    pps->width_ps = width_ps;
    sim_pin_init(&pps->pin, name, false);
}

static double local_time(sim_pps_t *pps, double t)
{
    if (!pps->osc) {
        return t;
    }
    return t + sim_osc_error_at(pps->osc, (sim_time_ps_t)llround((t < 0 ? 0 : t) * 1e12));
}

double sim_pps_edge_s(sim_pps_t *pps, int64_t n)
{
    double local = (double)n + pps->epoch_s;
    return pps->osc ? sim_osc_true_time(pps->osc, local) : local;
}

double sim_pps_nearest_edge_s(sim_pps_t *pps, double t_s)
{
    double n = round(local_time(pps, t_s) - pps->epoch_s);
    return sim_pps_edge_s(pps, (int64_t)n);
}

static void fire(void *ctx, uint32_t tag);

static void schedule_next(sim_pps_t *pps)
{
    /* First local second whose edge is strictly in the future. Compared in
     * integer ps: a double compare can call an edge "future" that rounds to
     * the current ps, and it would fire again forever. */
    sim_time_ps_t now_ps = sim_now_ps();
    int64_t n = (int64_t)floor(local_time(pps, sim_now_s()) - pps->epoch_s);
    sim_time_ps_t t_ps;
    do {
        n++;
        double t = sim_pps_edge_s(pps, n);
        if (pps->jitter_s > 0.0) {
            t += pps->jitter_s * sim_hash_gauss(pps->jitter_seed, (uint64_t)n);
        }
        t_ps = t <= 0.0 ? 0 : (sim_time_ps_t)llround(t * 1e12);
    } while (t_ps <= now_ps);
    pps->next_n = (uint64_t)n;
    pps->next_ps = t_ps;
    sim_schedule_ps(pps->next_ps, fire, pps, pps->gen);
}

static void fire(void *ctx, uint32_t tag)
{
    sim_pps_t *pps = ctx;
    if (tag != pps->gen || !pps->enabled) {
        return; /* rescheduled or stopped since */
    }
    sim_time_ps_t t = pps->next_ps;
    pps->last_ps = t;
    pps->edges++;
    sim_pin_set_exact(&pps->pin, true, t);
    sim_pin_set_at(&pps->pin, false, t + pps->width_ps);

    if (sim_log_is_open()) {
        double ts = (double)t / 1e12;
        sim_log(pps->name, "truth", "pps_offset_s", ts - round(ts), "edge %llu",
                (unsigned long long)pps->next_n);
    }
    if (pps->on_edge) {
        pps->on_edge(pps->ctx, t);
    }
    if (tag == pps->gen && pps->enabled) {
        schedule_next(pps);
    }
}

void sim_pps_start(sim_pps_t *pps)
{
    if (pps->enabled) {
        return;
    }
    pps->enabled = true;
    pps->gen++;
    schedule_next(pps);
}

void sim_pps_stop(sim_pps_t *pps)
{
    pps->enabled = false;
    pps->gen++;
}

void sim_pps_reschedule(sim_pps_t *pps)
{
    if (!pps->enabled) {
        return;
    }
    pps->gen++;
    schedule_next(pps);
}

void sim_pps_align(sim_pps_t *pps, double ref_s, double cycle_s)
{
    double local = local_time(pps, ref_s);
    if (cycle_s > 0.0) {
        local = round(local / cycle_s) * cycle_s;
    }
    pps->epoch_s = local - floor(local);
    sim_pps_reschedule(pps);
}
