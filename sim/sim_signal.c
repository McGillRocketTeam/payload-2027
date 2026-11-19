#include "sim_signal.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

sim_signal_t sim_signal_const(double value)
{
    sim_signal_t s = {0};
    s.kind = SIM_SIGNAL_CONST;
    s.offset = value;
    return s;
}

sim_signal_t sim_signal_ramp(double start, double slope_per_s)
{
    sim_signal_t s = {0};
    s.kind = SIM_SIGNAL_RAMP;
    s.offset = start;
    s.slope = slope_per_s;
    return s;
}

sim_signal_t sim_signal_sine(double offset, double amplitude, double freq_hz)
{
    sim_signal_t s = {0};
    s.kind = SIM_SIGNAL_SINE;
    s.offset = offset;
    s.amplitude = amplitude;
    s.freq_hz = freq_hz;
    return s;
}

sim_signal_t sim_signal_step(double before, double after, double t_step_s)
{
    sim_signal_t s = {0};
    s.kind = SIM_SIGNAL_STEP;
    s.offset = before;
    s.amplitude = after - before;
    s.t_step_s = t_step_s;
    return s;
}

sim_signal_t sim_signal_table(const double *t_s, const double *v, size_t n)
{
    sim_signal_t s = {0};
    s.kind = SIM_SIGNAL_TABLE;
    s.table_t = t_s;
    s.table_v = v;
    s.table_n = n;
    return s;
}

sim_signal_t sim_signal_func(double (*fn)(void *ctx, double t_s), void *ctx)
{
    sim_signal_t s = {0};
    s.kind = SIM_SIGNAL_FUNC;
    s.fn = fn;
    s.ctx = ctx;
    return s;
}

static double table_eval(const sim_signal_t *s, double t)
{
    if (s->table_n == 0) {
        return 0.0;
    }
    if (t <= s->table_t[0]) {
        return s->table_v[0];
    }
    if (t >= s->table_t[s->table_n - 1]) {
        return s->table_v[s->table_n - 1];
    }
    /* Binary search for the segment containing t. */
    size_t lo = 0;
    size_t hi = s->table_n - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (s->table_t[mid] <= t) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    double t0 = s->table_t[lo];
    double t1 = s->table_t[hi];
    double f = (t1 > t0) ? (t - t0) / (t1 - t0) : 0.0;
    return s->table_v[lo] + f * (s->table_v[hi] - s->table_v[lo]);
}

double sim_signal_eval(const sim_signal_t *s, sim_time_ns_t now)
{
    double v = sim_signal_eval_clean(s, now);
    if (s->noise_std > 0.0) {
        v += s->noise_std * sim_rand_gauss();
    }
    return v;
}

double sim_signal_eval_clean(const sim_signal_t *s, sim_time_ns_t now)
{
    double t = (double)now / 1e9 - s->t0_s;
    double v;

    switch (s->kind) {
    case SIM_SIGNAL_RAMP:
        v = s->offset + s->slope * t;
        break;
    case SIM_SIGNAL_SINE:
        v = s->offset + s->amplitude * sin(2.0 * M_PI * s->freq_hz * t + s->phase_rad);
        break;
    case SIM_SIGNAL_STEP:
        v = s->offset + (t >= s->t_step_s ? s->amplitude : 0.0);
        break;
    case SIM_SIGNAL_TABLE:
        v = table_eval(s, t);
        break;
    case SIM_SIGNAL_FUNC:
        v = s->fn ? s->fn(s->ctx, t) : 0.0;
        break;
    case SIM_SIGNAL_CONST:
    default:
        v = s->offset;
        break;
    }
    return v;
}

static bool csv_field(const char *line, unsigned col, double *out)
{
    const char *p = line;
    for (unsigned i = 0; i < col; i++) {
        p = strchr(p, ',');
        if (!p) {
            return false;
        }
        p++;
    }
    char *end;
    double v = strtod(p, &end);
    if (end == p) {
        return false;
    }
    *out = v;
    return true;
}

size_t sim_signal_load_csv(sim_signal_t *sig, const char *path, unsigned t_col, unsigned v_col)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        return 0;
    }

    size_t cap = 1024;
    size_t n = 0;
    double *ts = malloc(cap * sizeof(double));
    double *vs = malloc(cap * sizeof(double));
    char line[512];

    while (ts && vs && fgets(line, sizeof(line), f)) {
        double t;
        double v;
        if (!csv_field(line, t_col, &t) || !csv_field(line, v_col, &v)) {
            continue;
        }
        if (n == cap) {
            cap *= 2;
            double *nt = realloc(ts, cap * sizeof(double));
            double *nv = realloc(vs, cap * sizeof(double));
            if (!nt || !nv) {
                free(nt ? nt : ts);
                free(nv ? nv : vs);
                ts = vs = NULL;
                break;
            }
            ts = nt;
            vs = nv;
        }
        ts[n] = t;
        vs[n] = v;
        n++;
    }
    fclose(f);

    if (!ts || !vs || n == 0) {
        free(ts);
        free(vs);
        return 0;
    }
    *sig = sim_signal_table(ts, vs, n);
    return n;
}

void sim_signal_free(sim_signal_t *sig)
{
    if (sig->kind == SIM_SIGNAL_TABLE) {
        free((void *)sig->table_t);
        free((void *)sig->table_v);
    }
    *sig = sim_signal_const(0.0);
}
