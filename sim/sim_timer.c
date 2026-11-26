#include "sim_timer.h"

#include <math.h>
#include <string.h>

void sim_timer_init(sim_timer_t *tim, const char *name, sim_osc_t *clock, double f_hz)
{
    memset(tim, 0, sizeof(*tim));
    tim->name = name;
    tim->clock = clock;
    tim->f_hz = f_hz;
    /* The timer starts counting now, from 0. */
    tim->offset_ticks = -fmod(sim_now_s() * f_hz, 4294967296.0);
}

uint32_t sim_timer_count_at(sim_timer_t *tim, sim_time_ps_t t_ps)
{
    double t = (double)t_ps / 1e12;
    if (tim->clock) {
        t += sim_osc_error_at(tim->clock, t_ps);
    }
    double ticks = floor(t * tim->f_hz + tim->offset_ticks);
    return (uint32_t)(uint64_t)fmod(ticks < 0 ? ticks + 4294967296.0 : ticks, 4294967296.0);
}

uint32_t sim_timer_read(void *handle)
{
    return sim_timer_count_at(handle, sim_now_ps());
}

static void on_edge(void *ctx, bool rising, sim_time_ps_t t)
{
    __typeof__(((sim_timer_t *)0)->ch[0]) *c = ctx;
    if (rising != c->rising) {
        return;
    }
    c->ccr = sim_timer_count_at(c->tim, t);
    c->captures++;
    if (c->isr) {
        c->isr(c->ctx, c->index, c->ccr);
    }
}

bool sim_timer_capture(sim_timer_t *tim, int channel, sim_pin_t *pin, bool rising,
                       sim_timer_isr_t isr, void *ctx)
{
    if (channel < 0 || channel >= SIM_TIMER_CHANNELS) {
        return false;
    }
    tim->ch[channel].tim = tim;
    tim->ch[channel].index = channel;
    tim->ch[channel].rising = rising;
    tim->ch[channel].isr = isr;
    tim->ch[channel].ctx = ctx;
    return sim_pin_listen(pin, on_edge, &tim->ch[channel]);
}
