#include "sim_pin.h"

#include <string.h>

#include "sim_log.h"

void sim_pin_init(sim_pin_t *pin, const char *name, bool level)
{
    memset(pin, 0, sizeof(*pin));
    pin->name = name;
    pin->level = level;
}

bool sim_pin_listen(sim_pin_t *pin, sim_pin_fn fn, void *ctx)
{
    if (pin->n_listeners == SIM_PIN_MAX_LISTENERS) {
        return false;
    }
    pin->listeners[pin->n_listeners].fn = fn;
    pin->listeners[pin->n_listeners].ctx = ctx;
    pin->n_listeners++;
    return true;
}

static void drive(sim_pin_t *pin, bool level, sim_time_ps_t t)
{
    if (level == pin->level) {
        return;
    }
    pin->level = level;
    if (level) {
        pin->rises++;
        pin->last_rise_ps = t;
    } else {
        pin->falls++;
        pin->last_fall_ps = t;
    }
    if (sim_log_edges_enabled()) {
        sim_log(pin->name, "edge", level ? "rising" : "falling", (double)t / 1e12, NULL);
    }
    for (int i = 0; i < pin->n_listeners; i++) {
        pin->listeners[i].fn(pin->listeners[i].ctx, level, t);
    }
}

void sim_pin_set(sim_pin_t *pin, bool level)
{
    drive(pin, level, sim_now_ps());
}

void sim_pin_set_exact(sim_pin_t *pin, bool level, sim_time_ps_t t_ps)
{
    drive(pin, level, t_ps);
}

/* tag carries the level; the exact time is the event time itself. */
static void on_event(void *ctx, uint32_t tag)
{
    drive(ctx, tag != 0, sim_event_time_ps());
}

void sim_pin_set_at(sim_pin_t *pin, bool level, sim_time_ps_t t_ps)
{
    if (t_ps <= sim_now_ps()) {
        sim_pin_set(pin, level);
        return;
    }
    sim_schedule_ps(t_ps, on_event, pin, level ? 1u : 0u);
}

void sim_pin_pulse_at(sim_pin_t *pin, sim_time_ps_t t_ps, sim_time_ps_t width_ps)
{
    sim_pin_set_at(pin, true, t_ps);
    sim_pin_set_at(pin, false, t_ps + width_ps);
}

bool sim_pin_read(void *handle)
{
    return ((sim_pin_t *)handle)->level;
}
