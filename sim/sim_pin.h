#ifndef SIM_SIM_PIN_H_
#define SIM_SIM_PIN_H_

/*
 * A digital signal between chips: PPS lines, BITE / alarm outputs, TDC INTB,
 * IMU interrupts. One driver, up to SIM_PIN_MAX_LISTENERS listeners (the MCU,
 * a CSAC 1PPS input, a TDC START, ...), like a trace fanning out on the PCB.
 *
 * Edges carry their exact time in picoseconds. Listeners are called when the
 * edge happens (as an event), so an MCU interrupt handler attached here runs
 * at the edge, and a TDC model can measure edge-to-edge time to the ps.
 *
 * MCU side: sim_pin_read() matches a `bool read(void *handle)` GPIO function.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sim.h"

#define SIM_PIN_MAX_LISTENERS 8

typedef void (*sim_pin_fn)(void *ctx, bool rising, sim_time_ps_t t_edge_ps);

typedef struct {
    const char *name;
    bool level;
    sim_time_ps_t last_rise_ps;
    sim_time_ps_t last_fall_ps;
    uint32_t rises;
    uint32_t falls;
    struct {
        sim_pin_fn fn;
        void *ctx;
    } listeners[SIM_PIN_MAX_LISTENERS];
    int n_listeners;
} sim_pin_t;

void sim_pin_init(sim_pin_t *pin, const char *name, bool level);
bool sim_pin_listen(sim_pin_t *pin, sim_pin_fn fn, void *ctx);

/* Drive the pin now (t = sim_now_ps()) or at an exact time (scheduled). */
void sim_pin_set(sim_pin_t *pin, bool level);
/* Drive it right now, but stamp the edge with exact time t_ps (from an event). */
void sim_pin_set_exact(sim_pin_t *pin, bool level, sim_time_ps_t t_ps);
void sim_pin_set_at(sim_pin_t *pin, bool level, sim_time_ps_t t_ps);
/* Rising edge at t_ps, falling edge width_ps later. */
void sim_pin_pulse_at(sim_pin_t *pin, sim_time_ps_t t_ps, sim_time_ps_t width_ps);

/* GPIO read for io structs. handle = sim_pin_t*. */
bool sim_pin_read(void *handle);

#endif /* SIM_SIM_PIN_H_ */
