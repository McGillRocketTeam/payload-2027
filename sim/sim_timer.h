#ifndef SIM_SIM_TIMER_H_
#define SIM_SIM_TIMER_H_

/*
 * The MCU's 32-bit timebase timer (TIM2/TIM5 on the F407) with input capture.
 *
 * It counts at f_hz *as measured by its clock source*: the MCU crystal or the
 * CSAC 10 MHz (both are just sim_osc_t, so either can be selected per run).
 * A capture channel attached to a pin latches the counter at the exact edge
 * time, then calls the ISR with that value. The ISR may run late (another
 * handler was busy); the captured value stays exact, as in hardware.
 *
 *   sim_timer_init(&tim, "tim2", &hse_osc, 84e6);
 *   sim_timer_capture(&tim, 0, &gps.pps.pin, true, my_pps_isr, &ctx);
 *
 * Not modelled: the 2-3 tick input synchroniser delay and capture filter.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sim_osc.h"
#include "sim_pin.h"

#define SIM_TIMER_CHANNELS 4

typedef void (*sim_timer_isr_t)(void *ctx, int channel, uint32_t capture);

typedef struct sim_timer sim_timer_t;

struct sim_timer {
    const char *name;
    sim_osc_t *clock;   /* NULL: ideal clock */
    double f_hz;
    double offset_ticks;
    struct {
        sim_timer_t *tim;
        int index;
        bool rising;
        sim_timer_isr_t isr;
        void *ctx;
        uint32_t ccr;
        uint32_t captures;
    } ch[SIM_TIMER_CHANNELS];
};

void sim_timer_init(sim_timer_t *tim, const char *name, sim_osc_t *clock, double f_hz);
bool sim_timer_capture(sim_timer_t *tim, int channel, sim_pin_t *pin, bool rising,
                       sim_timer_isr_t isr, void *ctx);

uint32_t sim_timer_count_at(sim_timer_t *tim, sim_time_ps_t t_ps);
/* Counter read for io structs. handle = sim_timer_t*. */
uint32_t sim_timer_read(void *handle);

#endif /* SIM_SIM_TIMER_H_ */
