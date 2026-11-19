#include "sim_device.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "sim_log.h"

const char *sim_fault_name(sim_fault_mode_t m)
{
    static const char *names[] = {"none", "off", "bus_error", "corrupt", "stuck", "stall"};
    return (unsigned)m < sizeof(names) / sizeof(names[0]) ? names[m] : "?";
}

void sim_device_init(sim_device_t *dev, const char *name)
{
    memset(dev, 0, sizeof(*dev));
    dev->name = name;
    dev->powered = true;
}

void sim_device_power(sim_device_t *dev, bool on)
{
    bool was = dev->powered;
    dev->powered = on;
    if (on && !was && dev->on_power_on) {
        dev->on_power_on(dev);
    }
    if (on != was) {
        sim_log(dev->name, "power", on ? "on" : "off", on, NULL);
    }
    sim_device_trace(dev, "power %s", on ? "on" : "off");
}

static sim_fault_t *add_fault(sim_device_t *dev, sim_fault_mode_t mode, sim_time_ns_t start,
                              sim_time_ns_t end, uint32_t remaining, double probability)
{
    for (int i = 0; i < SIM_MAX_FAULTS; i++) {
        sim_fault_t *f = &dev->faults[i];
        if (f->mode == SIM_FAULT_NONE) {
            f->mode = mode;
            f->start_ns = start;
            f->end_ns = end;
            f->remaining = remaining;
            f->probability = probability;
            f->stall_ns = SIM_MS(250); /* typical worst-case SD write stall */
            sim_log(dev->name, "inject", sim_fault_name(mode), probability,
                    "start=%.6fs end=%s transfers=%u", (double)start / 1e9,
                    end == SIM_FOREVER ? "never" : "set", remaining);
            return f;
        }
    }
    return NULL;
}

sim_fault_t *sim_inject(sim_device_t *dev, sim_fault_mode_t mode)
{
    return add_fault(dev, mode, sim_now_ns(), SIM_FOREVER, 0, 1.0);
}

sim_fault_t *sim_inject_for(sim_device_t *dev, sim_fault_mode_t mode, sim_time_ns_t duration)
{
    return sim_inject_at(dev, mode, sim_now_ns(), duration);
}

sim_fault_t *sim_inject_at(sim_device_t *dev, sim_fault_mode_t mode, sim_time_ns_t start,
                           sim_time_ns_t duration)
{
    sim_time_ns_t end = (duration == SIM_FOREVER || start + duration < start) ? SIM_FOREVER
                                                                              : start + duration;
    return add_fault(dev, mode, start, end, 0, 1.0);
}

sim_fault_t *sim_inject_next(sim_device_t *dev, sim_fault_mode_t mode, uint32_t transfers)
{
    if (transfers == 0) {
        return NULL;
    }
    return add_fault(dev, mode, sim_now_ns(), SIM_FOREVER, transfers, 1.0);
}

sim_fault_t *sim_inject_random(sim_device_t *dev, sim_fault_mode_t mode, double probability)
{
    return add_fault(dev, mode, sim_now_ns(), SIM_FOREVER, 0, probability);
}

void sim_clear_faults(sim_device_t *dev)
{
    memset(dev->faults, 0, sizeof(dev->faults));
    sim_log(dev->name, "inject", "clear", 0, NULL);
}

static bool in_window(const sim_fault_t *f, sim_time_ns_t now)
{
    return f->mode != SIM_FAULT_NONE && now >= f->start_ns && now < f->end_ns;
}

bool sim_device_is_stuck(const sim_device_t *dev)
{
    sim_time_ns_t now = sim_now_ns();
    for (int i = 0; i < SIM_MAX_FAULTS; i++) {
        if (dev->faults[i].mode == SIM_FAULT_STUCK && in_window(&dev->faults[i], now)) {
            return true;
        }
    }
    return false;
}

sim_fault_mode_t sim_device_begin(sim_device_t *dev)
{
    dev->transfers++;
    if (!dev->powered) {
        dev->faulted_transfers++;
        return SIM_FAULT_OFF;
    }

    sim_fault_mode_t result = SIM_FAULT_NONE;
    sim_time_ns_t now = sim_now_ns();

    for (int i = 0; i < SIM_MAX_FAULTS; i++) {
        sim_fault_t *f = &dev->faults[i];
        if (f->mode == SIM_FAULT_STUCK || !in_window(f, now)) {
            continue;
        }
        if (f->probability < 1.0 && sim_rand_uniform() >= f->probability) {
            continue;
        }
        sim_fault_mode_t mode = f->mode;
        if (mode == SIM_FAULT_STALL) {
            sim_advance_ns(f->stall_ns);
        }
        if (f->remaining > 0 && --f->remaining == 0) {
            f->mode = SIM_FAULT_NONE; /* used up, free the slot */
        }
        /* Strongest effect wins: OFF > BUS_ERROR > CORRUPT > STALL. */
        if (result == SIM_FAULT_NONE || mode == SIM_FAULT_OFF ||
            (mode == SIM_FAULT_BUS_ERROR && result != SIM_FAULT_OFF) ||
            (mode == SIM_FAULT_CORRUPT && result == SIM_FAULT_STALL)) {
            result = mode;
        }
    }

    if (result != SIM_FAULT_NONE) {
        if (result != SIM_FAULT_STALL) {
            dev->faulted_transfers++;
        }
        sim_log(dev->name, "fault", sim_fault_name(result), dev->transfers, NULL);
    }
    return result;
}

void sim_spi_idle_fill(sim_miso_t miso, uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        buf[i] = miso == SIM_MISO_PULL_UP     ? 0xFF
                 : miso == SIM_MISO_PULL_DOWN ? 0x00
                                              : (uint8_t)sim_rand_u32();
    }
}

void sim_corrupt(uint8_t *buf, size_t len)
{
    if (len == 0) {
        return;
    }
    uint32_t bit = sim_rand_u32() % (uint32_t)(len * 8);
    buf[bit / 8] ^= (uint8_t)(1u << (bit % 8));
}

void sim_device_trace(const sim_device_t *dev, const char *fmt, ...)
{
    if (!dev->trace && !sim_trace_all_enabled()) {
        return;
    }
    printf("[%12.6f ms] %-10s ", (double)sim_now_ns() / 1e6, dev->name ? dev->name : "?");
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
}
