#ifndef SIM_SIM_DEVICE_H_
#define SIM_SIM_DEVICE_H_

/*
 * Common base for every simulated device: power state, fault injection,
 * transaction counters and tracing. Each device embeds a sim_device_t as its
 * first member called `base`, so the same calls work on any of them:
 *
 *   sim_inject(&tmp.base, SIM_FAULT_OFF);                      unplug it now
 *   sim_inject_for(&imu.base, SIM_FAULT_BUS_ERROR, SIM_MS(200)); 200 ms of errors
 *   sim_inject_next(&gps.base, SIM_FAULT_CORRUPT, 3);           next 3 transfers
 *   sim_inject_random(&sd.base, SIM_FAULT_STALL, 0.01)->stall_ns = SIM_MS(250);
 *   sim_device_power(&mac.base, false);                         unpowered (state lost)
 *
 * What a fault means on the wire depends on the bus (see each device header),
 * but the rough meaning is the same everywhere:
 *
 *   OFF        device does not answer: I2C NACK, UART silent. SPI has no
 *              acknowledge, so the master still clocks bytes in: it reads
 *              whatever MISO idles at (sim_miso_t), and HAL reports success.
 *              Unlike power-off, the chip keeps its registers (loose wire).
 *   BUS_ERROR  the transfer fails: HAL error / timeout return code.
 *   CORRUPT    the transfer "succeeds" but one random bit is flipped.
 *   STUCK      the device answers, but its measurements freeze.
 *   STALL      the transfer succeeds after stall_ns of extra (virtual) time.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sim.h"

typedef enum {
    SIM_FAULT_NONE = 0,
    SIM_FAULT_OFF,
    SIM_FAULT_BUS_ERROR,
    SIM_FAULT_CORRUPT,
    SIM_FAULT_STUCK,
    SIM_FAULT_STALL,
} sim_fault_mode_t;

#define SIM_MAX_FAULTS 4
#define SIM_FOREVER    UINT64_MAX

typedef struct {
    sim_fault_mode_t mode;   /* SIM_FAULT_NONE = slot unused */
    sim_time_ns_t start_ns;  /* active from start_ns ... */
    sim_time_ns_t end_ns;    /* ... until end_ns (SIM_FOREVER = never ends) */
    uint32_t remaining;      /* 0 = unlimited, else number of transfers left */
    double probability;      /* chance per transfer while active (1.0 = always) */
    sim_time_ns_t stall_ns;  /* extra time per transfer for SIM_FAULT_STALL */
} sim_fault_t;

typedef struct sim_device sim_device_t;

struct sim_device {
    const char *name;
    bool powered;
    bool trace;
    sim_fault_t faults[SIM_MAX_FAULTS];

    /* Called on a power-off -> power-on edge, to restore reset values. */
    void (*on_power_on)(sim_device_t *dev);

    /* Statistics, handy in asserts. */
    uint32_t transfers;
    uint32_t faulted_transfers;
};

void sim_device_init(sim_device_t *dev, const char *name);

/* Power off: device stops answering. Power on: device resets (on_power_on). */
void sim_device_power(sim_device_t *dev, bool on);

/* Fault injection. Each returns the slot so extra fields can be tweaked, or
 * NULL if all SIM_MAX_FAULTS slots are taken. */
sim_fault_t *sim_inject(sim_device_t *dev, sim_fault_mode_t mode);
sim_fault_t *sim_inject_for(sim_device_t *dev, sim_fault_mode_t mode, sim_time_ns_t duration);
sim_fault_t *sim_inject_at(sim_device_t *dev, sim_fault_mode_t mode, sim_time_ns_t start,
                           sim_time_ns_t duration);
sim_fault_t *sim_inject_next(sim_device_t *dev, sim_fault_mode_t mode, uint32_t transfers);
sim_fault_t *sim_inject_random(sim_device_t *dev, sim_fault_mode_t mode, double probability);
void sim_clear_faults(sim_device_t *dev);

/* True if a STUCK fault is active right now (does not consume a transfer). */
bool sim_device_is_stuck(const sim_device_t *dev);

/*
 * For device implementations: call once at the start of every bus transfer.
 * Counts it, applies STALL time, and returns what the transfer should do.
 * An unpowered device always reports SIM_FAULT_OFF. STUCK is not returned
 * here (it does not affect the wire); devices check sim_device_is_stuck().
 */
sim_fault_mode_t sim_device_begin(sim_device_t *dev);

/* What an SPI master reads when no chip drives MISO (set per board). */
typedef enum {
    SIM_MISO_PULL_UP = 0, /* reads 0xFF (default) */
    SIM_MISO_PULL_DOWN,   /* reads 0x00 */
    SIM_MISO_FLOATING,    /* reads noise */
} sim_miso_t;

void sim_spi_idle_fill(sim_miso_t miso, uint8_t *buf, size_t len);

const char *sim_fault_name(sim_fault_mode_t mode);

/* Flip one random bit in buf. */
void sim_corrupt(uint8_t *buf, size_t len);

/* printf-style trace line, prefixed with sim time and device name. */
void sim_device_trace(const sim_device_t *dev, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

#endif /* SIM_SIM_DEVICE_H_ */
