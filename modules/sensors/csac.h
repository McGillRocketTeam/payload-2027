#ifndef SENSORS_CSAC_H_
#define SENSORS_CSAC_H_

/*
 * CSAC sensor module for the Microchip SA.45s.
 * Polls serial telemetry (lock status, steer, temperature) and switches
 * 1PPS disciplining on or off. The PPS itself is measured by a TDC.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensors/drivers/sa45s/sa45s.h"
#include "sensors/sensor.h"

typedef struct {
    /* Non-blocking: copies up to max received bytes into buf, returns the count. */
    size_t (*uart_read)(void *handle, uint8_t *buf, size_t max);
    /* Returns 0 on success. */
    int (*uart_write)(void *handle, const uint8_t *buf, size_t len);
    void *handle;
} csac_io_t;

typedef struct {
    csac_io_t io;
    bool initialized;
    sensor_status_t status;
    sa45s_telemetry_t telemetry;
    /* TODO(workshop): receive line buffer. */
} csac_t;

/* Stores dependencies only. Must not touch hardware. */
void csac_create(csac_t *dev, const csac_io_t *io);

sensor_status_t csac_init(csac_t *dev);
sensor_status_t csac_poll(csac_t *dev);
sensor_status_t csac_status(const csac_t *dev);
bool csac_is_initialized(const csac_t *dev);

/* TODO(workshop): turn 1PPS disciplining on (pad) or off (launch). */
sensor_status_t csac_set_discipline(csac_t *dev, bool enable);

#endif /* SENSORS_CSAC_H_ */
