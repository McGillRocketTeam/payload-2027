#ifndef SENSORS_MAC_H_
#define SENSORS_MAC_H_

/*
 * MAC sensor module for the Microchip SA53.
 * Polls serial telemetry (lock status, steer, temperature) and switches
 * 1PPS disciplining on or off. The PPS itself is measured by a TDC.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensors/drivers/sa5x/sa5x.h"
#include "sensors/sensor.h"

typedef struct {
    /* Non-blocking: copies up to max received bytes into buf, returns the count. */
    size_t (*uart_read)(void *handle, uint8_t *buf, size_t max);
    /* Returns 0 on success. */
    int (*uart_write)(void *handle, const uint8_t *buf, size_t len);
    void *handle;
} mac_io_t;

typedef struct {
    mac_io_t io;
    bool initialized;
    sensor_status_t status;
    sa5x_telemetry_t telemetry;
    /* TODO(workshop): receive line buffer. */
} mac_t;

/* Stores dependencies only. Must not touch hardware. */
void mac_create(mac_t *dev, const mac_io_t *io);

sensor_status_t mac_init(mac_t *dev);
sensor_status_t mac_poll(mac_t *dev);
sensor_status_t mac_status(const mac_t *dev);
bool mac_is_initialized(const mac_t *dev);

/* TODO(workshop): turn 1PPS disciplining on (pad) or off (launch). */
sensor_status_t mac_set_discipline(mac_t *dev, bool enable);

#endif /* SENSORS_MAC_H_ */
