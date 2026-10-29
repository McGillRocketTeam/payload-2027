#ifndef SENSORS_GPS_H_
#define SENSORS_GPS_H_

/*
 * GPS sensor module for the u-blox NEO-M9N.
 * Reads UBX frames over UART and caches time, fix and timepulse data.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensors/sensor.h"

typedef struct {
    /* Non-blocking: copies up to max received bytes into buf, returns the count. */
    size_t (*uart_read)(void *handle, uint8_t *buf, size_t max);
    /* Returns 0 on success. */
    int (*uart_write)(void *handle, const uint8_t *buf, size_t len);
    void (*delay_ms)(void *handle, uint32_t ms);
    void *handle;
} gps_io_t;

typedef struct {
    gps_io_t io;
    bool initialized;
    sensor_status_t status;
    /* TODO(workshop): UBX parser state and cached NAV-PVT / TIM-TP fields. */
} gps_t;

/* Stores dependencies only. Must not touch hardware. */
void gps_create(gps_t *dev, const gps_io_t *io);

sensor_status_t gps_init(gps_t *dev);
sensor_status_t gps_poll(gps_t *dev);
sensor_status_t gps_status(const gps_t *dev);
bool gps_is_initialized(const gps_t *dev);

#endif /* SENSORS_GPS_H_ */
