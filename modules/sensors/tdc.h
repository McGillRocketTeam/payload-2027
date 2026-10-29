#ifndef SENSORS_TDC_H_
#define SENSORS_TDC_H_

/*
 * TDC sensor module: measures each clock's PPS against the CSAC PPS.
 * Wraps the TDC7200 driver behind the sensor contract in sensors/sensor.h.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sensors/drivers/tdc7200/tdc7200.h"
#include "sensors/sensor.h"

typedef struct {
    tdc7200_ctx_t bus;
    /* Drives the TDC7200 ENABLE pin. */
    void (*set_enable)(void *handle, bool high);
    void (*delay_ms)(void *handle, uint32_t ms);
    void *handle;
} tdc_io_t;

typedef struct {
    tdc_io_t io;
    bool initialized;
    sensor_status_t status;
    /* TODO(workshop): cached measurement results. */
} tdc_t;

/* Stores dependencies only. Must not touch hardware. */
void tdc_create(tdc_t *dev, const tdc_io_t *io);

sensor_status_t tdc_init(tdc_t *dev);
sensor_status_t tdc_poll(tdc_t *dev);
sensor_status_t tdc_status(const tdc_t *dev);
bool tdc_is_initialized(const tdc_t *dev);

#endif /* SENSORS_TDC_H_ */
