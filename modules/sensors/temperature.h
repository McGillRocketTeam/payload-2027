#ifndef SENSORS_TEMPERATURE_H_
#define SENSORS_TEMPERATURE_H_

/*
 * Temperature sensor module for the TI TMP102 (payload bay temperature).
 */

#include <stdbool.h>

#include "sensors/drivers/tmp102/tmp102.h"
#include "sensors/sensor.h"

typedef struct {
    tmp102_ctx_t bus;
    bool initialized;
    sensor_status_t status;
    /* TODO(workshop): cached temperature reading. */
} temperature_t;

/* Stores dependencies only. Must not touch hardware. */
void temperature_create(temperature_t *dev, const tmp102_ctx_t *bus);

sensor_status_t temperature_init(temperature_t *dev);
sensor_status_t temperature_poll(temperature_t *dev);
sensor_status_t temperature_status(const temperature_t *dev);
bool temperature_is_initialized(const temperature_t *dev);

#endif /* SENSORS_TEMPERATURE_H_ */
