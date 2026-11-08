#include "sensors/temperature.h"

void temperature_create(temperature_t *dev, const tmp102_ctx_t *bus)
{
    dev->bus = *bus;
    dev->initialized = false;
    dev->status = SENSOR_NOT_PRESENT;
}

sensor_status_t temperature_init(temperature_t *dev)
{
    /* TODO(workshop): check the sensor responds and set the conversion rate. */
    dev->status = SENSOR_ERROR;
    return dev->status;
}

sensor_status_t temperature_poll(temperature_t *dev)
{
    /* TODO(workshop): read and cache the temperature. */
    return dev->status;
}

sensor_status_t temperature_status(const temperature_t *dev)
{
    return dev->status;
}

bool temperature_is_initialized(const temperature_t *dev)
{
    return dev->initialized;
}
