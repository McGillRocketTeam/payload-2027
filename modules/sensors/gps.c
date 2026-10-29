#include "sensors/gps.h"

void gps_create(gps_t *dev, const gps_io_t *io)
{
    dev->io = *io;
    dev->initialized = false;
    dev->status = SENSOR_NOT_PRESENT;
}

sensor_status_t gps_init(gps_t *dev)
{
    /* TODO(workshop): send UBX-CFG-VALSET to enable NAV-PVT and TIM-TP, disable NMEA. */
    dev->status = SENSOR_ERROR;
    return dev->status;
}

sensor_status_t gps_poll(gps_t *dev)
{
    /* TODO(workshop): drain uart_read into the UBX parser and decode complete frames. */
    return dev->status;
}

sensor_status_t gps_status(const gps_t *dev)
{
    return dev->status;
}

bool gps_is_initialized(const gps_t *dev)
{
    return dev->initialized;
}
