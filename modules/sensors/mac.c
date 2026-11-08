#include "sensors/mac.h"

void mac_create(mac_t *dev, const mac_io_t *io)
{
    dev->io = *io;
    dev->initialized = false;
    dev->status = SENSOR_NOT_PRESENT;
}

sensor_status_t mac_init(mac_t *dev)
{
    /* TODO(workshop): request telemetry and check the MAC answers. */
    dev->status = SENSOR_ERROR;
    return dev->status;
}

sensor_status_t mac_poll(mac_t *dev)
{
    /* TODO(workshop): drain uart_read, parse complete telemetry lines. */
    return dev->status;
}

sensor_status_t mac_status(const mac_t *dev)
{
    return dev->status;
}

bool mac_is_initialized(const mac_t *dev)
{
    return dev->initialized;
}

sensor_status_t mac_set_discipline(mac_t *dev, bool enable)
{
    /* TODO(workshop): send sa5x_build_discipline_command(). */
    (void)enable;
    return dev->status;
}
