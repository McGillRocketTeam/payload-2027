#include "sensors/csac.h"

void csac_create(csac_t *dev, const csac_io_t *io)
{
    dev->io = *io;
    dev->initialized = false;
    dev->status = SENSOR_NOT_PRESENT;
}

sensor_status_t csac_init(csac_t *dev)
{
    /* TODO(workshop): request telemetry and check the CSAC answers. */
    dev->status = SENSOR_ERROR;
    return dev->status;
}

sensor_status_t csac_poll(csac_t *dev)
{
    /* TODO(workshop): drain uart_read, parse complete telemetry lines. */
    return dev->status;
}

sensor_status_t csac_status(const csac_t *dev)
{
    return dev->status;
}

bool csac_is_initialized(const csac_t *dev)
{
    return dev->initialized;
}

sensor_status_t csac_set_discipline(csac_t *dev, bool enable)
{
    /* TODO(workshop): send sa45s_build_discipline_command(). */
    (void)enable;
    return dev->status;
}
