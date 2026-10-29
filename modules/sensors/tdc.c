#include "sensors/tdc.h"

void tdc_create(tdc_t *dev, const tdc_io_t *io)
{
    dev->io = *io;
    dev->initialized = false;
    dev->status = SENSOR_NOT_PRESENT;
}

sensor_status_t tdc_init(tdc_t *dev)
{
    /* TODO(workshop): enable the chip, check a register with a known reset value, configure. */
    dev->status = SENSOR_ERROR;
    return dev->status;
}

sensor_status_t tdc_poll(tdc_t *dev)
{
    /* TODO(workshop): read results once INTB signals a finished measurement. */
    return dev->status;
}

sensor_status_t tdc_status(const tdc_t *dev)
{
    return dev->status;
}

bool tdc_is_initialized(const tdc_t *dev)
{
    return dev->initialized;
}
