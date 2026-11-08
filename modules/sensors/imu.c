#include "sensors/imu.h"

void imu_create(imu_t *dev, const stmdev_ctx_t *bus)
{
    dev->bus = *bus;
    dev->initialized = false;
    dev->status = SENSOR_NOT_PRESENT;
}

sensor_status_t imu_init(imu_t *dev)
{
    /* TODO(workshop): check WHO_AM_I (lsm6dsr_device_id_get), reset, set ranges and output data rates. */
    dev->status = SENSOR_ERROR;
    return dev->status;
}

sensor_status_t imu_poll(imu_t *dev)
{
    /* TODO(workshop): read new samples when data is ready. */
    return dev->status;
}

sensor_status_t imu_status(const imu_t *dev)
{
    return dev->status;
}

bool imu_is_initialized(const imu_t *dev)
{
    return dev->initialized;
}
