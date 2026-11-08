#ifndef SENSORS_IMU_H_
#define SENSORS_IMU_H_

/*
 * IMU sensor module for the ST LSM6DSR (same part as FC).
 * Provides acceleration (thrust axis) and vibration (all axes) data.
 * Wraps ST's lsm6dsr_reg driver, copied from flightComputer-2027.
 */

#include <stdbool.h>

#include "sensors/drivers/lsm6dsr/lsm6dsr_reg.h"
#include "sensors/sensor.h"

typedef struct {
    stmdev_ctx_t bus;
    bool initialized;
    sensor_status_t status;
    /* TODO(workshop): cached acceleration, angular rate and temperature readings. */
} imu_t;

/* Stores dependencies only. Must not touch hardware. */
void imu_create(imu_t *dev, const stmdev_ctx_t *bus);

sensor_status_t imu_init(imu_t *dev);
sensor_status_t imu_poll(imu_t *dev);
sensor_status_t imu_status(const imu_t *dev);
bool imu_is_initialized(const imu_t *dev);

#endif /* SENSORS_IMU_H_ */
