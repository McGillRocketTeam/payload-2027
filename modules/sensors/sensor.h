#ifndef SENSORS_SENSOR_H_
#define SENSORS_SENSOR_H_

/*
 * C counterpart of FC's modules/sensors/Sensor.h.
 *
 * FC uses a C++ base class. We keep the same contract with a naming
 * convention instead. Every sensor module <name> provides:
 *
 *   void            <name>_create(<name>_t *dev, ...);   no hardware access
 *   sensor_status_t <name>_init(<name>_t *dev);          configures the chip
 *   sensor_status_t <name>_poll(<name>_t *dev);          collects new data
 *   sensor_status_t <name>_status(const <name>_t *dev);
 *   bool            <name>_is_initialized(const <name>_t *dev);
 *
 * plus const getters that return cached values. See docs/driver_conventions.md.
 */

typedef enum {
    SENSOR_NOMINAL = 0,
    SENSOR_ERROR,
    SENSOR_NOT_PRESENT,
} sensor_status_t;

#endif /* SENSORS_SENSOR_H_ */
