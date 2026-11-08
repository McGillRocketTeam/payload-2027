#ifndef SENSORS_DRIVERS_TMP102_H_
#define SENSORS_DRIVERS_TMP102_H_

/*
 * TI TMP102 temperature sensor, register-level driver (I2C).
 *
 * Board independent: bus access goes through tmp102_ctx_t, filled in by the
 * board code (payload/MRT).
 */

#include <stddef.h>
#include <stdint.h>

typedef struct {
    /* Reads len bytes starting at register reg. Returns 0 on success. */
    int (*i2c_read)(void *handle, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);
    /* Writes len bytes starting at register reg. Returns 0 on success. */
    int (*i2c_write)(void *handle, uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len);
    /* 7-bit I2C address, set by how the ADD0 pin is wired. */
    uint8_t addr;
    void *handle;
} tmp102_ctx_t;

/* TODO(workshop): register map and configuration bits from the TMP102 datasheet. */

/* TODO(workshop): read the raw temperature register. Returns 0 on success. */
int tmp102_read_raw(const tmp102_ctx_t *ctx, int16_t *raw);

/* TODO(workshop): convert a raw reading to degrees Celsius (datasheet resolution). */
float tmp102_raw_to_celsius(int16_t raw);

#endif /* SENSORS_DRIVERS_TMP102_H_ */
