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

// register addresses
#define TMP102_TEMP_REG 0x00 // temperature register
#define TMP102_CONFIG_REG 0x01 // config register
#define TMP102_TLOW_REG 0x02 // low limit register
#define TMP102_THIGH_REG 0x03 // high limit register

// configuration bits (TODO)
#define TMP102_EM_CFG_BIT// extended mode bit (BIT IS 4 IN BYTE 2 of config)

typedef struct {
    /* Reads len bytes starting at register reg. Returns 0 on success. */
    int (*i2c_read)(void *handle, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);
    /* Writes len bytes starting at register reg. Returns 0 on success. */
    int (*i2c_write)(void *handle, uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len);
    /* 7-bit I2C address,` set by how the ADD0 pin is wired. */
    uint8_t addr;
    void *handle;
} tmp102_ctx_t;

/* TODO(workshop): register map and configuration bits from the TMP102 datasheet. 

/* TODO(workshop): read the raw temperature register. Returns 0 on success. */
int tmp102_read_raw(const tmp102_ctx_t *ctx, int16_t *raw);

/* TODO(workshop): convert a raw reading to degrees Celsius (datasheet resolution). */
float tmp102_raw_to_celsius(int16_t raw);

#endif /* SENSORS_DRIVERS_TMP102_H_ */
