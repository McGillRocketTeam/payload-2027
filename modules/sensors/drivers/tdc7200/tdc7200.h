#ifndef SENSORS_DRIVERS_TDC7200_H_
#define SENSORS_DRIVERS_TDC7200_H_

/*
 * TI TDC7200 register-level driver.
 *
 * Board independent: every bus access goes through tdc7200_ctx_t, which the
 * board code (payload/MRT) fills with functions that wrap the real SPI.
 * Same idea as ST's stmdev_ctx_t used by FC's lsm6dsr_reg.c.
 */

#include <stddef.h>
#include <stdint.h>

typedef struct {
    /*
     * Full-duplex SPI transfer of len bytes. Chip select is asserted for the
     * whole transfer by the board code. Returns 0 on success.
     */
    int (*spi_transfer)(void *handle, const uint8_t *tx, uint8_t *rx, size_t len);
    void *handle;
} tdc7200_ctx_t;

/*
 * TODO(workshop): register map.
 * Fill in register addresses, reset values and bit fields from the TDC7200
 * datasheet (register map section) and commit the one-page summary to docs/.
 */

/* TODO(workshop): read one 8-bit register. Returns 0 on success. */
int tdc7200_read_reg(const tdc7200_ctx_t *ctx, uint8_t addr, uint8_t *value);

/* TODO(workshop): write one 8-bit register. Returns 0 on success. */
int tdc7200_write_reg(const tdc7200_ctx_t *ctx, uint8_t addr, uint8_t value);

/*
 * TODO(workshop): 24-bit result registers (TIME, CLOCK_COUNT, CALIBRATION)
 * and the time-of-flight calculation from the datasheet.
 */

#endif /* SENSORS_DRIVERS_TDC7200_H_ */
