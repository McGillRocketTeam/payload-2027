#ifndef SENSORS_DRIVERS_MAX2771_H_
#define SENSORS_DRIVERS_MAX2771_H_

/*
 * Analog Devices MAX2771 GNSS RF front end, configuration driver (SPI).
 *
 * Only covers configuration registers. How the I/Q samples reach the SD card
 * is still an open design question.
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
} max2771_ctx_t;

/* TODO(workshop): register map and SPI frame format from the MAX2771 datasheet. */

/* TODO(workshop): write one configuration register. Returns 0 on success. */
int max2771_write_reg(const max2771_ctx_t *ctx, uint16_t addr, uint32_t value);

/* TODO(workshop): read one configuration register. Returns 0 on success. */
int max2771_read_reg(const max2771_ctx_t *ctx, uint16_t addr, uint32_t *value);

#endif /* SENSORS_DRIVERS_MAX2771_H_ */
