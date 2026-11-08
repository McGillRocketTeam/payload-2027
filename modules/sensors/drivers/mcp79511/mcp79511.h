#ifndef SENSORS_DRIVERS_MCP79511_H_
#define SENSORS_DRIVERS_MCP79511_H_

/*
 * Microchip MCP79511 SPI real-time clock, register-level driver.
 *
 * Expected use (confirm with hardware): divide a TCXO or MEMS 32.768 kHz
 * output down to a 1 Hz PPS on the MFP pin, which then goes to a TDC7200.
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
} mcp79511_ctx_t;

/* TODO(workshop): register map and SPI instruction set from the MCP79511 datasheet. */

/* TODO(workshop): read one register. Returns 0 on success. */
int mcp79511_read_reg(const mcp79511_ctx_t *ctx, uint8_t addr, uint8_t *value);

/* TODO(workshop): write one register. Returns 0 on success. */
int mcp79511_write_reg(const mcp79511_ctx_t *ctx, uint8_t addr, uint8_t value);

/*
 * TODO(workshop): configure the external 32.768 kHz clock input and a 1 Hz
 * square wave on MFP. Returns 0 on success.
 */
int mcp79511_enable_1hz_output(const mcp79511_ctx_t *ctx);

#endif /* SENSORS_DRIVERS_MCP79511_H_ */
