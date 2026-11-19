#ifndef SIM_DEVICES_SIM_MCP79511_H_
#define SIM_DEVICES_SIM_MCP79511_H_

/*
 * Simulated Microchip MCP79511 SPI real-time clock, per its data sheet
 * (DS20002300D, docs/). On the payload it turns a 32.768 kHz TCXO or MEMS
 * output (on X1, external clock mode) into a 1 Hz PPS on MFP.
 * Firmware side: mcp79511_ctx_t.spi_transfer = sim_mcp79511_spi_transfer,
 * handle = &rtc; the 1 Hz output is rtc.mfp.
 *
 * Modelled:
 *   - instructions READ, WRITE (RTCC 00h-1Fh and SRAM 20h-5Fh, each block
 *     wraps on itself, 60h+ no response), CLRRAM, SRREAD, SRWRITE, EEREAD,
 *     EEWRITE (8-byte pages, 128 bytes, BP<1:0> protection, WEL, 5 ms write
 *     cycle with WIP), EEWREN, EEWRDI, IDREAD, UNLOCK (0x55, 0xAA) + IDWRITE;
 *     EUI-48 preprogrammed at protected 02h-07h
 *   - POR values (WKDAY / DATE / MONTH = 1, CONTROL reads 80h), SRAM random
 *     at power-up (not initialised by POR)
 *   - timekeeping from the input clock: BCD seconds .. year, 12/24 h, leap
 *     years, weekday 1-7, hundredths with the 41/40 x 4.096 kHz algorithm,
 *     LPYR, OSCRUN (set after 32 cycles, cleared 1 ms after the clock stops)
 *   - EXTOSC (external clock on X1) or ST (crystal); the board feeds a clock,
 *     so only EXTOSC counts (ST alone is logged as an error)
 *   - digital trimming: +-2 x TRIMVAL cycles once a minute (CRSTRIM = 0) or
 *     every second (CRSTRIM = 1), TRIMSIGN 1 = add (seconds get shorter)
 *   - MFP square wave, 1 Hz (SQWFS 00, trimmed): rising edge on each second
 *     tick; every edge is logged as "truth" with its offset from the true second
 * Not modelled: alarms, VBAT switchover and power-fail timestamps, the 4.096 /
 * 8.192 / 32.768 kHz outputs (logged as unmodelled).
 * UNVERIFIED (not in the data sheet): where in the second the 1 Hz edge sits
 * (on the tick) and its duty cycle (50 %), which second of the minute gets
 * the trim (the one ending at :00), that the first tick comes one full second
 * after the clock starts, the MFP level when not in square-wave mode (high,
 * pulled up), and what ST does with an external clock on X1.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensors/drivers/mcp79511/mcp79511.h"
#include "sim_device.h"
#include "sim_osc.h"
#include "sim_pin.h"

typedef struct {
    sim_osc_t *input;         /* 32.768 kHz TCXO / MEMS on X1 (NULL: ideal) */
    bool input_present;       /* clock actually reaching X1 */
    uint8_t eui48[6];
    sim_miso_t miso;
} sim_mcp79511_config_t;

typedef struct {
    sim_device_t base;
    sim_mcp79511_config_t cfg;
    sim_pin_t mfp;

    uint8_t rtcc[32];
    uint8_t sram[64];
    uint8_t eeprom[128];
    uint8_t id[16];
    uint8_t status;           /* BP1 BP0 WEL WIP */
    int unlock_step;          /* 0, 1 after 0x55, 2 after 0xAA */
    sim_time_ps_t wip_until_ps;

    bool running;
    double start_local_s;     /* input clock's local time when counting started */
    double tick_cycle;        /* input cycle index of the next second tick */
    double last_tick_cycle;
    uint32_t gen;
    uint64_t ticks;
} sim_mcp79511_t;

sim_mcp79511_config_t sim_mcp79511_default_config(sim_osc_t *input);
void sim_mcp79511_init(sim_mcp79511_t *dev, const char *name, const sim_mcp79511_config_t *cfg);
void sim_mcp79511_power(sim_mcp79511_t *dev, bool on);

int sim_mcp79511_spi_transfer(void *handle, const uint8_t *tx, uint8_t *rx, size_t len);
mcp79511_ctx_t sim_mcp79511_ctx(sim_mcp79511_t *dev);

#endif /* SIM_DEVICES_SIM_MCP79511_H_ */
