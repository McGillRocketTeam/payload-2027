#ifndef SIM_DEVICES_SIM_CSAC_H_
#define SIM_DEVICES_SIM_CSAC_H_

/*
 * Simulated Microchip SA.45s chip-scale atomic clock (CSAC), per the SA.45s
 * User's Guide (DS50003041, docs/). Firmware side: its UART plugs into
 * csac_io_t (uart = &csac.uart), BITE is csac.bite, 1PPS out is csac.clk.pps.pin,
 * 1PPS in is wired with sim_csac_attach_ref().
 *
 * Modelled:
 *   - serial 57600 8N1, ASCII; `!cmd[CRLF]` commands, single-character
 *     shortcuts (6 ^ F M S D U T m ?), ESC aborts, `?` for bad commands
 *   - optional checksum mode (mode bit 0x40): *XX required on commands and
 *     appended to replies; bad or missing checksum -> `*`
 *   - telemetry `!6` / `!^` with the exact field list of Table 3-3 (fw 1.09)
 *   - start-up: status 8 -> 0 over warmup_s, BITE high until 5 s after lock,
 *     lock loss on demand (sim_csac_lose_lock), BITE high within 1 s
 *   - !F (FA / FD / FL / F?) steering in 1e-15, over-range values clamped to the
 *     limit, applied at the hardware's ~1e-12 resolution, latch only when locked
 *   - !M mode register (non-volatile), 1PPS modes mutually exclusive
 *   - !S sync to the next 1PPS input edge (reply S, or E after 3 s)
 *   - phase measurement mode (0x0004) and disciplining (0x0010) with !D tau,
 *     !DC cable compensation, !m threshold, DiscOK 0/1/2
 *   - TOD: counts on 1PPS out, !TA / !TD / !T? (reply right after next 1PPS)
 *   - !> pulse width multiple, !U stored (ULP itself is not modelled)
 * Not modelled: analog tuning, ultra-low-power cycling, deferred command (@).
 *
 * Every number in sim_csac.c cites the user guide section or the SA.45s data
 * sheet (900-00744, Space CSAC) it comes from. UNVERIFIED marks the few that
 * neither document gives. Note: that data sheet is the Space CSAC's; check it
 * matches the variant we fly.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sim_clock.h"
#include "sim_device.h"
#include "sim_pin.h"
#include "sim_signal.h"
#include "sim_uart.h"

typedef struct {
    const char *serial;        /* YYMMCSXXXXX */
    const char *firmware;      /* telemetry Ver field */
    double warmup_s;           /* power-on to status 0 */
    double reply_delay_s;      /* command processing time */
    uint16_t mode;             /* mode register as found at first power-up */
    sim_signal_t temperature_c;/* unit temperature */
    sim_clock_config_t clock;
    bool bite_when_off;        /* BITE pin level while unpowered (board pull) */
} sim_csac_config_t;

typedef struct {
    sim_device_t base;
    sim_csac_config_t cfg;
    sim_uart_t uart;
    sim_pin_t bite;
    sim_clock_t clk;

    /* Non-volatile (survive power cycles). */
    uint16_t mode;
    int32_t tau_s;
    int32_t cable_100ps;
    int32_t threshold_ns;
    int32_t pulse_mult;
    uint32_t ulp_sleep, ulp_wake;

    /* Volatile. */
    int status;
    uint16_t alarm;
    int64_t steer_e15;
    uint32_t tod;
    sim_time_ps_t power_on_ps;
    sim_time_ps_t lock_ps;
    bool tod_query;
    bool sync_reply;
    uint32_t gen;
    double step_s;             /* current status countdown step */
    char line[160];
    size_t line_len;
    bool in_cmd;

    uint32_t bad_commands;
} sim_csac_t;

sim_csac_config_t sim_csac_default_config(uint64_t seed);
/* Starts unpowered; call sim_csac_power(dev, true). */
void sim_csac_init(sim_csac_t *dev, const char *name, const sim_csac_config_t *cfg);
void sim_csac_power(sim_csac_t *dev, bool on);
void sim_csac_attach_ref(sim_csac_t *dev, sim_pin_t *ref_pps);

/* Lose atomic lock now; it is re-acquired after relock_s. */
void sim_csac_lose_lock(sim_csac_t *dev, double relock_s);

#endif /* SIM_DEVICES_SIM_CSAC_H_ */
