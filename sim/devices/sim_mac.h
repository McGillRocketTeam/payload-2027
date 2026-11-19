#ifndef SIM_DEVICES_SIM_MAC_H_
#define SIM_DEVICES_SIM_MAC_H_

/*
 * Simulated Microchip SA5x (SA53) miniature atomic clock, per the MAC-SA5X
 * User's Guide (DS50002938, docs/). Firmware side: its UART plugs into
 * mac_io_t (uart = &mac.uart); BITE (low = locked) is mac.bite; ALARM is
 * mac.alarm (polarity: see alarm_active_high); 1PPS out is mac.clk.pps.pin.
 *
 * Modelled:
 *   - C3 protocol at 57600 8N1: {command#XX,arg|CC} -> [#XX=value|CC]<CR><LF>,
 *     optional sequence number and checksum, error replies [!n] (1, 2, 3, 100,
 *     101, 102)
 *   - power-on announcements [>Loading...] then [>Microchip SA5X] 3 s later;
 *     commands are ignored until then
 *   - the parameter table of Table 4-6 through get / set / add / upd, with
 *     read-only checks and ranges
 *   - warm-up ~300 s (LockProgress 0-100 %, Locked, BITE), acquisition failure
 *     on demand (sim_mac_fail_acquisition)
 *   - Alarms bitfield and ALARM pin with ackalm (pin stays low while any
 *     active alarm is unacknowledged); No PPS Input, Acquisition Failed
 *   - two 1PPS inputs (PpsSource), PhaseMetering or Disciplining (exclusive),
 *     TauPps0/1, DisciplineThresholdPps0/1, PhaseLimit JamSync, CableDelay,
 *     PpsOffset, PpsWidth, PpsQErr (applied to the next input pulse)
 *   - DigitalTuning steering (applied at ~1e-14), latch (only when locked),
 *     store / load / reset; Disciplining Range Warning when the servo needs
 *     more than +-2e-8
 *   - thermal: baseplate = ambient + ~15 C after warm-up, Temperature reads
 *     ~12.5 C above it, Temperature Warning and loss of lock above +75 C
 *   - TimeOfDay counting on 1PPS out
 * Not modelled: browse, extremes?, health?, BSL baud change, legacy SA.3Xm
 * compatibility mode (these answer [!1]).
 *
 * Every number in sim_mac.c cites the user guide or the MAC-SA5X data sheet
 * (DS00003348E, SA53 column). UNVERIFIED marks the few neither document gives.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sim_clock.h"
#include "sim_device.h"
#include "sim_pin.h"
#include "sim_signal.h"
#include "sim_uart.h"

typedef struct {
    const char *serial;
    const char *swrev;
    double warmup_s;
    double reply_delay_s;
    sim_signal_t temperature_c;   /* ambient; the baseplate warms ~15 C at start-up */
    sim_clock_config_t clock;
    bool bite_when_off;           /* pin levels while unpowered (board pulls) */
    bool alarm_when_off;
    /* ALARM polarity: the user guide says active low (default), the data sheet
     * says "Low = Normal Operation, High = Alarm". Check on the real unit. */
    bool alarm_active_high;
} sim_mac_config_t;

/* Settings that `store` saves and power-up / `load` restore. */
typedef struct {
    int64_t pps_offset_ns, pps_width_ns, cable_ns, pps_qerr_ps, phase_limit_ns;
    int64_t disciplining, pps_source, tau0, tau1, metering, thr0, thr1, analog_en;
} sim_mac_settings_t;

#define SIM_MAC_NPARAMS 33

typedef struct {
    sim_device_t base;
    sim_mac_config_t cfg;
    sim_uart_t uart;
    sim_pin_t bite;
    sim_pin_t alarm;
    sim_clock_t clk;

    sim_mac_settings_t set;
    sim_mac_settings_t stored;
    bool has_stored;

    bool app_running;
    bool acq_failed;
    uint32_t alarms;
    uint32_t acked;
    int64_t digital_tuning;
    int64_t qerr_next_ps;
    uint32_t tod;
    bool tod_pending;
    uint32_t tod_next;
    sim_time_ps_t power_on_ps;
    sim_time_ps_t lock_ps;
    double runtime_s, locktime_s;
    uint32_t gen;
    double hot_since_s;
    bool overheated;
    double upd_last[SIM_MAC_NPARAMS];

    char line[256];
    size_t line_len;
    bool in_cmd;
    uint32_t bad_commands;
} sim_mac_t;

sim_mac_config_t sim_mac_default_config(uint64_t seed);
/* Starts unpowered (MACs are off on the pad). */
void sim_mac_init(sim_mac_t *dev, const char *name, const sim_mac_config_t *cfg);
void sim_mac_power(sim_mac_t *dev, bool on);
void sim_mac_attach_ref(sim_mac_t *dev, int input, sim_pin_t *ref_pps);

/* The next (or current) lock attempt fails: Alarms bit 3, never locks. */
void sim_mac_fail_acquisition(sim_mac_t *dev);
/* Lose atomic lock now and re-acquire after relock_s. */
void sim_mac_lose_lock(sim_mac_t *dev, double relock_s);

/* Baseplate temperature now (ambient + start-up rise). */
double sim_mac_baseplate_c(const sim_mac_t *dev);

#endif /* SIM_DEVICES_SIM_MAC_H_ */
