#ifndef SIM_DEVICES_SIM_GPS_H_
#define SIM_DEVICES_SIM_GPS_H_

/*
 * Simulated u-blox NEO-M9N. Firmware side: UART plugs into gps_io_t
 * (uart = &gps.uart), timepulse is gps.pps (a sim_pin_t).
 *
 * Time: GPS time is true sim time plus cfg.gps_start (no error). With
 * cfg.qerr_enabled the pulse lands on the receiver's own clock tick nearest
 * the second instead (sawtooth error, a few ns), reported in TIM-TP qErr,
 * like the real part.
 *
 * Modelled:
 *   - UART1 at 38400 8N1 (default), NMEA out by default (GGA GLL GSA GSV RMC
 *     VTG at 1 Hz, boot screen as TXT) until switched off; baud 4800-921600
 *   - UBX framing + checksum in both directions; bad checksum -> ignored
 *   - all 582 configuration keys of SPG 4.04 with their real defaults
 *     (sim_gps_keys.inc, generated from the interface description)
 *   - CFG-VALSET v0 and v1 (transactions), RAM / BBR / Flash layers, up to 64
 *     pairs, NAK on unknown key, no layer, bad transaction step or invalid
 *     RAM config; CFG-VALGET from RAM / BBR / Flash / Default with group and
 *     all-keys wildcards and position paging
 *   - CFG-RST: hot / warm / cold, hardware and software resets (RAM config
 *     rebuilt from BBR/Flash), GNSS-only reset (config kept), GNSS stop/start;
 *     never acknowledged
 *   - UBX-NAV-PVT, UBX-TIM-TP (periodic or polled), UBX-MON-VER (polled)
 *   - timepulse TP1: 1 Hz, LEN_LOCK when locked (USE_LOCKED), LEN otherwise,
 *     polarity, user delay, antenna cable delay compensation (real delay in
 *     cfg.antenna_cable_delay_ns vs CFG-TP-ANT_CABLEDELAY, default 50 ns);
 *     TIM-TP time base follows CFG-TP-TIMEGRID_TP1 (default UTC)
 *   - fix: cold-start TTFF, hot re-acquisition, sky blockage, dynamic model,
 *     operational limits 4 g / 80 km / 500 m/s
 * Not modelled (accepted and logged as "unmodelled"): timepulse other than
 * 1 Hz period/length, other messages and ports, GSV satellite lists,
 * jamming/spoofing status.
 *
 * Sources: NEO-M9N data sheet (UBX-19014285) and the M9 SPG 4.04 interface
 * description (UBX-21022436): UART defaults, message set, key IDs and
 * defaults, message layouts, VALSET/VALGET/RST rules, MON-VER strings,
 * start times, limits, timepulse accuracy.
 * Integration manual (UBX-19014286): dynamic-model limits (tables 3-4), hot
 * start within 4 h of power-off on backup power, warm/cold download the
 * ephemeris (warm uses the cold-start time), no timepulse on the UTC grid
 * until the UTC parameters arrive (up to 12.5 min after a cold start), UTC
 * fields rounded to 0.01 s, reset types, cable/user delay signs.
 * UNVERIFIED (in none of the three documents): the sign and granularity of
 * qErr, NAV-PVT output latency, the g limit of non-airborne models (the data
 * sheet's 4 g is used), and when confirmedDate/Time get set.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sim_device.h"
#include "sim_pin.h"
#include "sim_signal.h"
#include "sim_uart.h"

/* CFG keys used by the model (u-blox M9 SPG 4.04 interface description,
 * UBX-21022436). The full table of 582 keys lives in sim_gps_keys.inc. */
#define GPS_KEY_UART1_BAUDRATE        0x40520001u
#define GPS_KEY_UART1INPROT_UBX       0x10730001u
#define GPS_KEY_UART1INPROT_NMEA      0x10730002u
#define GPS_KEY_UART1OUTPROT_UBX      0x10740001u
#define GPS_KEY_UART1OUTPROT_NMEA     0x10740002u
#define GPS_KEY_MSGOUT_NAV_PVT_UART1  0x20910007u
#define GPS_KEY_MSGOUT_TIM_TP_UART1   0x2091017Eu
#define GPS_KEY_MSGOUT_GGA_UART1      0x209100BBu
#define GPS_KEY_MSGOUT_GLL_UART1      0x209100CAu
#define GPS_KEY_MSGOUT_GSA_UART1      0x209100C0u
#define GPS_KEY_MSGOUT_GSV_UART1      0x209100C5u
#define GPS_KEY_MSGOUT_RMC_UART1      0x209100ACu
#define GPS_KEY_MSGOUT_VTG_UART1      0x209100B1u
#define GPS_KEY_RATE_MEAS             0x30210001u
#define GPS_KEY_RATE_NAV              0x30210002u
#define GPS_KEY_NAVSPG_DYNMODEL       0x20110021u
#define GPS_KEY_TP_PERIOD_TP1         0x40050002u
#define GPS_KEY_TP_PERIOD_LOCK_TP1    0x40050003u
#define GPS_KEY_TP_LEN_TP1            0x40050004u
#define GPS_KEY_TP_LEN_LOCK_TP1       0x40050005u
#define GPS_KEY_TP_TP1_ENA            0x10050007u
#define GPS_KEY_TP_POL_TP1            0x1005000Bu
#define GPS_KEY_TP_TIMEGRID_TP1       0x2005000Cu
#define GPS_KEY_TP_USE_LOCKED_TP1     0x10050009u
#define GPS_KEY_TP_USER_DELAY_TP1     0x40050006u
#define GPS_KEY_TP_ANT_CABLEDELAY     0x30050001u
#define GPS_KEY_TP_PULSE_DEF          0x20050023u
#define GPS_KEY_TP_PULSE_LENGTH_DEF   0x20050030u

/* Every key the receiver knows (generated table, sim_gps_keys.inc). */
#define SIM_GPS_NKEYS 582

typedef struct {
    double gps_start_s;        /* GPS time (s since 1980-01-06) at sim t = 0 */
    int leap_s;                /* GPS - UTC, 18 */
    double cold_ttff_s;        /* first fix after power-on / cold start */
    double hot_reacq_s;        /* fix back after a short outage */
    double max_accel_g;        /* used when the dynamic model has no g limit */
    double nav_latency_s;      /* epoch -> NAV-PVT on the wire */
    sim_signal_t lat_deg, lon_deg, alt_m;  /* WGS84 position, alt above MSL */
    sim_signal_t vel_n, vel_e, vel_d;      /* m/s */
    sim_signal_t sky;          /* >= 0.5 = antenna sees the sky */
    sim_signal_t num_sv;
    const sim_signal_t *accel_g[3]; /* e.g. &imu.cfg.accel_g[i]; NULL = 1 g at rest */
    bool qerr_enabled;
    double pps_noise_rms_s;    /* extra random pulse error; data sheet: 30 ns RMS, 60 ns 99 % */
    double rx_clock_hz;        /* receiver timebase granularity for qErr */
    double rx_clock_ppb;       /* receiver clock offset, makes the sawtooth */
    bool pps_when_off;
    bool backup_power;         /* V_BCKP: hot start + BBR config kept on power-up */
    double antenna_cable_delay_ns; /* real delay; the receiver compensates 50 ns by default */
} sim_gps_config_t;

typedef struct {
    sim_device_t base;
    sim_gps_config_t cfg;
    sim_uart_t uart;
    sim_pin_t pps;

    uint64_t ram[SIM_GPS_NKEYS];    /* RAM layer: current configuration */
    uint64_t saved[SIM_GPS_NKEYS];  /* BBR layer */
    uint64_t flash[SIM_GPS_NKEYS];  /* Flash layer */

    /* CFG-VALSET staging (version 1 transactions). */
    uint64_t tx_val[SIM_GPS_NKEYS];
    uint8_t tx_set[(SIM_GPS_NKEYS + 7) / 8];
    uint8_t tx_layers;
    bool tx_active;
    bool gnss_stopped;

    bool fix;
    bool has_time;
    bool ephemeris_valid;  /* hot start possible */
    bool utc_known;        /* leap seconds / UTC parameters received */
    double utc_due_s;      /* when they arrive (< 0: not scheduled) */
    double off_since_s;
    double ok_since_s;     /* conditions good since; < 0 = not */
    uint64_t epoch;
    uint32_t gen;

    /* UBX receive parser. */
    int rx_state;
    uint8_t rx_cls, rx_id;
    uint16_t rx_len, rx_pos;
    uint8_t rx_buf[1024];
    uint8_t rx_cka, rx_ckb;

    uint32_t bad_frames;
    uint32_t naks;
} sim_gps_t;

sim_gps_config_t sim_gps_default_config(void);
void sim_gps_init(sim_gps_t *dev, const char *name, const sim_gps_config_t *cfg);
void sim_gps_power(sim_gps_t *dev, bool on);

/* Current configuration value of a key (0 if unknown). */
uint64_t sim_gps_get_key(const sim_gps_t *dev, uint32_t key);

/* Build a UBX frame (for tests and tools). Returns its length. */
size_t sim_ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *out);

#endif /* SIM_DEVICES_SIM_GPS_H_ */
