#ifndef SIM_SIM_BOARD_H_
#define SIM_SIM_BOARD_H_

/*
 * The payload board, wired as in the design review (DR2), with one shared
 * environment: the board temperature and the acceleration signals drive the
 * TMP102, the IMU, the GPS dynamics and every oscillator's temperature and
 * g sensitivity at once.
 *
 * FLIGHT board
 *   GPS (NEO-M9N)          UART, TIMEPULSE -> MCU timer capture CH0 and the
 *                          1PPS inputs of both CSACs and both MACs
 *   CSAC x2, MAC x2        UARTs, BITE (+ MAC ALARM) pins, 1PPS outputs
 *   TCXO x2, MEMS x2       32.768 kHz into MCP79511 x4 -> 1 Hz PPS on MFP
 *   TDC7200 x8             START = CSAC 1 PPS; STOP (through the delay line)
 *                          = GPS, CSAC 2, MAC 1, MAC 2, TCXO 1, TCXO 2,
 *                          MEMS 1, MEMS 2; INTB / TRIGG pins; ENABLE via io
 *   LSM6DSR, TMP102, SD    SPI, I2C, FatFs
 *   MCU timebase           32-bit timer on the board crystal (84 MHz)
 *
 * GROUND-TEST board
 *   GPS, CSAC 1, one TDC7200 (START = CSAC 1, STOP = GPS, no delay line),
 *   LSM6DSR, TMP102, SD, MCU timer. The other parts exist but stay unpowered
 *   and unwired.
 *
 * Power at t = 0: GPS, CSACs, sensors and RTCs on (RTC oscillators off until
 * the firmware sets EXTOSC); MACs off (they are off on the pad); TDC ENABLE
 * low (the firmware raises it).
 *
 * Assumptions to confirm with the hardware team are marked ASSUMED.
 */

#include <stdbool.h>
#include <stdint.h>

#include "devices/sim_csac.h"
#include "devices/sim_gps.h"
#include "devices/sim_lsm6dsr.h"
#include "devices/sim_mac.h"
#include "devices/sim_mcp79511.h"
#include "devices/sim_tdc7200.h"
#include "devices/sim_tmp102.h"
#include "sim_osc.h"
#include "sim_sd.h"
#include "sim_signal.h"
#include "sim_timer.h"

typedef enum {
    SIM_BOARD_FLIGHT = 0,
    SIM_BOARD_GROUND_TEST,
} sim_board_kind_t;

/* Where TDC STOP n comes from (flight board order). */
typedef enum {
    SIM_TDC_GPS = 0,
    SIM_TDC_CSAC2,
    SIM_TDC_MAC1,
    SIM_TDC_MAC2,
    SIM_TDC_TCXO1,
    SIM_TDC_TCXO2,
    SIM_TDC_MEMS1,
    SIM_TDC_MEMS2,
    SIM_TDC_COUNT,
} sim_tdc_channel_t;

typedef enum {
    SIM_TDC_CLOCK_CSAC_10MHZ = 0, /* CSAC 1 RF output (ASSUMED default) */
    SIM_TDC_CLOCK_MCU_8MHZ,       /* MCU clock output from the crystal */
} sim_tdc_clock_t;

typedef struct {
    sim_board_kind_t kind;
    uint64_t seed;                /* every part's unit-to-unit variation */
    const char *sd_root;          /* host folder for the SD card */
    sim_tdc_clock_t tdc_clock;
    double tdc_stop_delay_s;      /* flight delay line (ASSUMED 1 us); ground test: 0 */
    double mcu_timer_hz;          /* 84 MHz: TIM2/TIM5 on APB1 x2 (ASSUMED) */
    double mcu_crystal_ppm;       /* +-ppm for the board crystal (UNVERIFIED: part unknown) */
    int imu_thrust_axis;          /* 0 X, 1 Y, 2 Z (ASSUMED Z) */
} sim_board_config_t;

typedef struct {
    sim_board_config_t cfg;

    /* Environment: change these and every part sees it. */
    sim_signal_t temperature_c;   /* board / payload bay */
    sim_signal_t accel_g[3];      /* specific force on the body axes, g */

    /* Parts. */
    sim_gps_t gps;
    sim_csac_t csac[2];
    sim_mac_t mac[2];
    sim_osc_t tcxo[2];
    sim_osc_t mems[2];
    sim_mcp79511_t rtc[4];        /* 0-1 TCXO, 2-3 MEMS */
    sim_tdc7200_t tdc[SIM_TDC_COUNT];
    int n_tdc;
    sim_osc_t mcu_crystal;
    sim_timer_t timer;            /* channel 0 = GPS PPS capture */
    sim_lsm6dsr_t imu;
    sim_tmp102_t tmp;
} sim_board_t;

sim_board_config_t sim_board_default_config(sim_board_kind_t kind);

/* Builds and wires everything. Large: declare the board static. */
void sim_board_init(sim_board_t *b, const sim_board_config_t *cfg);

/* The pin that feeds TDC STOP for a channel (flight board). */
sim_pin_t *sim_board_pps(sim_board_t *b, sim_tdc_channel_t ch);

#endif /* SIM_SIM_BOARD_H_ */
