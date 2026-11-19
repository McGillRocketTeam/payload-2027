#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "sensors/drivers/lsm6dsr/lsm6dsr_reg.h"
#include "sim.h"
#include "sim_board.h"
#include "sim_log.h"
#include "sim_scenario.h"

/*
 * End-to-end: the whole board, driven the way firmware would drive it, and
 * the mission timeline.
 */

static sim_board_t board;
static sim_scenario_t sc;

void setUp(void)
{
    sim_reset();
    char path[128];
    snprintf(path, sizeof(path), ".pio/sim/board_%s.csv", Unity.CurrentTestName);
    sim_log_open(path);
}

void tearDown(void)
{
    sim_log_close();
}

/* ---- firmware-like helpers ---- */

static void uart_send(sim_uart_t *u, const char *s)
{
    sim_uart_write(u, (const uint8_t *)s, strlen(s));
}

static void gps_use_gps_grid(void)
{
    /* UBX-CFG-VALSET, RAM: CFG-TP-TIMEGRID_TP1 = 1 (GPS) */
    uint8_t p[9] = {0, 0x01, 0, 0, 0x0C, 0x00, 0x05, 0x20, 1};
    uint8_t f[20];
    size_t n = sim_ubx_frame(0x06, 0x8A, p, 9, f);
    sim_uart_write(&board.gps.uart, f, n);
}

static void tdc_wr(sim_tdc7200_t *t, uint8_t a, uint8_t v)
{
    uint8_t tx[2] = {(uint8_t)(0x40 | a), v};
    uint8_t rx[2];
    sim_tdc7200_spi_transfer(t, tx, rx, 2);
}

static uint32_t tdc_rd24(sim_tdc7200_t *t, uint8_t a)
{
    uint8_t tx[4] = {a, 0, 0, 0};
    uint8_t rx[4];
    sim_tdc7200_spi_transfer(t, tx, rx, 4);
    return ((uint32_t)rx[1] << 16 | (uint32_t)rx[2] << 8 | rx[3]) & 0x7FFFFF;
}

/* Mode 2 TOF (data sheet eq. 2) with the nominal clock. */
static double tdc_tof(sim_tdc7200_t *t, double clock_hz)
{
    double cal1 = tdc_rd24(t, 0x1B);
    double cal2 = tdc_rd24(t, 0x1C);
    double norm = (1.0 / clock_hz) / ((cal2 - cal1) / 9.0);
    return norm * ((double)tdc_rd24(t, 0x10) - (double)tdc_rd24(t, 0x12)) +
           tdc_rd24(t, 0x11) / clock_hz;
}

/* Ground test: sync CSAC 1 to GPS, then measure GPS vs CSAC with the TDC. */
static double ground_test_offset(double stop_delay_s, bool *ok)
{
    sim_board_config_t cfg = sim_board_default_config(SIM_BOARD_GROUND_TEST);
    cfg.tdc_stop_delay_s = stop_delay_s;
    sim_board_init(&board, &cfg);
    sim_advance_ms(100);
    gps_use_gps_grid();
    sim_advance_ns(SIM_S(200));          /* GPS fix (24 s) and CSAC lock (180 s) */
    uart_send(&board.csac[0].uart, "!MM\r\n"); /* INIT: CSAC phase measurement mode (DR2) */
    sim_advance_ms(50);
    uart_send(&board.csac[0].uart, "S");       /* sync CSAC 1PPS to GPS */
    sim_advance_ns(SIM_S(3));

    sim_tdc7200_t *t = &board.tdc[0];
    sim_tdc7200_set_enable(t, true);
    sim_advance_ms(2);
    tdc_wr(t, 0x00, 0x03); /* mode 2, start */
    sim_advance_ns(SIM_S(2));
    *ok = (tdc_rd24(t, 0x1C) != 0);
    return *ok ? tdc_tof(t, 10e6) - stop_delay_s : NAN;
}

void test_ground_test_board_with_delay_line_matches_csac_phase_meter(void)
{
    bool ok;
    double offset = ground_test_offset(1e-6, &ok);
    TEST_ASSERT_TRUE(ok);
    /* After !S the CSAC is within one RF cycle of GPS. */
    TEST_ASSERT_TRUE(fabs(offset) <= 100e-9);
    /* The CSAC's own phase meter (GPS - CSAC) sees the same thing. */
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, board.csac[0].clk.phase_s, offset);
}

void test_ground_test_board_without_delay_line_cannot_measure_after_sync(void)
{
    bool ok;
    (void)ground_test_offset(0.0, &ok);
    /* GPS within +-50 ns of the CSAC: either before START (missed, overflow)
     * or closer than 2 tCLOCK (timing violation). */
    TEST_ASSERT_TRUE(!ok || sim_log_count("tdc_gps", "error") > 0);
    TEST_ASSERT_TRUE(sim_log_count("tdc_gps", "error") > 0);
}

/* ---- mission ---- */

void test_mission_kinematics(void)
{
    sim_board_config_t cfg = sim_board_default_config(SIM_BOARD_FLIGHT);
    sim_board_init(&board, &cfg);
    sim_scenario_init(&sc, &board, NULL);
    double v = 5.0 * 9.80665 * 3.4;
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, v, sc.v_burnout_mps);
    TEST_ASSERT_DOUBLE_WITHIN(1e-3, sc.pad_alt_m + sc.h_apogee_m,
                              sim_scenario_altitude_m(&sc, sc.t_apogee_s));
    TEST_ASSERT_DOUBLE_WITHIN(1e-6, sc.pad_alt_m, sim_scenario_altitude_m(&sc, sc.t_landing_s + 1));
}

void test_flight_board_mission_to_after_boost(void)
{
    sim_board_config_t cfg = sim_board_default_config(SIM_BOARD_FLIGHT);
    sim_board_init(&board, &cfg);
    sim_scenario_init(&sc, &board, NULL);
    stmdev_ctx_t imu = sim_lsm6dsr_ctx(&board.imu);
    lsm6dsr_xl_full_scale_set(&imu, LSM6DSR_16g);
    lsm6dsr_xl_data_rate_set(&imu, LSM6DSR_XL_ODR_6667Hz);

    /* Pad: MACs off, then on at 15 min and announcing on their UARTs. */
    sim_scenario_run(&sc, sc.m.macs_on_s - 1.0, NULL, NULL, 0);
    TEST_ASSERT_FALSE(board.mac[0].base.powered);
    sim_scenario_run(&sc, sc.m.macs_on_s + 0.5, NULL, NULL, 0);
    uint8_t buf[64] = {0};
    sim_uart_read(&board.mac[0].uart, buf, sizeof(buf) - 1);
    TEST_ASSERT_NOT_NULL(strstr((char *)buf, "[>Loading...]"));
    TEST_ASSERT_TRUE(board.gps.fix);
    TEST_ASSERT_EQUAL_INT(0, board.csac[0].status);

    /* Boost: the IMU reads the thrust... */
    sim_scenario_run(&sc, sc.t_launch_s + 1.0, NULL, NULL, 0);
    int16_t raw[3];
    lsm6dsr_acceleration_raw_get(&imu, raw);
    TEST_ASSERT_FLOAT_WITHIN(50.0f, 6000.0f, lsm6dsr_from_fs16g_to_mg(raw[2]));
    /* ...and the GPS (portable model, 6 g) has lost its fix. */
    sim_scenario_run(&sc, sc.t_launch_s + 2.5, NULL, NULL, 0);
    TEST_ASSERT_FALSE(board.gps.fix);
    TEST_ASSERT_EQUAL_INT(0, board.csac[0].status); /* still locked */

    TEST_ASSERT_EQUAL_UINT32(1, sim_log_count("gps", "state") >= 1 ? 1 : 0);
}

void test_flight_board_has_eight_tdcs_on_the_right_signals(void)
{
    sim_board_config_t cfg = sim_board_default_config(SIM_BOARD_FLIGHT);
    sim_board_init(&board, &cfg);
    TEST_ASSERT_EQUAL_INT(8, board.n_tdc);
    TEST_ASSERT_EQUAL_PTR(&board.gps.pps, sim_board_pps(&board, SIM_TDC_GPS));
    TEST_ASSERT_EQUAL_PTR(&board.rtc[3].mfp, sim_board_pps(&board, SIM_TDC_MEMS2));
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 1e-6, board.tdc[5].cfg.stop_delay_s);

    sim_board_config_t g = sim_board_default_config(SIM_BOARD_GROUND_TEST);
    sim_board_init(&board, &g);
    TEST_ASSERT_EQUAL_INT(1, board.n_tdc);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 0.0, board.tdc[0].cfg.stop_delay_s);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_ground_test_board_with_delay_line_matches_csac_phase_meter);
    RUN_TEST(test_ground_test_board_without_delay_line_cannot_measure_after_sync);
    RUN_TEST(test_mission_kinematics);
    RUN_TEST(test_flight_board_mission_to_after_boost);
    RUN_TEST(test_flight_board_has_eight_tdcs_on_the_right_signals);
    return UNITY_END();
}
