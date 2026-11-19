#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "devices/sim_tdc7200.h"
#include "sim.h"
#include "sim_log.h"
#include "sim_osc.h"
#include "sim_pin.h"

/*
 * TDC7200 model against the data sheet (SNAS647D), through raw SPI frames
 * exactly as the data sheet specifies them.
 */

static sim_tdc7200_t tdc;
static sim_pin_t start;
static sim_pin_t stop;

void setUp(void)
{
    sim_reset();
    char path[128];
    snprintf(path, sizeof(path), ".pio/sim/tdc_%s.csv", Unity.CurrentTestName);
    sim_log_open(path);
    sim_tdc7200_config_t cfg = sim_tdc7200_default_config(NULL, 8e6);
    sim_tdc7200_init(&tdc, "tdc1", &cfg);
    sim_pin_init(&start, "start", false);
    sim_pin_init(&stop, "stop", false);
    sim_tdc7200_attach_start(&tdc, &start);
    sim_tdc7200_attach_stop(&tdc, &stop);
}

void tearDown(void)
{
    sim_log_close();
}

/* ---- SPI helpers: command = auto-inc | RW | address (data sheet 8.5.1.5) ---- */

static uint8_t rd8(uint8_t addr)
{
    uint8_t tx[2] = {addr, 0};
    uint8_t rx[2];
    sim_tdc7200_spi_transfer(&tdc, tx, rx, 2);
    return rx[1];
}

static void wr8(uint8_t addr, uint8_t v)
{
    uint8_t tx[2] = {(uint8_t)(0x40 | addr), v};
    uint8_t rx[2];
    sim_tdc7200_spi_transfer(&tdc, tx, rx, 2);
}

static uint32_t rd24(uint8_t addr)
{
    uint8_t tx[4] = {addr, 0, 0, 0};
    uint8_t rx[4];
    sim_tdc7200_spi_transfer(&tdc, tx, rx, 4);
    return ((uint32_t)rx[1] << 16 | (uint32_t)rx[2] << 8 | rx[3]) & 0x7FFFFF;
}

static void enable(void)
{
    sim_tdc7200_set_enable(&tdc, true);
    sim_advance_ms(2); /* past T3 LDO settling (1.5 ms) */
}

/* Equation 1 / 2 of the data sheet, from the result registers. */
static double tof(int mode, int n, double clock_hz)
{
    int cal_periods[4] = {2, 10, 20, 40};
    double cal2p = cal_periods[(rd8(0x01) >> 6) & 3];
    double cal1 = rd24(0x1B);
    double cal2 = rd24(0x1C);
    double calcount = (cal2 - cal1) / (cal2p - 1.0);
    double norm_lsb = (1.0 / clock_hz) / calcount;
    if (mode == 1) {
        return rd24((uint8_t)(0x10 + 2 * (n - 1))) * norm_lsb;
    }
    double t1 = rd24(0x10);
    double tn1 = rd24((uint8_t)(0x10 + 2 * n));
    double cc = rd24((uint8_t)(0x11 + 2 * (n - 1)));
    return norm_lsb * (t1 - tn1) + cc / clock_hz;
}

/* ---- the data sheet's own worked examples, to check the equation above ---- */

void test_datasheet_examples_for_the_tof_equations(void)
{
    double calcount = (21121.0 - 2110.0) / (10 - 1);
    double norm = (1.0 / 8e6) / calcount;
    TEST_ASSERT_DOUBLE_WITHIN(0.01e-9, 247.061e-9, 4175 * norm); /* 8.4.2.1.1 */

    calcount = (23133.0 - 2315.0) / (10 - 1);
    norm = (1.0 / 8e6) / calcount;
    TEST_ASSERT_DOUBLE_WITHIN(0.001e-6, 39.855e-6, norm * (2147 - 201) + 318 / 8e6); /* 8.4.2.2.1 */
}

/* ---- registers and SPI ---- */

void test_reset_values_and_spi_framing(void)
{
    enable();
    TEST_ASSERT_EQUAL_HEX8(0x00, rd8(0x00));
    TEST_ASSERT_EQUAL_HEX8(0x40, rd8(0x01));
    TEST_ASSERT_EQUAL_HEX8(0x07, rd8(0x03));
    TEST_ASSERT_EQUAL_HEX8(0xFF, rd8(0x04));
    TEST_ASSERT_EQUAL_HEX8(0x00, rd8(0x08));

    /* Auto-increment read of CONFIG1..INT_MASK in one transaction. */
    uint8_t tx[5] = {0x80, 0, 0, 0, 0};
    uint8_t rx[5];
    sim_tdc7200_spi_transfer(&tdc, tx, rx, 5);
    TEST_ASSERT_EQUAL_HEX8(0x40, rx[2]);
    TEST_ASSERT_EQUAL_HEX8(0x07, rx[4]);

    /* Without auto-increment, bytes past the register are not driven. */
    uint8_t tx2[3] = {0x01, 0, 0};
    sim_tdc7200_spi_transfer(&tdc, tx2, rx, 3);
    TEST_ASSERT_EQUAL_HEX8(0x40, rx[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, rx[2]);

    wr8(0x01, 0x42);
    TEST_ASSERT_EQUAL_HEX8(0x42, rd8(0x01));
}

void test_disabled_or_starting_chip_does_not_answer(void)
{
    TEST_ASSERT_EQUAL_HEX8(0xFF, rd8(0x01)); /* ENABLE low */
    sim_tdc7200_set_enable(&tdc, true);
    TEST_ASSERT_EQUAL_HEX8(0xFF, rd8(0x01)); /* < 100 us after ENABLE */
    sim_advance_ns(SIM_US(150));
    TEST_ASSERT_EQUAL_HEX8(0x40, rd8(0x01));
    wr8(0x01, 0x42);
    sim_tdc7200_set_enable(&tdc, false);
    enable();
    TEST_ASSERT_EQUAL_HEX8(0x40, rd8(0x01)); /* settings lost */
}

/* ---- measurements ---- */

static void edge_pair(double t_start, double t_stop)
{
    sim_pin_pulse_at(&start, (sim_time_ps_t)llround(t_start * 1e12), 1000000ULL);
    sim_pin_pulse_at(&stop, (sim_time_ps_t)llround(t_stop * 1e12), 1000000ULL);
}

void test_mode2_measures_time_of_flight(void)
{
    enable();
    wr8(0x00, 0x03); /* mode 2, start measurement */
    TEST_ASSERT_TRUE(tdc.trigg.level);
    double t0 = sim_now_s() + 1.234567e-6;
    edge_pair(t0, t0 + 39.855e-6);
    sim_advance_ms(1);

    TEST_ASSERT_FALSE(tdc.intb.level);          /* interrupt: active low */
    TEST_ASSERT_EQUAL_HEX8(0x19, rd8(0x02));    /* complete | started | new meas */
    TEST_ASSERT_EQUAL_HEX8(0x00, rd8(0x00) & 1); /* START_MEAS cleared */
    TEST_ASSERT_DOUBLE_WITHIN(150e-12, 39.855e-6, tof(2, 1, 8e6));

    wr8(0x02, 0x1F); /* write 1 to clear */
    TEST_ASSERT_TRUE(tdc.intb.level);
}

void test_mode1_short_interval(void)
{
    enable();
    wr8(0x00, 0x01); /* mode 1 */
    double t0 = sim_now_s() + 1e-6;
    edge_pair(t0, t0 + 247.061e-9);
    sim_advance_ms(1);
    TEST_ASSERT_DOUBLE_WITHIN(150e-12, 247.061e-9, tof(1, 1, 8e6));
}

void test_mode2_noise_matches_datasheet_std(void)
{
    enable();
    double sum = 0.0;
    double sum2 = 0.0;
    const int n = 300;
    for (int i = 0; i < n; i++) {
        wr8(0x00, 0x03);
        double t0 = sim_now_s() + 1e-6 + i * 13.7e-9; /* vary the phase vs CLOCK */
        edge_pair(t0, t0 + 1e-6);
        sim_advance_ms(1);
        double e = tof(2, 1, 8e6) - 1e-6;
        sum += e;
        sum2 += e * e;
        wr8(0x02, 0x1F);
    }
    double mean = sum / n;
    double std = sqrt(sum2 / n - mean * mean);
    TEST_ASSERT_DOUBLE_WITHIN(10e-12, 35e-12, std); /* data sheet 7.5: 35 ps at 1 us */
}

void test_three_stops(void)
{
    enable();
    wr8(0x01, 0x42); /* 10 calibration periods, 3 stops */
    wr8(0x00, 0x03);
    double t0 = sim_now_s() + 1e-6;
    sim_pin_pulse_at(&start, (sim_time_ps_t)llround(t0 * 1e12), 1000000ULL);
    for (int k = 1; k <= 3; k++) {
        sim_pin_pulse_at(&stop, (sim_time_ps_t)llround((t0 + k * 5e-6) * 1e12), 1000000ULL);
    }
    sim_advance_ms(1);
    for (int k = 1; k <= 3; k++) {
        TEST_ASSERT_DOUBLE_WITHIN(150e-12, k * 5e-6, tof(2, k, 8e6));
    }
}

void test_reference_clock_error_enters_the_measurement(void)
{
    sim_osc_config_t c = sim_osc_ideal();
    c.y0 = 100e-6; /* reference 100 ppm fast */
    sim_osc_t ref;
    sim_osc_init(&ref, "ref", &c, 0.0);
    tdc.cfg.clock = &ref;
    enable();
    wr8(0x00, 0x03);
    double t0 = sim_now_s() + 1e-6;
    edge_pair(t0, t0 + 1e-3);
    sim_advance_ms(3);
    /* Computed with the nominal 8 MHz, the 1 ms reads ~100 ns long. */
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1e-3 * (1.0 + 100e-6), tof(2, 1, 8e6));
}

void test_missing_stop_overflows_clock_counter(void)
{
    enable();
    wr8(0x06, 0x01); /* CLOCK_CNTR_OVF = 0x0100 */
    wr8(0x07, 0x00);
    wr8(0x00, 0x03);
    sim_pin_pulse_at(&start, sim_now_ps() + 1000000ULL, 1000000ULL);
    sim_advance_ms(1);
    TEST_ASSERT_EQUAL_HEX8(0x04, rd8(0x02) & 0x07); /* CLOCK_CNTR_OVF_INT only */
    TEST_ASSERT_FALSE(tdc.intb.level);
    TEST_ASSERT_EQUAL_UINT32(0, rd24(0x1B)); /* no FORCE_CAL: no calibration */
}

/* The payload case: STOP (oscillator under test) slightly ahead of START
 * (CSAC). The early STOP is not seen, and the next one is a second away. */
void test_stop_before_start_runs_out_of_range(void)
{
    enable();
    wr8(0x00, 0x03);
    double t0 = sim_now_s() + 1e-3;
    sim_pin_pulse_at(&stop, (sim_time_ps_t)llround(t0 * 1e12), 1000000ULL);
    sim_pin_pulse_at(&start, (sim_time_ps_t)llround((t0 + 200e-9) * 1e12), 1000000ULL);
    sim_advance_ms(20); /* (2^16 - 2) x 125 ns = 8.2 ms range */
    TEST_ASSERT_EQUAL_HEX8(0x04, rd8(0x02) & 0x04);
    TEST_ASSERT_EQUAL_UINT32(1, sim_log_count("tdc1", "error"));
}

/* Flight board: a known delay on STOP makes the same case measurable. */
void test_stop_delay_line_makes_early_stop_measurable(void)
{
    tdc.cfg.stop_delay_s = 2e-6;
    enable();
    wr8(0x00, 0x03);
    double t0 = sim_now_s() + 1e-3;
    sim_pin_pulse_at(&stop, (sim_time_ps_t)llround(t0 * 1e12), 1000000ULL); /* 200 ns ahead */
    sim_pin_pulse_at(&start, (sim_time_ps_t)llround((t0 + 200e-9) * 1e12), 1000000ULL);
    sim_advance_ms(3);
    double offset = tof(2, 1, 8e6) - 2e-6; /* firmware subtracts the known delay */
    TEST_ASSERT_DOUBLE_WITHIN(150e-12, -200e-9, offset);
}

void test_polling_int_status_is_flagged(void)
{
    enable();
    wr8(0x00, 0x03);
    (void)rd8(0x02);
    TEST_ASSERT_EQUAL_UINT32(1, sim_log_count("tdc1", "error"));
}

void test_stop_mask_ignores_early_stops(void)
{
    enable();
    wr8(0x01, 0x41); /* two stops */
    wr8(0x08, 0x01); /* mask = 0x0100 = 256 clocks = 32 us */
    wr8(0x09, 0x00);
    wr8(0x00, 0x03);
    double t0 = sim_now_s() + 1e-6;
    sim_pin_pulse_at(&start, (sim_time_ps_t)llround(t0 * 1e12), 1000000ULL);
    sim_pin_pulse_at(&stop, (sim_time_ps_t)llround((t0 + 19e-6) * 1e12), 1000000ULL);  /* masked */
    sim_pin_pulse_at(&stop, (sim_time_ps_t)llround((t0 + 119e-6) * 1e12), 1000000ULL);
    sim_pin_pulse_at(&stop, (sim_time_ps_t)llround((t0 + 219e-6) * 1e12), 1000000ULL);
    sim_advance_ms(1);
    TEST_ASSERT_DOUBLE_WITHIN(150e-12, 119e-6, tof(2, 1, 8e6));
    TEST_ASSERT_DOUBLE_WITHIN(150e-12, 219e-6, tof(2, 2, 8e6));
}

void test_parity_bit_is_even(void)
{
    enable();
    wr8(0x00, 0x43); /* parity on, mode 2, start */
    double t0 = sim_now_s() + 1e-6;
    edge_pair(t0, t0 + 10e-6);
    sim_advance_ms(1);
    uint8_t tx[4] = {0x10, 0, 0, 0};
    uint8_t rx[4];
    sim_tdc7200_spi_transfer(&tdc, tx, rx, 4);
    uint32_t v = (uint32_t)rx[1] << 16 | (uint32_t)rx[2] << 8 | rx[3];
    TEST_ASSERT_EQUAL_INT(0, __builtin_popcount(v) % 2);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_datasheet_examples_for_the_tof_equations);
    RUN_TEST(test_reset_values_and_spi_framing);
    RUN_TEST(test_disabled_or_starting_chip_does_not_answer);
    RUN_TEST(test_mode2_measures_time_of_flight);
    RUN_TEST(test_mode1_short_interval);
    RUN_TEST(test_mode2_noise_matches_datasheet_std);
    RUN_TEST(test_three_stops);
    RUN_TEST(test_reference_clock_error_enters_the_measurement);
    RUN_TEST(test_missing_stop_overflows_clock_counter);
    RUN_TEST(test_stop_before_start_runs_out_of_range);
    RUN_TEST(test_stop_delay_line_makes_early_stop_measurable);
    RUN_TEST(test_polling_int_status_is_flagged);
    RUN_TEST(test_stop_mask_ignores_early_stops);
    RUN_TEST(test_parity_bit_is_even);
    return UNITY_END();
}
