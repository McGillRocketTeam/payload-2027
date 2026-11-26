#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "sim.h"
#include "sim_log.h"
#include "sim_osc.h"
#include "sim_pin.h"
#include "sim_pps.h"
#include "sim_timer.h"

/* Events, oscillators, PPS outputs, the capture timer and the run log. */

void setUp(void)
{
    sim_reset();
}

void tearDown(void)
{
    sim_log_close();
}

/* ---- events ---- */

static sim_time_ps_t fired_at[4];
static sim_time_ps_t fired_exact[4];
static int n_fired;

static void record(void *ctx, uint32_t tag)
{
    (void)ctx;
    fired_at[n_fired] = sim_now_ps();
    fired_exact[n_fired] = sim_event_time_ps();
    n_fired++;
    if (tag == 99) {
        sim_advance_ms(5); /* a slow handler, e.g. a blocking SPI read */
    }
}

void test_events_fire_in_time_order_at_exact_time(void)
{
    n_fired = 0;
    sim_schedule_ps(3000, record, NULL, 0);
    sim_schedule_ps(1000, record, NULL, 0);
    sim_advance_ns(10);
    TEST_ASSERT_EQUAL_INT(2, n_fired);
    TEST_ASSERT_EQUAL_UINT64(1000, fired_at[0]);
    TEST_ASSERT_EQUAL_UINT64(3000, fired_at[1]);
    TEST_ASSERT_EQUAL_UINT64(10000, sim_now_ps());
}

void test_busy_handler_delays_later_events_but_keeps_exact_time(void)
{
    n_fired = 0;
    sim_schedule_ps(SIM_MS(1) * SIM_PS_PER_NS, record, NULL, 99);
    sim_schedule_ps(SIM_MS(2) * SIM_PS_PER_NS, record, NULL, 0);
    sim_advance_ms(10);
    TEST_ASSERT_EQUAL_INT(2, n_fired);
    /* Second event was due at 2 ms but the first handler was busy until 6 ms. */
    TEST_ASSERT_EQUAL_UINT64(SIM_MS(6) * SIM_PS_PER_NS, fired_at[1]);
    TEST_ASSERT_EQUAL_UINT64(SIM_MS(2) * SIM_PS_PER_NS, fired_exact[1]);
}

/* ---- oscillators and PPS ---- */

static sim_osc_t osc;
static sim_pps_t pps;

void test_ideal_pps_lands_on_true_seconds(void)
{
    sim_pps_init(&pps, "ideal", NULL, 100 * SIM_PS_PER_NS);
    sim_pps_start(&pps);
    sim_advance_ns(SIM_S(3) + SIM_MS(500));
    TEST_ASSERT_EQUAL_UINT32(3, pps.pin.rises);
    TEST_ASSERT_EQUAL_UINT64(3 * SIM_PS_PER_S, pps.pin.last_rise_ps);
}

void test_frequency_offset_makes_pps_run_early(void)
{
    sim_osc_config_t c = sim_osc_ideal();
    c.y0 = 1e-6; /* 1 ppm fast */
    sim_osc_init(&osc, "fast", &c, 0.0);
    sim_pps_init(&pps, "fast", &osc, 100 * SIM_PS_PER_NS);
    sim_pps_start(&pps);
    sim_advance_ns(SIM_S(10) + SIM_MS(500));
    /* Edge 10 comes 10 us early. */
    double t = (double)pps.pin.last_rise_ps / 1e12;
    TEST_ASSERT_DOUBLE_WITHIN(1e-11, 10.0 - 10.0 / (1.0 + 1e-6), 10.0 - t);
}

void test_oscillator_noise_is_independent_of_query_pattern(void)
{
    sim_osc_config_t c = sim_osc_ideal();
    c.adev_1s = 1e-9;
    c.rw_per_sqrt_s = 1e-11;
    c.seed = 42;
    sim_osc_t a;
    sim_osc_t b;
    sim_osc_init(&a, "a", &c, 0.0);
    sim_osc_init(&b, "b", &c, 0.0);
    for (int i = 1; i <= 1000; i++) {
        (void)sim_osc_error_at(&a, (sim_time_ps_t)i * 7919ULL * 1000000ULL); /* odd steps */
    }
    sim_time_ps_t t = 7919ULL * 1000ULL * 1000000ULL;
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, sim_osc_error_at(&a, t), sim_osc_error_at(&b, t));
    TEST_ASSERT_TRUE(fabs(sim_osc_error_at(&b, t)) > 1e-12); /* noise is really there */
}

void test_acceleration_sensitivity_shifts_frequency(void)
{
    sim_signal_t accel = sim_signal_step(1.0, 6.0, 1.0); /* boost at t = 1 s */
    sim_osc_config_t c = sim_osc_ideal();
    c.g_sens[2] = 1e-9;
    c.accel_g[2] = &accel;
    sim_osc_init(&osc, "g", &c, 0.0);
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 1e-9, sim_osc_freq_at(&osc, SIM_PS_PER_S / 2));
    TEST_ASSERT_DOUBLE_WITHIN(1e-15, 6e-9, sim_osc_freq_at(&osc, 2 * SIM_PS_PER_S));
}

void test_align_snaps_to_nearest_rf_cycle(void)
{
    sim_osc_config_t c = sim_osc_ideal();
    c.y0 = 3e-7;
    sim_osc_init(&osc, "csac", &c, 0.0);
    sim_pps_init(&pps, "csac", &osc, 100 * SIM_PS_PER_NS);
    pps.epoch_s = 0.123456789;
    sim_pps_start(&pps);
    sim_advance_ns(SIM_MS(2500));
    sim_pps_align(&pps, 3.0, 100e-9); /* reference edge at t = 3 s, 10 MHz RF */
    sim_advance_ns(SIM_MS(1000));
    double t = (double)pps.pin.last_rise_ps / 1e12;
    TEST_ASSERT_DOUBLE_WITHIN(50e-9 + 1e-12, 3.0, t);
}

void test_ecs327txo_preset_stays_inside_its_data_sheet_limits(void)
{
    sim_signal_t temp = sim_signal_const(-40.0);
    for (uint64_t seed = 1; seed <= 50; seed++) {
        sim_osc_config_t c = sim_osc_ecs327txo(seed);
        TEST_ASSERT_TRUE(fabs(c.y0) <= 1.5e-6);
        c.temperature_c = &temp;
        c.y0 = 0.0;
        c.aging_per_day = 0.0;
        sim_osc_init(&osc, "tcxo", &c, 0.0);
        TEST_ASSERT_TRUE(fabs(sim_osc_freq_at(&osc, 0)) <= 5e-6);
    }
}

void test_dsc60xxb_preset_stays_inside_its_stability_grade(void)
{
    sim_signal_t temp = sim_signal_const(85.0);
    for (uint64_t seed = 1; seed <= 50; seed++) {
        sim_osc_config_t c = sim_osc_dsc60xxb(seed, 25.0, -40.0, 85.0);
        c.temperature_c = &temp;
        c.aging_per_day = 0.0;
        sim_osc_init(&osc, "mems", &c, 0.0);
        TEST_ASSERT_TRUE(fabs(sim_osc_freq_at(&osc, 0)) <= 25e-6);
    }
}

/* ---- capture timer ---- */

static uint32_t captures[8];
static int n_captures;

static void pps_isr(void *ctx, int ch, uint32_t capture)
{
    (void)ctx;
    (void)ch;
    captures[n_captures++] = capture;
}

void test_timer_capture_measures_its_own_clock_error(void)
{
    sim_osc_config_t c = sim_osc_ideal();
    c.y0 = 20e-6; /* MCU crystal 20 ppm fast */
    sim_osc_init(&osc, "hse", &c, 0.0);
    sim_timer_t tim;
    sim_timer_init(&tim, "tim2", &osc, 84e6);
    sim_pps_init(&pps, "gps", NULL, 100 * SIM_PS_PER_NS);
    n_captures = 0;
    sim_timer_capture(&tim, 0, &pps.pin, true, pps_isr, NULL);
    sim_pps_start(&pps);
    sim_advance_ns(SIM_S(3) + SIM_MS(100));

    TEST_ASSERT_EQUAL_INT(3, n_captures);
    uint32_t ticks = captures[2] - captures[1];
    TEST_ASSERT_UINT32_WITHIN(1, (uint32_t)llround(84e6 * (1.0 + 20e-6)), ticks);
}

/* ---- run log ---- */

void test_run_log_records_truth_rows(void)
{
    const char *path = ".pio/sim/test_sim_timing.csv";
    TEST_ASSERT_TRUE(sim_log_open(path));
    sim_pps_init(&pps, "ideal", NULL, 100 * SIM_PS_PER_NS);
    sim_pps_start(&pps);
    sim_advance_ns(SIM_S(5) + SIM_MS(1));
    TEST_ASSERT_EQUAL_UINT32(5, sim_log_count("ideal", "truth"));
    sim_log_close();

    FILE *f = fopen(path, "r");
    TEST_ASSERT_NOT_NULL(f);
    char line[256];
    TEST_ASSERT_NOT_NULL(fgets(line, sizeof(line), f));
    TEST_ASSERT_EQUAL_STRING("t_s,source,kind,name,value,detail\n", line);
    TEST_ASSERT_NOT_NULL(fgets(line, sizeof(line), f));
    TEST_ASSERT_EQUAL_STRING_LEN("1.000000000000,ideal,truth,pps_offset_s,0,", line, 41);
    fclose(f);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_events_fire_in_time_order_at_exact_time);
    RUN_TEST(test_busy_handler_delays_later_events_but_keeps_exact_time);
    RUN_TEST(test_ideal_pps_lands_on_true_seconds);
    RUN_TEST(test_frequency_offset_makes_pps_run_early);
    RUN_TEST(test_oscillator_noise_is_independent_of_query_pattern);
    RUN_TEST(test_acceleration_sensitivity_shifts_frequency);
    RUN_TEST(test_align_snaps_to_nearest_rf_cycle);
    RUN_TEST(test_ecs327txo_preset_stays_inside_its_data_sheet_limits);
    RUN_TEST(test_dsc60xxb_preset_stays_inside_its_stability_grade);
    RUN_TEST(test_timer_capture_measures_its_own_clock_error);
    RUN_TEST(test_run_log_records_truth_rows);
    return UNITY_END();
}
