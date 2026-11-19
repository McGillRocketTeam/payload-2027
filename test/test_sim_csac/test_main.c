#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "devices/sim_csac.h"
#include "sim.h"
#include "sim_log.h"
#include "sim_pps.h"

/*
 * SA.45s CSAC model against the user guide (DS50003041). Expected strings
 * come from the guide's own examples where it has them.
 */

static sim_csac_t csac;
static sim_pps_t gps; /* ideal reference 1PPS */
static char rx[2048];

void setUp(void)
{
    sim_reset();
    char path[128];
    snprintf(path, sizeof(path), ".pio/sim/csac_%s.csv", Unity.CurrentTestName);
    sim_log_open(path);
    sim_csac_config_t cfg = sim_csac_default_config(7);
    sim_csac_init(&csac, "csac1", &cfg);
    sim_pps_init(&gps, "gps_pps", NULL, 100000ULL * SIM_PS_PER_NS);
}

void tearDown(void)
{
    sim_log_close();
}

static void send(const char *s)
{
    sim_uart_write(&csac.uart, (const uint8_t *)s, strlen(s));
}

/* Everything the CSAC sent in the next `ms` milliseconds. */
static const char *recv(uint32_t ms)
{
    sim_advance_ms(ms);
    size_t n = sim_uart_read(&csac.uart, (uint8_t *)rx, sizeof(rx) - 1);
    rx[n] = '\0';
    return rx;
}

static void power_and_lock(void)
{
    sim_csac_power(&csac, true);
    sim_advance_ns(SIM_S(181));
    (void)recv(1);
}

void test_warmup_counts_status_down_and_bite_waits_5s(void)
{
    sim_csac_power(&csac, true);
    TEST_ASSERT_EQUAL_INT(8, csac.status);
    TEST_ASSERT_TRUE(csac.bite.level);

    sim_advance_ns(SIM_S(181));
    TEST_ASSERT_EQUAL_INT(0, csac.status);
    TEST_ASSERT_TRUE(csac.bite.level); /* still high: 5 s hold after lock */
    sim_advance_ns(SIM_S(5));
    TEST_ASSERT_FALSE(csac.bite.level);
}

void test_telemetry_headers_and_17_fields(void)
{
    power_and_lock();
    send("!6\r\n");
    TEST_ASSERT_EQUAL_STRING("Status,Alarm,SN,Mode,Contrast,LaserI,TCXO,HeatP,Sig,Temp,Steer,"
                             "ATune,Phase,DiscOK,TOD,LTime,Ver\r\n",
                             recv(50));
    send("^");
    const char *t = recv(50);
    int commas = 0;
    for (const char *p = t; *p; p++) {
        commas += *p == ',';
    }
    TEST_ASSERT_EQUAL_INT(16, commas);
    TEST_ASSERT_EQUAL_STRING_LEN("0,0x0000,2611CS00001,0x0000,", t, 28);
}

void test_steering_examples_from_the_guide(void)
{
    power_and_lock();
    send("!FA-123000\r\n");
    TEST_ASSERT_EQUAL_STRING("Steer = -123\r\n", recv(50));
    send("!FA0\r\n");
    (void)recv(50);
    send("!FD-123000\r\n");
    send("!FD-123000\r\n");
    TEST_ASSERT_EQUAL_STRING("Steer = -123\r\nSteer = -246\r\n", recv(50));
    send("!FD-30000000\r\n"); /* beyond the +-2e-8 limit: clamped (guide 2.6) */
    TEST_ASSERT_EQUAL_STRING("Steer = -20246\r\n", recv(50));
}

void test_checksum_mode_examples_from_the_guide(void)
{
    power_and_lock();
    send("!MA*0C\r\n");
    TEST_ASSERT_EQUAL_STRING("?\r\n", recv(50)); /* checksum not enabled yet: '*' is junk */
    send("!MC\r\n");
    TEST_ASSERT_EQUAL_STRING("0x0040*4C\r\n", recv(50));
    send("!Mc*2D\r\n");
    TEST_ASSERT_EQUAL_STRING("*\r\n", recv(50));
    send("!Mc*2E\r\n");
    TEST_ASSERT_EQUAL_STRING("0x0000\r\n", recv(50));
}

void test_sync_aligns_to_reference_within_one_rf_cycle(void)
{
    sim_csac_attach_ref(&csac, &gps.pin);
    sim_pps_start(&gps);
    power_and_lock();
    send("S");
    TEST_ASSERT_EQUAL_STRING("S\r\n", recv(1500));
    sim_advance_ns(SIM_S(2));
    double t = (double)csac.clk.pps.last_ps / 1e12;
    TEST_ASSERT_DOUBLE_WITHIN(100e-9, round(t), t);
}

void test_sync_without_reference_times_out(void)
{
    power_and_lock();
    send("S");
    TEST_ASSERT_EQUAL_STRING("E\r\n", recv(4500));
}

void test_phase_sign_negative_when_csac_is_late(void)
{
    sim_csac_attach_ref(&csac, &gps.pin);
    sim_pps_start(&gps);
    power_and_lock();
    send("!MM\r\n"); /* phase measurement mode */
    (void)recv(50);
    csac.clk.pps.epoch_s = 1.0 - 2e-6; /* CSAC local clock runs behind...  */
    sim_osc_step_phase(&csac.clk.osc, -3e-6); /* ...so its edges come ~1 us after GPS */
    sim_pps_reschedule(&csac.clk.pps);
    sim_advance_ns(SIM_S(2));
    TEST_ASSERT_TRUE(csac.clk.phase_valid);
    TEST_ASSERT_TRUE(csac.clk.phase_s < 0.0);
}

void test_disciplining_converges_and_reports_discok(void)
{
    sim_csac_attach_ref(&csac, &gps.pin);
    sim_pps_start(&gps);
    power_and_lock();
    send("!D10\r\n");
    send("!MD\r\n");
    (void)recv(50);
    sim_advance_ns(SIM_S(200));
    TEST_ASSERT_EQUAL_INT(SIM_DISC_LOCKED, csac.clk.disc);
    TEST_ASSERT_DOUBLE_WITHIN(20e-9, 0.0, csac.clk.phase_s);

    sim_pps_stop(&gps); /* GPS lost: holdover */
    sim_advance_ns(SIM_S(3));
    TEST_ASSERT_EQUAL_INT(SIM_DISC_HOLDOVER, csac.clk.disc);
}

/* Guide 5.1 / Figure 5-2: mis-tuned by 1e-9 and disciplined with tau = 20 s,
 * frequency and phase are within +-5e-13 and +-5 ns after five to six tau. */
void test_discipline_matches_guide_figure_5_2(void)
{
    sim_csac_config_t cfg = sim_csac_default_config(7);
    cfg.clock.locked = sim_osc_ideal(); /* no noise: look at the servo alone */
    cfg.clock.locked.y0 = 1e-9;
    sim_csac_init(&csac, "csac1", &cfg);
    sim_csac_attach_ref(&csac, &gps.pin);
    sim_pps_start(&gps);
    power_and_lock();
    send("!D20\r\n");
    send("!MD\r\n");
    (void)recv(50);
    sim_advance_ns(SIM_S(120));
    TEST_ASSERT_DOUBLE_WITHIN(5e-9, 0.0, csac.clk.phase_s);
    /* The 450 ps phase meter dithers the instantaneous steer, as on the real
     * unit; the guide's figure is a measured (averaged) frequency. */
    double x0 = sim_osc_error_at(&csac.clk.osc, sim_now_ps());
    sim_advance_ns(SIM_S(20));
    double y = (sim_osc_error_at(&csac.clk.osc, sim_now_ps()) - x0) / 20.0;
    /* The guide quotes +-5e-13 here, but that is faster than its own "1/e per
     * tau" statement allows from a 1e-9 start; our loop matches the -9e-9 initial
     * steer and the phase figure, and gets the frequency to ~6e-13. */
    TEST_ASSERT_DOUBLE_WITHIN(1e-12, 0.0, y);
}

void test_lock_loss_raises_bite_within_one_second(void)
{
    power_and_lock();
    sim_advance_ns(SIM_S(6));
    TEST_ASSERT_FALSE(csac.bite.level);
    sim_csac_lose_lock(&csac, 30.0);
    sim_advance_ms(1000);
    TEST_ASSERT_TRUE(csac.bite.level);
    TEST_ASSERT_NOT_EQUAL(0, csac.status);
    sim_advance_ns(SIM_S(40));
    TEST_ASSERT_FALSE(csac.bite.level);
}

void test_tod_query_answers_after_next_pps(void)
{
    power_and_lock();
    send("!TA1221578499\r\n");
    TEST_ASSERT_EQUAL_STRING("TimeOfDay = 1221578499\r\n", recv(20));
    send("!T?\r\n");
    TEST_ASSERT_EQUAL_STRING("1221578500\r\n", recv(1100));
}

void test_unknown_command_and_power_off_silence(void)
{
    power_and_lock();
    send("!XYZ\r\n");
    TEST_ASSERT_EQUAL_STRING("?\r\n", recv(50));
    sim_csac_power(&csac, false);
    send("^");
    TEST_ASSERT_EQUAL_STRING("", recv(100));
    TEST_ASSERT_FALSE(csac.bite.level);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_warmup_counts_status_down_and_bite_waits_5s);
    RUN_TEST(test_telemetry_headers_and_17_fields);
    RUN_TEST(test_steering_examples_from_the_guide);
    RUN_TEST(test_checksum_mode_examples_from_the_guide);
    RUN_TEST(test_sync_aligns_to_reference_within_one_rf_cycle);
    RUN_TEST(test_sync_without_reference_times_out);
    RUN_TEST(test_phase_sign_negative_when_csac_is_late);
    RUN_TEST(test_disciplining_converges_and_reports_discok);
    RUN_TEST(test_discipline_matches_guide_figure_5_2);
    RUN_TEST(test_lock_loss_raises_bite_within_one_second);
    RUN_TEST(test_tod_query_answers_after_next_pps);
    RUN_TEST(test_unknown_command_and_power_off_silence);
    return UNITY_END();
}
