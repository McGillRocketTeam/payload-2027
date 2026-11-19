#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "devices/sim_gps.h"
#include "sim.h"
#include "sim_log.h"

/* NEO-M9N model: NMEA default, UBX configuration, fix model and timepulse. */

static sim_gps_t gps;
static uint8_t rx[16384];
static size_t rx_n;

static sim_signal_t accel[3];

void setUp(void)
{
    sim_reset();
    char path[128];
    snprintf(path, sizeof(path), ".pio/sim/gps_%s.csv", Unity.CurrentTestName);
    sim_log_open(path);
    accel[0] = sim_signal_const(0.0);
    accel[1] = sim_signal_const(0.0);
    accel[2] = sim_signal_const(1.0);
    sim_gps_config_t cfg = sim_gps_default_config();
    cfg.accel_g[0] = &accel[0];
    cfg.accel_g[1] = &accel[1];
    cfg.accel_g[2] = &accel[2];
    sim_gps_init(&gps, "gps", &cfg);
    gps.uart.rx_buffer = sizeof(rx);
}

void tearDown(void)
{
    sim_log_close();
}

static void drain(uint32_t ms)
{
    sim_advance_ms(ms);
    rx_n = sim_uart_read(&gps.uart, rx, sizeof(rx));
}

static void send_ubx(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len)
{
    uint8_t f[300];
    size_t n = sim_ubx_frame(cls, id, p, len, f);
    sim_uart_write(&gps.uart, f, n);
}

/* Find a UBX frame of cls/id in rx; returns payload pointer or NULL. */
static const uint8_t *find_ubx(uint8_t cls, uint8_t id, uint16_t *len)
{
    for (size_t i = 0; i + 8 <= rx_n; i++) {
        if (rx[i] == 0xB5 && rx[i + 1] == 0x62 && rx[i + 2] == cls && rx[i + 3] == id) {
            *len = (uint16_t)(rx[i + 4] | rx[i + 5] << 8);
            return rx + i + 6;
        }
    }
    return NULL;
}

static void valset(const uint32_t *keys, const uint32_t *vals, const uint8_t *sizes, int n)
{
    uint8_t p[128] = {0, 0x01, 0, 0}; /* version 0, RAM layer */
    uint16_t len = 4;
    for (int i = 0; i < n; i++) {
        for (int b = 0; b < 4; b++) {
            p[len++] = (uint8_t)(keys[i] >> (8 * b));
        }
        for (int b = 0; b < sizes[i]; b++) {
            p[len++] = (uint8_t)(vals[i] >> (8 * b));
        }
    }
    send_ubx(0x06, 0x8A, p, len);
}

/* Timepulse on the GPS grid: no wait for the UTC parameters. */
static void use_gps_grid(void)
{
    const uint32_t k[] = {GPS_KEY_TP_TIMEGRID_TP1};
    const uint32_t v[] = {1};
    const uint8_t s[] = {1};
    valset(k, v, s, 1);
}

static void switch_to_ubx(void)
{
    const uint32_t k[] = {GPS_KEY_UART1OUTPROT_NMEA, GPS_KEY_MSGOUT_NAV_PVT_UART1,
                          GPS_KEY_MSGOUT_TIM_TP_UART1};
    const uint32_t v[] = {0, 1, 1};
    const uint8_t s[] = {1, 1, 1};
    valset(k, v, s, 3);
}

void test_nmea_by_default_with_valid_checksum(void)
{
    sim_gps_power(&gps, true);
    drain(2000);
    rx[rx_n] = '\0';
    char *gga = strstr((char *)rx, "$GNGGA");
    TEST_ASSERT_NOT_NULL(gga);
    char *star = strchr(gga, '*');
    uint8_t x = 0;
    for (char *p = gga + 1; p < star; p++) {
        x ^= (uint8_t)*p;
    }
    unsigned ck;
    sscanf(star + 1, "%2X", &ck);
    TEST_ASSERT_EQUAL_HEX8(x, ck);
}

void test_valset_acks_and_switches_to_ubx(void)
{
    sim_gps_power(&gps, true);
    drain(100);
    switch_to_ubx();
    drain(100);
    uint16_t len;
    const uint8_t *ack = find_ubx(0x05, 0x01, &len);
    TEST_ASSERT_NOT_NULL(ack);
    TEST_ASSERT_EQUAL_HEX8(0x06, ack[0]);
    TEST_ASSERT_EQUAL_HEX8(0x8A, ack[1]);
    drain(2000);
    TEST_ASSERT_NOT_NULL(find_ubx(0x01, 0x07, &len));
    TEST_ASSERT_EQUAL_UINT16(92, len);
    rx[rx_n] = '\0';
    TEST_ASSERT_NULL(strstr((char *)rx, "$GN"));
}

void test_unknown_key_is_nakked(void)
{
    sim_gps_power(&gps, true);
    drain(100);
    const uint32_t k[] = {0x10AA0001u};
    const uint32_t v[] = {1};
    const uint8_t s[] = {1};
    valset(k, v, s, 1);
    drain(100);
    uint16_t len;
    TEST_ASSERT_NOT_NULL(find_ubx(0x05, 0x00, &len));
}

void test_no_timepulse_before_fix_then_on_whole_seconds(void)
{
    sim_gps_power(&gps, true);
    use_gps_grid();
    sim_advance_ns(SIM_S(20));
    TEST_ASSERT_EQUAL_UINT32(0, gps.pps.rises);
    sim_advance_ns(SIM_S(10));
    TEST_ASSERT_TRUE(gps.fix);
    TEST_ASSERT_TRUE(gps.pps.rises > 0);
    TEST_ASSERT_EQUAL_UINT64(0, gps.pps.last_rise_ps % SIM_PS_PER_S);
}

void test_nav_pvt_reports_fix_and_time(void)
{
    sim_gps_power(&gps, true);
    drain(100);
    switch_to_ubx();
    drain(30000);
    drain(1000);
    uint16_t len;
    const uint8_t *p = find_ubx(0x01, 0x07, &len);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_UINT8(3, p[20]);          /* fixType 3D */
    TEST_ASSERT_EQUAL_UINT8(0x01, p[21] & 0x01); /* gnssFixOK */
    TEST_ASSERT_EQUAL_UINT16(2027, (uint16_t)(p[4] | p[5] << 8));
    TEST_ASSERT_EQUAL_UINT8(18, p[23]);         /* numSV */
}

void test_loses_fix_under_boost_and_reacquires(void)
{
    sim_gps_power(&gps, true);
    use_gps_grid();
    sim_advance_ns(SIM_S(30));
    TEST_ASSERT_TRUE(gps.fix);

    accel[2] = sim_signal_const(6.0); /* boost: above the 4 g limit */
    sim_advance_ns(SIM_S(2));
    TEST_ASSERT_FALSE(gps.fix);
    uint32_t rises = gps.pps.rises;
    sim_advance_ns(SIM_S(3));
    TEST_ASSERT_EQUAL_UINT32(rises, gps.pps.rises); /* no pulse without fix */

    accel[2] = sim_signal_const(1.0);
    sim_advance_ns(SIM_S(4));
    TEST_ASSERT_TRUE(gps.fix); /* hot re-acquisition, not a 26 s cold start */
}

void test_portable_dynamic_model_drops_fix_when_climbing_fast(void)
{
    sim_gps_power(&gps, true);
    sim_advance_ns(SIM_S(30));
    gps.cfg.vel_d = sim_signal_const(-200.0);
    sim_advance_ns(SIM_S(2));
    TEST_ASSERT_FALSE(gps.fix);

    const uint32_t k[] = {GPS_KEY_NAVSPG_DYNMODEL};
    const uint32_t v[] = {8}; /* airborne < 4 g */
    const uint8_t s[] = {1};
    valset(k, v, s, 1);
    sim_advance_ns(SIM_S(4));
    TEST_ASSERT_TRUE(gps.fix);
}

void test_qerr_reported_matches_pulse_offset(void)
{
    gps.cfg.qerr_enabled = true;
    sim_gps_power(&gps, true);
    drain(100);
    switch_to_ubx();
    use_gps_grid();
    drain(29800); /* to t ~ 29.9 s: throw away the backlog */
    drain(500);   /* only the TIM-TP sent at 30.04 s, for the pulse at 31 s */
    uint16_t len;
    const uint8_t *p = find_ubx(0x0D, 0x01, &len);
    TEST_ASSERT_NOT_NULL(p);
    int32_t qerr_ps = (int32_t)((uint32_t)p[8] | (uint32_t)p[9] << 8 | (uint32_t)p[10] << 16 |
                                (uint32_t)p[11] << 24);
    sim_advance_ns(SIM_MS(600));
    double actual = (double)gps.pps.last_rise_ps / 1e12;
    double ideal = round(actual);
    TEST_ASSERT_DOUBLE_WITHIN(2e-12, ideal - actual, qerr_ps * 1e-12);
    TEST_ASSERT_TRUE(fabs(ideal - actual) < 11e-9); /* half a 48 MHz tick */
}

void test_cold_reset_loses_fix(void)
{
    sim_gps_power(&gps, true);
    sim_advance_ns(SIM_S(30));
    TEST_ASSERT_TRUE(gps.fix);
    const uint8_t rst[4] = {0xFF, 0xFF, 0x01, 0x00}; /* cold, controlled software reset */
    send_ubx(0x06, 0x04, rst, 4);
    sim_advance_ns(SIM_S(5));
    TEST_ASSERT_FALSE(gps.fix);
}

void test_mon_ver_poll(void)
{
    sim_gps_power(&gps, true);
    drain(100);
    send_ubx(0x0A, 0x04, NULL, 0);
    drain(100);
    uint16_t len;
    const uint8_t *p = find_ubx(0x0A, 0x04, &len);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_STRING("EXT CORE 4.04 (d964f4)", (const char *)p);
    TEST_ASSERT_EQUAL_STRING("PROTVER=32.01", (const char *)p + 70);
}

/* VALGET from the Default layer returns the documented default. */
void test_valget_default_layer_and_unknown_key(void)
{
    sim_gps_power(&gps, true);
    drain(100);
    uint8_t q[8] = {0, 7, 0, 0, 0x0C, 0x00, 0x05, 0x20}; /* CFG-TP-TIMEGRID_TP1 */
    send_ubx(0x06, 0x8B, q, 8);
    drain(100);
    uint16_t len;
    const uint8_t *p = find_ubx(0x06, 0x8B, &len);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_UINT16(9, len);
    TEST_ASSERT_EQUAL_UINT8(0, p[8]); /* 0 = UTC */

    uint8_t bad[8] = {0, 3, 0, 0, 0x0C, 0x00, 0x05, 0x20}; /* layer 3 does not exist */
    send_ubx(0x06, 0x8B, bad, 8);
    drain(100);
    TEST_ASSERT_NOT_NULL(find_ubx(0x05, 0x00, &len));
}

/* A real key the model does not simulate is still accepted (like the receiver). */
void test_valset_accepts_any_real_key(void)
{
    sim_gps_power(&gps, true);
    drain(100);
    const uint32_t k[] = {0x10310021u}; /* CFG-SIGNAL-GAL_ENA */
    const uint32_t v[] = {0};
    const uint8_t s[] = {1};
    valset(k, v, s, 1);
    drain(100);
    uint16_t len;
    TEST_ASSERT_NOT_NULL(find_ubx(0x05, 0x01, &len));
    TEST_ASSERT_EQUAL_UINT64(0, sim_gps_get_key(&gps, 0x10310021u));
}

/* VALSET v1: staged values only apply at the "apply" step. */
void test_valset_transaction(void)
{
    sim_gps_power(&gps, true);
    drain(100);
    uint8_t start[9] = {1, 0x01, 1, 0, 0x07, 0x00, 0x91, 0x20, 1}; /* NAV-PVT rate 1 */
    send_ubx(0x06, 0x8A, start, 9);
    drain(50);
    TEST_ASSERT_EQUAL_UINT64(0, sim_gps_get_key(&gps, GPS_KEY_MSGOUT_NAV_PVT_UART1));
    uint8_t apply[4] = {1, 0x01, 3, 0};
    send_ubx(0x06, 0x8A, apply, 4);
    drain(50);
    TEST_ASSERT_EQUAL_UINT64(1, sim_gps_get_key(&gps, GPS_KEY_MSGOUT_NAV_PVT_UART1));
}

/* Default time grid is UTC: TIM-TP reports UTC time of week, timeBase = 1. */
void test_tim_tp_uses_utc_time_base_by_default(void)
{
    sim_gps_power(&gps, true);
    drain(100);
    switch_to_ubx();
    drain(29800);
    drain(500);
    uint16_t len;
    const uint8_t *p = find_ubx(0x0D, 0x01, &len);
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_HEX8(0x01, p[14] & 0x01);
    uint32_t tow_ms = (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
    double gps_s = gps.cfg.gps_start_s + 31.0;
    double utc_tow = fmod(gps_s - gps.cfg.leap_s, 604800.0);
    TEST_ASSERT_EQUAL_UINT32((uint32_t)llround(utc_tow * 1000.0), tow_ms);
}

/* Default UTC grid after a cold start: no pulse until the UTC parameters
 * arrive, at most 12.5 min after the first fix (integration manual 3.13.1.3). */
void test_utc_grid_waits_for_utc_parameters(void)
{
    sim_gps_power(&gps, true);
    sim_advance_ns(SIM_S(30));
    TEST_ASSERT_TRUE(gps.fix);
    if (!gps.utc_known) {
        TEST_ASSERT_EQUAL_UINT32(0, gps.pps.rises);
    }
    sim_advance_ns(SIM_S(760));
    TEST_ASSERT_TRUE(gps.utc_known);
    TEST_ASSERT_TRUE(gps.pps.rises > 0);
}

/* Airborne <4g checks altitude only (table 4): fast climb keeps the fix. */
void test_airborne_model_ignores_velocity(void)
{
    sim_gps_power(&gps, true);
    const uint32_t k[] = {GPS_KEY_NAVSPG_DYNMODEL};
    const uint32_t v[] = {8};
    const uint8_t s[] = {1};
    valset(k, v, s, 1);
    sim_advance_ns(SIM_S(30));
    gps.cfg.vel_d = sim_signal_const(-400.0);
    sim_advance_ns(SIM_S(3));
    TEST_ASSERT_TRUE(gps.fix);
    gps.cfg.alt_m = sim_signal_const(85000.0); /* above 80 km */
    sim_advance_ns(SIM_S(2));
    TEST_ASSERT_FALSE(gps.fix);
}

/* Backup power: hot start within 4 h of power-off, warm (cold time) after. */
void test_backup_power_hot_then_warm_start(void)
{
    gps.cfg.backup_power = true;
    sim_gps_power(&gps, true);
    sim_advance_ns(SIM_S(30));
    TEST_ASSERT_TRUE(gps.fix);

    sim_gps_power(&gps, false);
    sim_advance_ns(SIM_S(600));
    sim_gps_power(&gps, true);
    sim_advance_ns(SIM_S(4));
    TEST_ASSERT_TRUE(gps.fix); /* hot: ~2 s */

    sim_gps_power(&gps, false);
    sim_advance_ns(SIM_S(5 * 3600));
    sim_gps_power(&gps, true);
    sim_advance_ns(SIM_S(4));
    TEST_ASSERT_FALSE(gps.fix); /* warm: ephemeris out of date */
    sim_advance_ns(SIM_S(25));
    TEST_ASSERT_TRUE(gps.fix);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_nmea_by_default_with_valid_checksum);
    RUN_TEST(test_valset_acks_and_switches_to_ubx);
    RUN_TEST(test_unknown_key_is_nakked);
    RUN_TEST(test_no_timepulse_before_fix_then_on_whole_seconds);
    RUN_TEST(test_nav_pvt_reports_fix_and_time);
    RUN_TEST(test_loses_fix_under_boost_and_reacquires);
    RUN_TEST(test_portable_dynamic_model_drops_fix_when_climbing_fast);
    RUN_TEST(test_qerr_reported_matches_pulse_offset);
    RUN_TEST(test_cold_reset_loses_fix);
    RUN_TEST(test_mon_ver_poll);
    RUN_TEST(test_valget_default_layer_and_unknown_key);
    RUN_TEST(test_valset_accepts_any_real_key);
    RUN_TEST(test_valset_transaction);
    RUN_TEST(test_tim_tp_uses_utc_time_base_by_default);
    RUN_TEST(test_utc_grid_waits_for_utc_parameters);
    RUN_TEST(test_airborne_model_ignores_velocity);
    RUN_TEST(test_backup_power_hot_then_warm_start);
    return UNITY_END();
}
