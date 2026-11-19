#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "devices/sim_mac.h"
#include "sim.h"
#include "sim_log.h"
#include "sim_pps.h"

/*
 * SA5x MAC model against the user guide (DS50002938). Expected strings come
 * from the guide's own examples where it has them.
 */

static sim_mac_t mac;
static sim_pps_t gps;
static char rx[4096];

void setUp(void)
{
    sim_reset();
    char path[128];
    snprintf(path, sizeof(path), ".pio/sim/mac_%s.csv", Unity.CurrentTestName);
    sim_log_open(path);
    sim_mac_config_t cfg = sim_mac_default_config(11);
    sim_mac_init(&mac, "mac1", &cfg);
    sim_pps_init(&gps, "gps_pps", NULL, 100000ULL * SIM_PS_PER_NS);
}

void tearDown(void)
{
    sim_log_close();
}

static void send(const char *s)
{
    sim_uart_write(&mac.uart, (const uint8_t *)s, strlen(s));
}

static const char *recv(uint32_t ms)
{
    sim_advance_ms(ms);
    size_t n = sim_uart_read(&mac.uart, (uint8_t *)rx, sizeof(rx) - 1);
    rx[n] = '\0';
    return rx;
}

static const char *ask(const char *cmd)
{
    send(cmd);
    return recv(50);
}

static void power_up(void)
{
    sim_mac_power(&mac, true);
    (void)recv(3100);
}

void test_unpowered_mac_is_silent_and_announces_on_power_up(void)
{
    TEST_ASSERT_EQUAL_STRING("", ask("{device?}"));
    sim_mac_power(&mac, true);
    TEST_ASSERT_EQUAL_STRING("[>Loading...]\r\n", recv(100));
    TEST_ASSERT_EQUAL_STRING("", ask("{device?}")); /* bootloader window: ignored */
    TEST_ASSERT_EQUAL_STRING("[>Microchip SA5X]\r\n", recv(3000));
    TEST_ASSERT_TRUE(mac.bite.level); /* active low: not locked */
}

void test_checksum_example_from_the_guide(void)
{
    power_up();
    TEST_ASSERT_EQUAL_STRING("[=sa5x|62]\r\n", ask("{device?|27}"));
    TEST_ASSERT_EQUAL_STRING("[!3]\r\n", ask("{device?|28}"));
    TEST_ASSERT_EQUAL_STRING("[#0A=sa5x]\r\n", ask("{device?#0A}"));
}

void test_error_responses(void)
{
    power_up();
    TEST_ASSERT_EQUAL_STRING("[!1]\r\n", ask("{type7}"));
    TEST_ASSERT_EQUAL_STRING("[!2]\r\n", ask("{get}"));
    TEST_ASSERT_EQUAL_STRING("[!100]\r\n", ask("{get,Nope}"));
    TEST_ASSERT_EQUAL_STRING("[!102]\r\n", ask("{set,Locked,1}"));
    TEST_ASSERT_EQUAL_STRING("[!101]\r\n", ask("{set,TauPps0,5}"));
}

void test_lock_after_warmup_by_name_or_id(void)
{
    power_up();
    TEST_ASSERT_EQUAL_STRING("[=0]\r\n", ask("{get,Locked}"));
    sim_advance_ns(SIM_S(300));
    TEST_ASSERT_EQUAL_STRING("[=1]\r\n", ask("{get,263}"));
    TEST_ASSERT_EQUAL_STRING("[=100]\r\n", ask("{get,LockProgress}"));
    TEST_ASSERT_FALSE(mac.bite.level);
}

void test_set_add_and_settings_survive_only_with_store(void)
{
    power_up();
    TEST_ASSERT_EQUAL_STRING("[=500]\r\n", ask("{set,TauPps0,500}"));
    TEST_ASSERT_EQUAL_STRING("[=600]\r\n", ask("{add,TauPps0,100}"));
    sim_mac_power(&mac, false);
    power_up();
    TEST_ASSERT_EQUAL_STRING("[=100]\r\n", ask("{get,TauPps0}")); /* volatile */
    ask("{set,TauPps0,700}");
    TEST_ASSERT_EQUAL_STRING("[=1]\r\n", ask("{store}"));
    sim_mac_power(&mac, false);
    power_up();
    TEST_ASSERT_EQUAL_STRING("[=700]\r\n", ask("{get,TauPps0}"));
}

void test_metering_and_disciplining_are_exclusive(void)
{
    power_up();
    TEST_ASSERT_EQUAL_STRING("[=1]\r\n", ask("{set,PhaseMetering,1}"));
    TEST_ASSERT_EQUAL_STRING("[!101]\r\n", ask("{set,Disciplining,1}"));
}

void test_no_pps_alarm_drives_alarm_pin_until_acknowledged(void)
{
    power_up();
    TEST_ASSERT_TRUE(mac.alarm.level);
    ask("{set,Disciplining,1}");
    sim_advance_ns(SIM_S(2));
    TEST_ASSERT_EQUAL_STRING("[=131072]\r\n", ask("{get,Alarms}"));
    TEST_ASSERT_FALSE(mac.alarm.level); /* active low */
    TEST_ASSERT_EQUAL_STRING("[=1]\r\n", ask("{ackalm,131072}"));
    TEST_ASSERT_TRUE(mac.alarm.level);
    TEST_ASSERT_EQUAL_STRING("[=131072]\r\n", ask("{get,Alarms}")); /* still active */
}

void test_disciplines_to_gps_and_reports_lock(void)
{
    sim_mac_attach_ref(&mac, 0, &gps.pin);
    sim_pps_start(&gps);
    power_up();
    sim_advance_ns(SIM_S(300));
    ask("{set,TauPps0,10}");
    ask("{set,Disciplining,1}");
    sim_advance_ns(SIM_S(120));
    TEST_ASSERT_EQUAL_STRING("[=1]\r\n", ask("{get,DisciplineLocked}"));
    double t = (double)mac.clk.pps.last_ps / 1e12;
    TEST_ASSERT_DOUBLE_WITHIN(20e-9, round(t), t);
    TEST_ASSERT_EQUAL_STRING("[=0]\r\n", ask("{get,Alarms}"));
}

void test_pps_source_selects_input(void)
{
    sim_pin_t other;
    sim_pin_init(&other, "other", false);
    sim_mac_attach_ref(&mac, 1, &gps.pin);
    sim_pps_start(&gps);
    power_up();
    TEST_ASSERT_EQUAL_STRING("[=0]\r\n", ask("{get,PpsInDetected}")); /* input 0 is empty */
    ask("{set,PpsSource,1}");
    sim_advance_ns(SIM_S(2));
    TEST_ASSERT_EQUAL_STRING("[=1]\r\n", ask("{get,PpsInDetected}"));
}

void test_upd_reports_only_changes(void)
{
    power_up();
    ask("{upd}");
    ask("{set,PpsWidth,30000}");
    const char *r = ask("{upd}");
    TEST_ASSERT_NOT_NULL(strstr(r, ",513,30000"));
    TEST_ASSERT_NULL(strstr(r, ",770,"));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_unpowered_mac_is_silent_and_announces_on_power_up);
    RUN_TEST(test_checksum_example_from_the_guide);
    RUN_TEST(test_error_responses);
    RUN_TEST(test_lock_after_warmup_by_name_or_id);
    RUN_TEST(test_set_add_and_settings_survive_only_with_store);
    RUN_TEST(test_metering_and_disciplining_are_exclusive);
    RUN_TEST(test_no_pps_alarm_drives_alarm_pin_until_acknowledged);
    RUN_TEST(test_disciplines_to_gps_and_reports_lock);
    RUN_TEST(test_pps_source_selects_input);
    RUN_TEST(test_upd_reports_only_changes);
    return UNITY_END();
}
