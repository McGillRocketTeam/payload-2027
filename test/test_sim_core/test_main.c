#include <string.h>
#include <unity.h>
#include <stdio.h>
#include <math.h>

#include "sim.h"
#include "sim_log.h"
#include "sim_device.h"
#include "sim_signal.h"
#include "sim_uart.h"

/* Tests for the simulator itself: time, fault injection, signals, UART. */

static sim_device_t dev;
static sim_uart_t uart; /* big: keep off the stack */

void setUp(void)
{
    sim_reset();
    sim_device_init(&dev, "dev");
}

void tearDown(void) {}

void test_delay_advances_virtual_time(void)
{
    sim_delay_ms(NULL, 250);
    TEST_ASSERT_EQUAL_UINT64(SIM_MS(250), sim_now_ns());
}

void test_no_fault_by_default(void)
{
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_NONE, sim_device_begin(&dev));
    TEST_ASSERT_EQUAL_UINT32(1, dev.transfers);
    TEST_ASSERT_EQUAL_UINT32(0, dev.faulted_transfers);
}

void test_unpowered_device_is_off(void)
{
    sim_device_power(&dev, false);
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_OFF, sim_device_begin(&dev));
}

static int power_on_calls;
static void count_power_on(sim_device_t *d)
{
    (void)d;
    power_on_calls++;
}

void test_power_on_edge_resets_device(void)
{
    power_on_calls = 0;
    dev.on_power_on = count_power_on;
    sim_device_power(&dev, true); /* already on: no edge */
    sim_device_power(&dev, false);
    sim_device_power(&dev, true);
    TEST_ASSERT_EQUAL_INT(1, power_on_calls);
}

void test_fault_window_starts_and_ends(void)
{
    sim_inject_at(&dev, SIM_FAULT_BUS_ERROR, SIM_MS(100), SIM_MS(50));

    TEST_ASSERT_EQUAL_INT(SIM_FAULT_NONE, sim_device_begin(&dev));
    sim_advance_ms(100);
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_BUS_ERROR, sim_device_begin(&dev));
    sim_advance_ms(49);
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_BUS_ERROR, sim_device_begin(&dev));
    sim_advance_ms(1);
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_NONE, sim_device_begin(&dev));
}

void test_fault_for_next_n_transfers(void)
{
    sim_inject_next(&dev, SIM_FAULT_CORRUPT, 2);
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_CORRUPT, sim_device_begin(&dev));
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_CORRUPT, sim_device_begin(&dev));
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_NONE, sim_device_begin(&dev));
}

void test_random_fault_rate_is_roughly_right(void)
{
    sim_inject_random(&dev, SIM_FAULT_BUS_ERROR, 0.1);
    int hits = 0;
    for (int i = 0; i < 10000; i++) {
        hits += sim_device_begin(&dev) == SIM_FAULT_BUS_ERROR;
    }
    TEST_ASSERT_INT_WITHIN(150, 1000, hits);
}

void test_stall_advances_time(void)
{
    sim_inject_next(&dev, SIM_FAULT_STALL, 1)->stall_ns = SIM_MS(250);
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_STALL, sim_device_begin(&dev));
    TEST_ASSERT_EQUAL_UINT64(SIM_MS(250), sim_now_ns());
}

void test_off_beats_other_faults(void)
{
    sim_inject(&dev, SIM_FAULT_CORRUPT);
    sim_inject(&dev, SIM_FAULT_OFF);
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_OFF, sim_device_begin(&dev));
}

void test_clear_faults(void)
{
    sim_inject(&dev, SIM_FAULT_OFF);
    sim_clear_faults(&dev);
    TEST_ASSERT_EQUAL_INT(SIM_FAULT_NONE, sim_device_begin(&dev));
}

void test_signals(void)
{
    sim_signal_t ramp = sim_signal_ramp(20.0, 2.0);
    sim_signal_t step = sim_signal_step(1.0, 5.0, 2.0);
    const double t[] = {0.0, 1.0, 2.0};
    const double v[] = {0.0, 10.0, 0.0};
    sim_signal_t table = sim_signal_table(t, v, 3);

    sim_advance_ns(SIM_MS(1500));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 23.0, sim_signal_eval(&ramp, sim_now_ns()));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, sim_signal_eval(&step, sim_now_ns()));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 5.0, sim_signal_eval(&table, sim_now_ns()));
    sim_advance_ns(SIM_S(10));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 5.0, sim_signal_eval(&step, sim_now_ns()));
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0, sim_signal_eval(&table, sim_now_ns())); /* held */
}

/* ---- UART ---- */

static uint8_t last_cmd[32];
static size_t last_cmd_len;

static void echo_device(sim_uart_t *u, const uint8_t *buf, size_t len, void *user)
{
    (void)user;
    memcpy(last_cmd, buf, len < sizeof(last_cmd) ? len : sizeof(last_cmd));
    last_cmd_len = len;
    sim_uart_send(u, buf, len);
}

void test_uart_bytes_arrive_at_baud_rate(void)
{
    sim_uart_init(&uart, "uart", 9600); /* ~1.04 ms per byte */
    sim_uart_send_str(&uart, "HELLO");

    uint8_t buf[8];
    TEST_ASSERT_EQUAL_size_t(0, sim_uart_read(&uart, buf, sizeof(buf)));
    sim_advance_ns(SIM_US(2100));
    TEST_ASSERT_EQUAL_size_t(2, sim_uart_read(&uart, buf, sizeof(buf)));
    sim_advance_ms(10);
    TEST_ASSERT_EQUAL_size_t(3, sim_uart_read(&uart, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_MEMORY("LLO", buf, 3);
}

void test_uart_device_answers_commands(void)
{
    sim_uart_init(&uart, "uart", 115200);
    uart.on_rx = echo_device;

    TEST_ASSERT_EQUAL_INT(0, sim_uart_write(&uart, (const uint8_t *)"!^", 2));
    TEST_ASSERT_EQUAL_size_t(2, last_cmd_len);
    sim_advance_ms(1);
    uint8_t buf[4];
    TEST_ASSERT_EQUAL_size_t(2, sim_uart_read(&uart, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_MEMORY("!^", buf, 2);
}

void test_uart_overrun_when_mcu_reads_too_rarely(void)
{
    sim_uart_init(&uart, "uart", 115200);
    uart.rx_buffer = 64;
    uint8_t block[200] = {0};
    sim_uart_send(&uart, block, sizeof(block));
    sim_advance_ms(100);

    uint8_t buf[256];
    TEST_ASSERT_EQUAL_size_t(64, sim_uart_read(&uart, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_UINT64(136, uart.overrun_bytes);
}

void test_uart_off_is_silent(void)
{
    sim_uart_init(&uart, "uart", 115200);
    sim_inject(&uart.base, SIM_FAULT_OFF);
    sim_uart_send_str(&uart, "data");
    sim_advance_ms(10);
    uint8_t buf[8];
    TEST_ASSERT_EQUAL_size_t(0, sim_uart_read(&uart, buf, sizeof(buf)));
}

void test_ps_after_s_rejects_unrepresentable_times(void)
{
    sim_advance_ms(1);
    sim_time_ps_t t = 42;
    TEST_ASSERT_FALSE(sim_ps_after_s(-1.0, &t));
    TEST_ASSERT_FALSE(sim_ps_after_s(NAN, &t));
    TEST_ASSERT_FALSE(sim_ps_after_s(INFINITY, &t));
    TEST_ASSERT_FALSE(sim_ps_after_s(1e9, &t)); /* 1e21 ps: does not fit */
    TEST_ASSERT_EQUAL_UINT64(42, t);            /* untouched on failure */
    TEST_ASSERT_TRUE(sim_ps_after_s(1.5, &t));
    TEST_ASSERT_EQUAL_UINT64(SIM_MS(1) * SIM_PS_PER_NS + 1500000000000ULL, t);
}

static void noop_event(void *ctx, uint32_t tag)
{
    (void)ctx;
    (void)tag;
}

void test_null_event_fn_is_logged_as_such(void)
{
    const char *path = ".pio/sim/core_null_event_fn.csv";
    TEST_ASSERT_TRUE(sim_log_open(path));
    TEST_ASSERT_TRUE(sim_schedule_ps(10, noop_event, NULL, 0));
    TEST_ASSERT_FALSE(sim_schedule_ps(10, NULL, NULL, 0));
    TEST_ASSERT_EQUAL_size_t(1, sim_pending_events());
    sim_log_close();

    char text[4096] = "";
    FILE *f = fopen(path, "r");
    TEST_ASSERT_NOT_NULL(f);
    size_t n = fread(text, 1, sizeof(text) - 1, f);
    fclose(f);
    text[n] = '\0';
    TEST_ASSERT_NOT_NULL(strstr(text, "null_event_fn"));
    TEST_ASSERT_NULL(strstr(text, "event_queue_full"));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_delay_advances_virtual_time);
    RUN_TEST(test_no_fault_by_default);
    RUN_TEST(test_unpowered_device_is_off);
    RUN_TEST(test_power_on_edge_resets_device);
    RUN_TEST(test_fault_window_starts_and_ends);
    RUN_TEST(test_fault_for_next_n_transfers);
    RUN_TEST(test_random_fault_rate_is_roughly_right);
    RUN_TEST(test_stall_advances_time);
    RUN_TEST(test_off_beats_other_faults);
    RUN_TEST(test_clear_faults);
    RUN_TEST(test_signals);
    RUN_TEST(test_uart_bytes_arrive_at_baud_rate);
    RUN_TEST(test_uart_device_answers_commands);
    RUN_TEST(test_uart_overrun_when_mcu_reads_too_rarely);
    RUN_TEST(test_uart_off_is_silent);
    RUN_TEST(test_ps_after_s_rejects_unrepresentable_times);
    RUN_TEST(test_null_event_fn_is_logged_as_such);
    return UNITY_END();
}
