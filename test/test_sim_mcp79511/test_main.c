#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "devices/sim_mcp79511.h"
#include "sim.h"
#include "sim_log.h"
#include "sim_osc.h"

/* MCP79511 model against the data sheet (DS20002300D), through raw SPI. */

static sim_mcp79511_t rtc;
static sim_osc_t tcxo;

#define READ 0x13
#define WRITE 0x12

static sim_time_ps_t edges[200];
static int n_edges;

static void on_mfp(void *ctx, bool rising, sim_time_ps_t t)
{
    (void)ctx;
    if (rising && n_edges < 200) {
        edges[n_edges++] = t;
    }
}

void setUp(void)
{
    sim_reset();
    char path[128];
    snprintf(path, sizeof(path), ".pio/sim/mcp_%s.csv", Unity.CurrentTestName);
    sim_log_open(path);
    sim_osc_config_t c = sim_osc_ideal();
    sim_osc_init(&tcxo, "tcxo1", &c, 0.0);
    sim_mcp79511_config_t cfg = sim_mcp79511_default_config(&tcxo);
    sim_mcp79511_init(&rtc, "rtc1", &cfg);
    n_edges = 0;
    sim_pin_listen(&rtc.mfp, on_mfp, NULL);
}

void tearDown(void)
{
    sim_log_close();
}

static void wr(uint8_t a, uint8_t v)
{
    uint8_t tx[3] = {WRITE, a, v};
    uint8_t rx[3];
    sim_mcp79511_spi_transfer(&rtc, tx, rx, 3);
}

static uint8_t rd(uint8_t a)
{
    uint8_t tx[3] = {READ, a, 0};
    uint8_t rx[3];
    sim_mcp79511_spi_transfer(&rtc, tx, rx, 3);
    return rx[2];
}

static void cmd1(uint8_t ins)
{
    uint8_t rx[1];
    sim_mcp79511_spi_transfer(&rtc, &ins, rx, 1);
}

/* External clock on X1 + 1 Hz square wave on MFP. */
static void start_1hz(void)
{
    wr(0x08, 0x48); /* SQWEN | EXTOSC, SQWFS = 00 (1 Hz) */
}

void test_por_values(void)
{
    TEST_ASSERT_EQUAL_HEX8(0x01, rd(0x04) & 0x07); /* weekday 1 */
    TEST_ASSERT_EQUAL_HEX8(0x01, rd(0x05));
    TEST_ASSERT_EQUAL_HEX8(0x01, rd(0x06) & 0x1F);
    TEST_ASSERT_EQUAL_HEX8(0x80, rd(0x08)); /* bit 7 reads as 1 */
    TEST_ASSERT_EQUAL_HEX8(0x00, rd(0x09));
}

void test_sram_block_wraps_and_unimplemented_addresses_do_not_answer(void)
{
    uint8_t tx[4] = {WRITE, 0x5F, 0xAB, 0xCD};
    uint8_t rx[4];
    sim_mcp79511_spi_transfer(&rtc, tx, rx, 4);
    TEST_ASSERT_EQUAL_HEX8(0xAB, rd(0x5F));
    TEST_ASSERT_EQUAL_HEX8(0xCD, rd(0x20)); /* wrapped 5Fh -> 20h */
    TEST_ASSERT_EQUAL_HEX8(0xFF, rd(0x60)); /* no response */
    cmd1(0x54);                              /* CLRRAM */
    uint8_t tx2[2] = {0x54, 0};
    sim_mcp79511_spi_transfer(&rtc, tx2, rx, 2);
    TEST_ASSERT_EQUAL_HEX8(0x00, rd(0x20));
}

void test_one_hz_output_follows_the_input_clock(void)
{
    sim_osc_config_t c = sim_osc_ideal();
    c.y0 = 2e-6; /* TCXO 2 ppm fast */
    sim_osc_init(&tcxo, "tcxo1", &c, 0.0);
    start_1hz();
    sim_advance_ms(1); /* OSCRUN after 32 cycles */
    TEST_ASSERT_EQUAL_HEX8(0x20, rd(0x04) & 0x20);
    sim_advance_ns(SIM_S(11));
    TEST_ASSERT_TRUE(n_edges >= 10);
    double period = (double)(edges[9] - edges[0]) / 9e12;
    TEST_ASSERT_DOUBLE_WITHIN(1e-11, 1.0 / (1.0 + 2e-6), period);
}

void test_calendar_rollover(void)
{
    wr(0x03, 0x23); /* 23 h, 24-hour mode */
    wr(0x02, 0x59);
    wr(0x01, 0x58);
    wr(0x05, 0x31);
    wr(0x06, 0x12);
    wr(0x07, 0x27); /* 2027-12-31 23:59:58 */
    wr(0x04, 0x07); /* weekday 7 */
    start_1hz();
    sim_advance_ns(SIM_S(3) + SIM_MS(100));
    TEST_ASSERT_EQUAL_HEX8(0x01, rd(0x01) & 0x7F);
    TEST_ASSERT_EQUAL_HEX8(0x00, rd(0x02));
    TEST_ASSERT_EQUAL_HEX8(0x00, rd(0x03) & 0x3F);
    TEST_ASSERT_EQUAL_HEX8(0x01, rd(0x05));
    TEST_ASSERT_EQUAL_HEX8(0x01, rd(0x06) & 0x1F);
    TEST_ASSERT_EQUAL_HEX8(0x28, rd(0x07));
    TEST_ASSERT_EQUAL_HEX8(0x20, rd(0x06) & 0x20); /* 2028 is a leap year */
    TEST_ASSERT_EQUAL_HEX8(0x01, rd(0x04) & 0x07); /* weekday 7 -> 1 */
}

void test_once_a_minute_trim_shortens_one_second(void)
{
    wr(0x09, 10);   /* TRIMVAL = 10 -> 20 cycles */
    wr(0x03, 0x80); /* TRIMSIGN = 1: add cycles */
    start_1hz();
    sim_advance_ns(SIM_S(125));
    int short_seconds = 0;
    for (int i = 1; i < n_edges; i++) {
        double p = (double)(edges[i] - edges[i - 1]) / 1e12;
        if (fabs(p - (32768.0 - 20.0) / 32768.0) < 1e-9) {
            short_seconds++;
        } else {
            TEST_ASSERT_DOUBLE_WITHIN(1e-9, 1.0, p);
        }
    }
    TEST_ASSERT_EQUAL_INT(2, short_seconds); /* two minutes, one trimmed second each */
}

void test_coarse_trim_every_second(void)
{
    wr(0x09, 1);    /* 2 cycles */
    wr(0x03, 0x00); /* TRIMSIGN = 0: subtract cycles, seconds get longer */
    wr(0x08, 0x4C); /* SQWEN | EXTOSC | CRSTRIM */
    sim_advance_ns(SIM_S(6));
    double p = (double)(edges[3] - edges[2]) / 1e12;
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, (32768.0 + 2.0) / 32768.0, p);
}

void test_hundredths_count(void)
{
    start_1hz();
    sim_advance_ns(SIM_S(2));
    sim_time_ps_t tick = edges[n_edges - 1];
    sim_run_until_ps(tick + 500000000000ULL); /* half a second after the tick */
    TEST_ASSERT_EQUAL_HEX8(0x50, rd(0x00));
}

void test_eeprom_needs_wren_and_has_a_write_cycle(void)
{
    uint8_t tx[4] = {0x02, 0x10, 0x11, 0x22}; /* EEWRITE without EEWREN */
    uint8_t rx[4];
    sim_mcp79511_spi_transfer(&rtc, tx, rx, 4);
    uint8_t rd_tx[4] = {0x03, 0x10, 0, 0};
    sim_mcp79511_spi_transfer(&rtc, rd_tx, rx, 4);
    TEST_ASSERT_EQUAL_HEX8(0xFF, rx[2]);

    cmd1(0x06); /* EEWREN */
    sim_mcp79511_spi_transfer(&rtc, tx, rx, 4);
    uint8_t sr[2] = {0x05, 0};
    sim_mcp79511_spi_transfer(&rtc, sr, rx, 2);
    TEST_ASSERT_EQUAL_HEX8(0x01, rx[1] & 0x03); /* WIP set, WEL cleared */
    sim_advance_ms(6);
    sim_mcp79511_spi_transfer(&rtc, sr, rx, 2);
    TEST_ASSERT_EQUAL_HEX8(0x00, rx[1] & 0x03);
    sim_mcp79511_spi_transfer(&rtc, rd_tx, rx, 4);
    TEST_ASSERT_EQUAL_HEX8(0x11, rx[2]);
    TEST_ASSERT_EQUAL_HEX8(0x22, rx[3]);
}

void test_eeprom_page_wraps_inside_8_bytes(void)
{
    cmd1(0x06);
    uint8_t tx[5] = {0x02, 0x0E, 0xA1, 0xA2, 0xA3}; /* 0E, 0F, then wraps to 08 */
    uint8_t rx[5];
    sim_mcp79511_spi_transfer(&rtc, tx, rx, 5);
    sim_advance_ms(6);
    uint8_t rd_tx[3] = {0x03, 0x08, 0};
    sim_mcp79511_spi_transfer(&rtc, rd_tx, rx, 3);
    TEST_ASSERT_EQUAL_HEX8(0xA3, rx[2]);
}

void test_eui48_preprogrammed(void)
{
    uint8_t tx[8] = {0x33, 0x02, 0, 0, 0, 0, 0, 0};
    uint8_t rx[8];
    sim_mcp79511_spi_transfer(&rtc, tx, rx, 8);
    TEST_ASSERT_EQUAL_MEMORY(rtc.cfg.eui48, rx + 2, 6);
}

void test_crystal_mode_with_an_external_clock_is_flagged(void)
{
    wr(0x01, 0x80); /* ST only */
    sim_advance_ns(SIM_S(3));
    TEST_ASSERT_EQUAL_INT(0, n_edges);
    TEST_ASSERT_EQUAL_UINT32(1, sim_log_count("rtc1", "error"));
}

void test_invalid_month_zero_rolls_over_safely(void)
{
    wr(0x06, 0x00); /* month 0: invalid, e.g. from a zeroed time struct */
    wr(0x05, 0x31);
    wr(0x03, 0x23);
    wr(0x02, 0x59);
    wr(0x01, 0x59);
    start_1hz();
    sim_advance_ms(1500); /* first tick: 23:59:59 -> 00:00:00, next day */
    TEST_ASSERT_EQUAL_HEX8(0x00, rd(0x03) & 0x3F);
    TEST_ASSERT_EQUAL_HEX8(0x01, rd(0x05));
    TEST_ASSERT_EQUAL_HEX8(0x01, rd(0x06) & 0x1F);
}

void test_stopping_the_oscillator_cancels_the_pending_falling_edge(void)
{
    start_1hz();
    sim_advance_ms(2200); /* rose at 2.0 s, falling edge due at 2.5 s */
    TEST_ASSERT_TRUE(rtc.mfp.level);
    uint32_t falls = rtc.mfp.falls;
    wr(0x08, 0x40); /* EXTOSC off, SQWEN kept: the clock stops */
    sim_advance_ms(1300);
    TEST_ASSERT_TRUE(rtc.mfp.level);
    TEST_ASSERT_EQUAL_UINT32(falls, rtc.mfp.falls);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_por_values);
    RUN_TEST(test_sram_block_wraps_and_unimplemented_addresses_do_not_answer);
    RUN_TEST(test_one_hz_output_follows_the_input_clock);
    RUN_TEST(test_calendar_rollover);
    RUN_TEST(test_once_a_minute_trim_shortens_one_second);
    RUN_TEST(test_coarse_trim_every_second);
    RUN_TEST(test_hundredths_count);
    RUN_TEST(test_eeprom_needs_wren_and_has_a_write_cycle);
    RUN_TEST(test_eeprom_page_wraps_inside_8_bytes);
    RUN_TEST(test_eui48_preprogrammed);
    RUN_TEST(test_crystal_mode_with_an_external_clock_is_flagged);
    RUN_TEST(test_invalid_month_zero_rolls_over_safely);
    RUN_TEST(test_stopping_the_oscillator_cancels_the_pending_falling_edge);
    return UNITY_END();
}
