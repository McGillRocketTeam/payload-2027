#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "devices/sim_lsm6dsr.h"
#include "sensors/drivers/lsm6dsr/lsm6dsr_reg.h"
#include "sim.h"
#include "sim_log.h"

/*
 * LSM6DSR FIFO, interrupt pins and sensor errors, driven through ST's own
 * driver (lsm6dsr_reg.c), against the data sheet (DS11976).
 */

static sim_lsm6dsr_t imu;
static stmdev_ctx_t ctx;
static int int1_rises;
static sim_time_ps_t int1_last;

static void on_int1(void *c, bool rising, sim_time_ps_t t)
{
    (void)c;
    if (rising) {
        int1_rises++;
        int1_last = t;
    }
}

void setUp(void)
{
    sim_reset();
    char path[128];
    snprintf(path, sizeof(path), ".pio/sim/imu_%s.csv", Unity.CurrentTestName);
    sim_log_open(path);
    sim_lsm6dsr_init(&imu, NULL);
    ctx = sim_lsm6dsr_ctx(&imu);
    int1_rises = 0;
    sim_pin_listen(&imu.int1, on_int1, NULL);
}

void tearDown(void)
{
    sim_log_close();
}

static void start_fifo(lsm6dsr_fifo_mode_t mode)
{
    lsm6dsr_xl_full_scale_set(&ctx, LSM6DSR_16g);
    lsm6dsr_xl_data_rate_set(&ctx, LSM6DSR_XL_ODR_6667Hz);
    lsm6dsr_fifo_xl_batch_set(&ctx, LSM6DSR_XL_BATCHED_AT_6667Hz);
    lsm6dsr_fifo_mode_set(&ctx, mode);
}

static uint16_t level(void)
{
    uint16_t n = 0;
    lsm6dsr_fifo_data_level_get(&ctx, &n);
    return n;
}

void test_fifo_collects_accel_words_at_the_batch_rate(void)
{
    start_fifo(LSM6DSR_STREAM_MODE);
    sim_advance_ms(15); /* 6667 Hz x 15 ms = 100 words */
    TEST_ASSERT_INT_WITHIN(1, 100, level());

    lsm6dsr_fifo_tag_t tag;
    uint8_t raw[6];
    lsm6dsr_fifo_sensor_tag_get(&ctx, &tag);
    lsm6dsr_fifo_out_raw_get(&ctx, raw);
    TEST_ASSERT_EQUAL_INT(LSM6DSR_XL_NC_TAG, tag);
    int16_t z = (int16_t)(raw[4] | raw[5] << 8);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, lsm6dsr_from_fs16g_to_mg(z));
    TEST_ASSERT_INT_WITHIN(1, 99, level()); /* one word popped */
}

void test_fifo_mode_stops_at_512_words(void)
{
    start_fifo(LSM6DSR_FIFO_MODE);
    sim_advance_ms(200); /* 1333 samples offered */
    TEST_ASSERT_EQUAL_UINT16(512, level()); /* 3 kbytes = 512 words */
    uint8_t ovr = 0;
    lsm6dsr_fifo_ovr_flag_get(&ctx, &ovr);
    TEST_ASSERT_EQUAL_UINT8(1, ovr);
    TEST_ASSERT_EQUAL_UINT32(0, imu.fifo_overruns); /* FIFO mode does not overwrite */
}

void test_continuous_mode_overwrites_and_latches_overrun(void)
{
    start_fifo(LSM6DSR_STREAM_MODE);
    sim_advance_ms(200);
    /* Check the latched flag first: lsm6dsr_fifo_data_level_get() reads
     * FIFO_STATUS2 too, and that read clears it (data sheet 9.42). */
    uint8_t st2 = 0;
    lsm6dsr_read_reg(&ctx, LSM6DSR_FIFO_STATUS2, &st2, 1);
    TEST_ASSERT_EQUAL_HEX8(0x08, st2 & 0x08); /* FIFO_OVR_LATCHED */
    TEST_ASSERT_TRUE(imu.fifo_overruns > 0); /* counted as the chip is accessed */
    lsm6dsr_read_reg(&ctx, LSM6DSR_FIFO_STATUS2, &st2, 1);
    TEST_ASSERT_EQUAL_HEX8(0x00, st2 & 0x08); /* reset by the read */
    TEST_ASSERT_EQUAL_UINT16(512, level());
    TEST_ASSERT_EQUAL_UINT32(1, sim_log_count("lsm6dsr", "error"));
}

void test_bypass_resets_the_fifo(void)
{
    start_fifo(LSM6DSR_STREAM_MODE);
    sim_advance_ms(5);
    TEST_ASSERT_TRUE(level() > 0);
    lsm6dsr_fifo_mode_set(&ctx, LSM6DSR_BYPASS_MODE);
    TEST_ASSERT_EQUAL_UINT16(0, level());
}

void test_watermark_interrupt_on_int1_at_the_right_time(void)
{
    lsm6dsr_fifo_watermark_set(&ctx, 200);
    uint8_t int1 = 0x08; /* INT1_FIFO_TH */
    lsm6dsr_write_reg(&ctx, LSM6DSR_INT1_CTRL, &int1, 1);
    start_fifo(LSM6DSR_STREAM_MODE);
    sim_time_ps_t t0 = sim_now_ps();
    sim_advance_ms(50);
    TEST_ASSERT_EQUAL_INT(1, int1_rises);
    TEST_ASSERT_TRUE(imu.int1.level);
    double t = (double)(int1_last - t0) / 1e12;
    TEST_ASSERT_DOUBLE_WITHIN(1.0 / 6667.0, 200.0 / 6667.0, t); /* 200 words in */

    /* Draining below the watermark releases the line. */
    uint8_t w[7];
    for (int i = 0; i < 150; i++) {
        lsm6dsr_read_reg(&ctx, LSM6DSR_FIFO_DATA_OUT_TAG, w, 7);
    }
    TEST_ASSERT_FALSE(imu.int1.level);
}

void test_stop_on_wtm_limits_depth(void)
{
    lsm6dsr_fifo_watermark_set(&ctx, 64);
    lsm6dsr_fifo_stop_on_wtm_set(&ctx, 1);
    start_fifo(LSM6DSR_FIFO_MODE);
    sim_advance_ms(100);
    TEST_ASSERT_EQUAL_UINT16(64, level());
}

void test_gyro_and_temperature_words_are_tagged(void)
{
    lsm6dsr_gy_data_rate_set(&ctx, LSM6DSR_GY_ODR_833Hz);
    lsm6dsr_fifo_gy_batch_set(&ctx, LSM6DSR_GY_BATCHED_AT_833Hz);
    lsm6dsr_fifo_temp_batch_set(&ctx, LSM6DSR_TEMP_BATCHED_AT_52Hz);
    start_fifo(LSM6DSR_STREAM_MODE);
    sim_advance_ms(30);
    int xl = 0, gy = 0, temp = 0;
    uint16_t n = level();
    for (uint16_t i = 0; i < n; i++) {
        lsm6dsr_fifo_tag_t tag;
        uint8_t raw[6];
        lsm6dsr_fifo_sensor_tag_get(&ctx, &tag);
        lsm6dsr_fifo_out_raw_get(&ctx, raw);
        xl += tag == LSM6DSR_XL_NC_TAG;
        gy += tag == LSM6DSR_GYRO_NC_TAG;
        temp += tag == LSM6DSR_TEMPERATURE_TAG;
    }
    TEST_ASSERT_INT_WITHIN(2, 200, xl);  /* 6667 Hz x 30 ms */
    TEST_ASSERT_INT_WITHIN(1, 25, gy);   /* 833 Hz */
    TEST_ASSERT_INT_WITHIN(1, 1, temp);  /* 52 Hz */
}

void test_data_ready_latched_and_active_low(void)
{
    lsm6dsr_pin_polarity_set(&ctx, LSM6DSR_ACTIVE_LOW);
    uint8_t int1 = 0x01; /* INT1_DRDY_XL */
    lsm6dsr_write_reg(&ctx, LSM6DSR_INT1_CTRL, &int1, 1);
    TEST_ASSERT_TRUE(imu.int1.level); /* idle high when active low */
    lsm6dsr_xl_data_rate_set(&ctx, LSM6DSR_XL_ODR_104Hz);
    sim_advance_ms(20);
    TEST_ASSERT_FALSE(imu.int1.level); /* data ready: driven low, latched */
    int16_t raw[3];
    lsm6dsr_acceleration_raw_get(&ctx, raw);
    TEST_ASSERT_TRUE(imu.int1.level); /* released by the read */
}

void test_data_ready_pulsed_75us(void)
{
    lsm6dsr_data_ready_mode_set(&ctx, LSM6DSR_DRDY_PULSED);
    uint8_t int1 = 0x01;
    lsm6dsr_write_reg(&ctx, LSM6DSR_INT1_CTRL, &int1, 1);
    lsm6dsr_xl_data_rate_set(&ctx, LSM6DSR_XL_ODR_104Hz);
    sim_advance_ms(100); /* ~10 samples, nobody reads */
    TEST_ASSERT_INT_WITHIN(1, 10, int1_rises);
}

void test_typical_unit_noise_matches_density(void)
{
    sim_lsm6dsr_config_t c = sim_lsm6dsr_default_config();
    sim_lsm6dsr_typical_unit(&c, 7);
    sim_lsm6dsr_init(&imu, &c);
    ctx = sim_lsm6dsr_ctx(&imu);
    lsm6dsr_xl_full_scale_set(&ctx, LSM6DSR_2g);
    lsm6dsr_xl_data_rate_set(&ctx, LSM6DSR_XL_ODR_6667Hz);
    double sum = 0.0, sum2 = 0.0;
    const int n = 2000;
    for (int i = 0; i < n; i++) {
        sim_advance_ns(150000); /* one sample */
        int16_t raw[3];
        lsm6dsr_acceleration_raw_get(&ctx, raw);
        double g = lsm6dsr_from_fs2g_to_mg(raw[0]) / 1000.0;
        sum += g;
        sum2 += g * g;
    }
    double mean = sum / n;
    double std = sqrt(sum2 / n - mean * mean);
    TEST_ASSERT_DOUBLE_WITHIN(0.0005, 60e-6 * sqrt(6667.0 / 2.0), std); /* ~3.5 mg */
    TEST_ASSERT_DOUBLE_WITHIN(0.0002, c.accel_offset_g[0], mean);
    TEST_ASSERT_TRUE(fabs(c.accel_offset_g[0]) <= 0.010);
}

void test_burst_read_of_two_words_rolls_back_to_the_tag(void)
{
    start_fifo(LSM6DSR_STREAM_MODE);
    sim_advance_ms(15);
    uint16_t before = level();
    uint8_t b[14];
    TEST_ASSERT_EQUAL_INT(0, lsm6dsr_read_reg(&ctx, LSM6DSR_FIFO_DATA_OUT_TAG, b, sizeof(b)));
    for (int w = 0; w < 2; w++) {
        const uint8_t *p = b + 7 * w;
        TEST_ASSERT_EQUAL_INT(LSM6DSR_XL_NC_TAG, p[0] >> 3);
        int16_t z = (int16_t)(p[5] | p[6] << 8);
        TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, lsm6dsr_from_fs16g_to_mg(z));
    }
    TEST_ASSERT_UINT16_WITHIN(1, before - 2, level()); /* both words popped */
}

void test_stuck_sensor_repeats_its_own_last_value(void)
{
    lsm6dsr_gy_data_rate_set(&ctx, LSM6DSR_GY_ODR_833Hz);
    lsm6dsr_fifo_gy_batch_set(&ctx, LSM6DSR_GY_BATCHED_AT_833Hz);
    lsm6dsr_xl_full_scale_set(&ctx, LSM6DSR_16g);
    lsm6dsr_xl_data_rate_set(&ctx, LSM6DSR_XL_ODR_833Hz);
    lsm6dsr_fifo_xl_batch_set(&ctx, LSM6DSR_XL_BATCHED_AT_833Hz);
    sim_inject(&imu.base, SIM_FAULT_STUCK);
    lsm6dsr_fifo_mode_set(&ctx, LSM6DSR_STREAM_MODE);
    sim_advance_ms(30);
    int xl = 0, gy = 0;
    uint16_t n = level();
    for (uint16_t i = 0; i < n; i++) {
        lsm6dsr_fifo_tag_t tag;
        uint8_t raw[6];
        lsm6dsr_fifo_sensor_tag_get(&ctx, &tag);
        lsm6dsr_fifo_out_raw_get(&ctx, raw);
        int16_t z = (int16_t)(raw[4] | raw[5] << 8);
        if (tag == LSM6DSR_GYRO_NC_TAG) {
            gy++;
            TEST_ASSERT_INT_WITHIN(5, 0, z); /* gyro at rest, not 1 g of accel counts */
        } else if (tag == LSM6DSR_XL_NC_TAG) {
            xl++;
            TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, lsm6dsr_from_fs16g_to_mg(z));
        }
    }
    TEST_ASSERT_TRUE(xl > 10);
    TEST_ASSERT_TRUE(gy > 10);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_fifo_collects_accel_words_at_the_batch_rate);
    RUN_TEST(test_fifo_mode_stops_at_512_words);
    RUN_TEST(test_continuous_mode_overwrites_and_latches_overrun);
    RUN_TEST(test_bypass_resets_the_fifo);
    RUN_TEST(test_watermark_interrupt_on_int1_at_the_right_time);
    RUN_TEST(test_stop_on_wtm_limits_depth);
    RUN_TEST(test_gyro_and_temperature_words_are_tagged);
    RUN_TEST(test_data_ready_latched_and_active_low);
    RUN_TEST(test_data_ready_pulsed_75us);
    RUN_TEST(test_typical_unit_noise_matches_density);
    RUN_TEST(test_burst_read_of_two_words_rolls_back_to_the_tag);
    RUN_TEST(test_stuck_sensor_repeats_its_own_last_value);
    return UNITY_END();
}
