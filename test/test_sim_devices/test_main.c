#include <unity.h>

#include "devices/sim_lsm6dsr.h"
#include "devices/sim_tmp102.h"
#include "sensors/drivers/lsm6dsr/lsm6dsr_reg.h"
#include "sim.h"

/*
 * Simulated chips driven through the same interfaces the real drivers use.
 * The LSM6DSR tests go through ST's real lsm6dsr_reg.c driver.
 */

static sim_tmp102_t tmp;
static sim_lsm6dsr_t imu;

void setUp(void)
{
    sim_reset();
    sim_tmp102_init(&tmp, NULL);
    sim_lsm6dsr_init(&imu, NULL);
}

void tearDown(void) {}

/* ---- TMP102 ---- */

static int16_t tmp_read(const tmp102_ctx_t *ctx, uint8_t reg, int *ret)
{
    uint8_t b[2] = {0};
    *ret = ctx->i2c_read(ctx->handle, ctx->addr, reg, b, 2);
    return (int16_t)((b[0] << 8) | b[1]);
}

void test_tmp102_reset_values(void)
{
    tmp102_ctx_t ctx = sim_tmp102_ctx(&tmp);
    int ret;
    TEST_ASSERT_EQUAL_HEX16(0x60A0, (uint16_t)tmp_read(&ctx, SIM_TMP102_REG_CONFIG, &ret));
    TEST_ASSERT_EQUAL_HEX16(0x4B00, (uint16_t)tmp_read(&ctx, SIM_TMP102_REG_TLOW, &ret));
    TEST_ASSERT_EQUAL_HEX16(0x5000, (uint16_t)tmp_read(&ctx, SIM_TMP102_REG_THIGH, &ret));
}

void test_tmp102_reports_temperature_after_first_conversion(void)
{
    tmp.cfg.temperature = sim_signal_const(25.0);
    tmp102_ctx_t ctx = sim_tmp102_ctx(&tmp);
    int ret;

    sim_advance_ms(30);
    int16_t raw = tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret);
    TEST_ASSERT_EQUAL_INT(0, ret);
    TEST_ASSERT_EQUAL_HEX16(0x1900, (uint16_t)raw); /* datasheet table: 25 C = 0x190 << 4 */
}

void test_tmp102_negative_temperature(void)
{
    tmp.cfg.temperature = sim_signal_const(-25.0);
    tmp102_ctx_t ctx = sim_tmp102_ctx(&tmp);
    int ret;
    sim_advance_ms(30);
    TEST_ASSERT_EQUAL_HEX16(0xE700, (uint16_t)tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret));
}

void test_tmp102_updates_only_at_conversion_rate(void)
{
    tmp.cfg.temperature = sim_signal_ramp(20.0, 10.0); /* +10 C/s */
    tmp102_ctx_t ctx = sim_tmp102_ctx(&tmp);
    int ret;

    sim_advance_ms(30);
    int16_t a = tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret);
    sim_advance_ms(100); /* default 4 Hz: no new conversion yet */
    int16_t b = tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret);
    sim_advance_ms(200);
    int16_t c = tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret);

    TEST_ASSERT_EQUAL_INT16(a, b);
    TEST_ASSERT_TRUE(c > b);
}

void test_tmp102_wrong_address_nacks(void)
{
    tmp102_ctx_t ctx = sim_tmp102_ctx(&tmp);
    ctx.addr = 0x49;
    int ret;
    tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret);
    TEST_ASSERT_EQUAL_INT(SIM_I2C_NACK, ret);
}

void test_tmp102_unplugged_then_back(void)
{
    tmp102_ctx_t ctx = sim_tmp102_ctx(&tmp);
    int ret;
    sim_inject_for(&tmp.base, SIM_FAULT_OFF, SIM_MS(500));
    tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret);
    TEST_ASSERT_EQUAL_INT(SIM_I2C_NACK, ret);
    sim_advance_ms(500);
    tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret);
    TEST_ASSERT_EQUAL_INT(0, ret);
}

void test_tmp102_stuck_freezes_reading(void)
{
    tmp.cfg.temperature = sim_signal_ramp(20.0, 10.0);
    tmp102_ctx_t ctx = sim_tmp102_ctx(&tmp);
    int ret;
    sim_advance_ms(30);
    int16_t a = tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret);
    sim_inject(&tmp.base, SIM_FAULT_STUCK);
    sim_advance_ms(2000);
    TEST_ASSERT_EQUAL_INT16(a, tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret));
}

void test_tmp102_shutdown_and_one_shot(void)
{
    tmp.cfg.temperature = sim_signal_ramp(20.0, 10.0);
    tmp102_ctx_t ctx = sim_tmp102_ctx(&tmp);
    int ret;
    sim_advance_ms(30);

    const uint8_t sd[2] = {0x61, 0xA0}; /* SD = 1 */
    ctx.i2c_write(ctx.handle, ctx.addr, SIM_TMP102_REG_CONFIG, sd, 2);
    int16_t a = tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret);
    sim_advance_ms(2000);
    TEST_ASSERT_EQUAL_INT16(a, tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret));

    const uint8_t os[2] = {0xE1, 0xA0}; /* OS = 1, SD = 1 */
    ctx.i2c_write(ctx.handle, ctx.addr, SIM_TMP102_REG_CONFIG, os, 2);
    TEST_ASSERT_TRUE(tmp_read(&ctx, SIM_TMP102_REG_TEMP, &ret) > a);
}

/* ---- LSM6DSR through ST's driver ---- */

void test_lsm6dsr_who_am_i(void)
{
    stmdev_ctx_t ctx = sim_lsm6dsr_ctx(&imu);
    uint8_t id = 0;
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_device_id_get(&ctx, &id));
    TEST_ASSERT_EQUAL_HEX8(LSM6DSR_ID, id);
}

void test_lsm6dsr_unplugged_reads_ff(void)
{
    stmdev_ctx_t ctx = sim_lsm6dsr_ctx(&imu);
    uint8_t id = 0;
    sim_inject(&imu.base, SIM_FAULT_OFF);
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_device_id_get(&ctx, &id)); /* SPI can't tell */
    TEST_ASSERT_EQUAL_HEX8(0xFF, id);
}

void test_lsm6dsr_software_reset(void)
{
    stmdev_ctx_t ctx = sim_lsm6dsr_ctx(&imu);
    uint8_t rst = 1;
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_xl_full_scale_set(&ctx, LSM6DSR_16g));
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_reset_set(&ctx, PROPERTY_ENABLE));
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_reset_get(&ctx, &rst));
    TEST_ASSERT_EQUAL_UINT8(0, rst);

    lsm6dsr_fs_xl_t fs;
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_xl_full_scale_get(&ctx, &fs));
    TEST_ASSERT_EQUAL_INT(LSM6DSR_2g, fs);
}

static void imu_start(stmdev_ctx_t *ctx, lsm6dsr_fs_xl_t fs)
{
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_xl_full_scale_set(ctx, fs));
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_xl_data_rate_set(ctx, LSM6DSR_XL_ODR_6667Hz));
}

void test_lsm6dsr_reads_acceleration_at_16g(void)
{
    imu.cfg.accel_g[0] = sim_signal_const(5.0);
    imu.cfg.accel_g[2] = sim_signal_const(-1.0);
    stmdev_ctx_t ctx = sim_lsm6dsr_ctx(&imu);
    imu_start(&ctx, LSM6DSR_16g);

    uint8_t ready = 0;
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_xl_flag_data_ready_get(&ctx, &ready));
    TEST_ASSERT_EQUAL_UINT8(1, ready);

    int16_t raw[3];
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_acceleration_raw_get(&ctx, raw));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5000.0f, lsm6dsr_from_fs16g_to_mg(raw[0]));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, lsm6dsr_from_fs16g_to_mg(raw[1]));
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -1000.0f, lsm6dsr_from_fs16g_to_mg(raw[2]));

    /* Read once: flag clears until the next sample (150 us at 6.67 kHz). */
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_xl_flag_data_ready_get(&ctx, &ready));
    TEST_ASSERT_EQUAL_UINT8(0, ready);
    sim_advance_ns(SIM_US(150));
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_xl_flag_data_ready_get(&ctx, &ready));
    TEST_ASSERT_EQUAL_UINT8(1, ready);
}

void test_lsm6dsr_clips_at_full_scale(void)
{
    imu.cfg.accel_g[0] = sim_signal_const(10.0); /* beyond +-2 g */
    stmdev_ctx_t ctx = sim_lsm6dsr_ctx(&imu);
    imu_start(&ctx, LSM6DSR_2g);

    int16_t raw[3];
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_acceleration_raw_get(&ctx, raw));
    TEST_ASSERT_EQUAL_INT16(32767, raw[0]);
}

void test_lsm6dsr_counts_missed_samples(void)
{
    stmdev_ctx_t ctx = sim_lsm6dsr_ctx(&imu);
    imu_start(&ctx, LSM6DSR_16g);
    int16_t raw[3];
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_acceleration_raw_get(&ctx, raw));

    sim_advance_ms(10); /* ~66 samples at 6.67 kHz, only the last can be read */
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_acceleration_raw_get(&ctx, raw));
    TEST_ASSERT_INT_WITHIN(1, 65, (int)imu.xl_samples_missed);
}

void test_lsm6dsr_bus_error(void)
{
    stmdev_ctx_t ctx = sim_lsm6dsr_ctx(&imu);
    uint8_t id;
    sim_inject_next(&imu.base, SIM_FAULT_BUS_ERROR, 1);
    TEST_ASSERT_NOT_EQUAL_INT32(0, lsm6dsr_device_id_get(&ctx, &id));
    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_device_id_get(&ctx, &id));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_tmp102_reset_values);
    RUN_TEST(test_tmp102_reports_temperature_after_first_conversion);
    RUN_TEST(test_tmp102_negative_temperature);
    RUN_TEST(test_tmp102_updates_only_at_conversion_rate);
    RUN_TEST(test_tmp102_wrong_address_nacks);
    RUN_TEST(test_tmp102_unplugged_then_back);
    RUN_TEST(test_tmp102_stuck_freezes_reading);
    RUN_TEST(test_tmp102_shutdown_and_one_shot);
    RUN_TEST(test_lsm6dsr_who_am_i);
    RUN_TEST(test_lsm6dsr_unplugged_reads_ff);
    RUN_TEST(test_lsm6dsr_software_reset);
    RUN_TEST(test_lsm6dsr_reads_acceleration_at_16g);
    RUN_TEST(test_lsm6dsr_clips_at_full_scale);
    RUN_TEST(test_lsm6dsr_counts_missed_samples);
    RUN_TEST(test_lsm6dsr_bus_error);
    return UNITY_END();
}
