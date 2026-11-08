#include <unity.h>

#include "sensors/csac.h"
#include "sensors/gps.h"
#include "sensors/imu.h"
#include "sensors/mac.h"
#include "sensors/tdc.h"
#include "sensors/temperature.h"

/* Counts calls into fake hardware, so tests can check create() never touches it. */
static int hw_calls;

static int fake_spi_transfer(void *handle, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)handle;
    (void)tx;
    (void)rx;
    (void)len;
    hw_calls++;
    return 0;
}

static void fake_set_pin(void *handle, bool high)
{
    (void)handle;
    (void)high;
    hw_calls++;
}

static void fake_delay_ms(void *handle, uint32_t ms)
{
    (void)handle;
    (void)ms;
    hw_calls++;
}

static size_t fake_uart_read(void *handle, uint8_t *buf, size_t max)
{
    (void)handle;
    (void)buf;
    (void)max;
    hw_calls++;
    return 0;
}

static int fake_uart_write(void *handle, const uint8_t *buf, size_t len)
{
    (void)handle;
    (void)buf;
    (void)len;
    hw_calls++;
    return 0;
}

static int32_t fake_reg_write(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    (void)handle;
    (void)reg;
    (void)buf;
    (void)len;
    hw_calls++;
    return 0;
}

static int32_t fake_reg_read(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)handle;
    (void)reg;
    (void)buf;
    (void)len;
    hw_calls++;
    return 0;
}

static int fake_i2c_read(void *handle, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    (void)handle;
    (void)addr;
    (void)reg;
    (void)buf;
    (void)len;
    hw_calls++;
    return 0;
}

static int fake_i2c_write(void *handle, uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len)
{
    (void)handle;
    (void)addr;
    (void)reg;
    (void)buf;
    (void)len;
    hw_calls++;
    return 0;
}

void setUp(void)
{
    hw_calls = 0;
}

void tearDown(void) {}

void test_tdc_create_does_not_touch_hardware(void)
{
    tdc_io_t io = {
        .bus = {.spi_transfer = fake_spi_transfer, .handle = NULL},
        .set_enable = fake_set_pin,
        .delay_ms = fake_delay_ms,
        .handle = NULL,
    };
    tdc_t tdc;

    tdc_create(&tdc, &io);

    TEST_ASSERT_EQUAL_INT(0, hw_calls);
    TEST_ASSERT_FALSE(tdc_is_initialized(&tdc));
    TEST_ASSERT_EQUAL_INT(SENSOR_NOT_PRESENT, tdc_status(&tdc));
}

void test_gps_create_does_not_touch_hardware(void)
{
    gps_io_t io = {
        .uart_read = fake_uart_read,
        .uart_write = fake_uart_write,
        .delay_ms = fake_delay_ms,
        .handle = NULL,
    };
    gps_t gps;

    gps_create(&gps, &io);

    TEST_ASSERT_EQUAL_INT(0, hw_calls);
    TEST_ASSERT_FALSE(gps_is_initialized(&gps));
    TEST_ASSERT_EQUAL_INT(SENSOR_NOT_PRESENT, gps_status(&gps));
}

void test_imu_create_does_not_touch_hardware(void)
{
    stmdev_ctx_t bus = {
        .write_reg = fake_reg_write,
        .read_reg = fake_reg_read,
        .handle = NULL,
    };
    imu_t imu;

    imu_create(&imu, &bus);

    TEST_ASSERT_EQUAL_INT(0, hw_calls);
    TEST_ASSERT_FALSE(imu_is_initialized(&imu));
    TEST_ASSERT_EQUAL_INT(SENSOR_NOT_PRESENT, imu_status(&imu));
}

void test_temperature_create_does_not_touch_hardware(void)
{
    tmp102_ctx_t bus = {
        .i2c_read = fake_i2c_read,
        .i2c_write = fake_i2c_write,
        .addr = 0,
        .handle = NULL,
    };
    temperature_t temperature;

    temperature_create(&temperature, &bus);

    TEST_ASSERT_EQUAL_INT(0, hw_calls);
    TEST_ASSERT_FALSE(temperature_is_initialized(&temperature));
    TEST_ASSERT_EQUAL_INT(SENSOR_NOT_PRESENT, temperature_status(&temperature));
}

void test_csac_create_does_not_touch_hardware(void)
{
    csac_io_t io = {
        .uart_read = fake_uart_read,
        .uart_write = fake_uart_write,
        .handle = NULL,
    };
    csac_t csac;

    csac_create(&csac, &io);

    TEST_ASSERT_EQUAL_INT(0, hw_calls);
    TEST_ASSERT_FALSE(csac_is_initialized(&csac));
    TEST_ASSERT_EQUAL_INT(SENSOR_NOT_PRESENT, csac_status(&csac));
}

void test_mac_create_does_not_touch_hardware(void)
{
    mac_io_t io = {
        .uart_read = fake_uart_read,
        .uart_write = fake_uart_write,
        .handle = NULL,
    };
    mac_t mac;

    mac_create(&mac, &io);

    TEST_ASSERT_EQUAL_INT(0, hw_calls);
    TEST_ASSERT_FALSE(mac_is_initialized(&mac));
    TEST_ASSERT_EQUAL_INT(SENSOR_NOT_PRESENT, mac_status(&mac));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_tdc_create_does_not_touch_hardware);
    RUN_TEST(test_gps_create_does_not_touch_hardware);
    RUN_TEST(test_imu_create_does_not_touch_hardware);
    RUN_TEST(test_temperature_create_does_not_touch_hardware);
    RUN_TEST(test_csac_create_does_not_touch_hardware);
    RUN_TEST(test_mac_create_does_not_touch_hardware);
    return UNITY_END();
}
