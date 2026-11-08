#include <unity.h>

#include "sensors/drivers/lsm6dsr/lsm6dsr_reg.h"

/*
 * ST's LSM6DSR driver (copied from flightComputer-2027) running against a fake
 * bus. The fake answers register reads from a small register file.
 */

static uint8_t regs[0x80];
static uint8_t last_read_reg;

static int32_t fake_write_reg(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    (void)handle;
    for (uint16_t i = 0; i < len; i++) {
        regs[(uint8_t)(reg + i) & 0x7F] = buf[i];
    }
    return 0;
}

static int32_t fake_read_reg(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    (void)handle;
    last_read_reg = reg;
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = regs[(uint8_t)(reg + i) & 0x7F];
    }
    return 0;
}

static stmdev_ctx_t ctx = {.write_reg = fake_write_reg, .read_reg = fake_read_reg, .handle = NULL};

void setUp(void)
{
    for (unsigned i = 0; i < sizeof(regs); i++) {
        regs[i] = 0;
    }
    last_read_reg = 0;
}

void tearDown(void) {}

void test_device_id_reads_who_am_i(void)
{
    uint8_t id = 0;
    regs[LSM6DSR_WHO_AM_I] = LSM6DSR_ID;

    TEST_ASSERT_EQUAL_INT32(0, lsm6dsr_device_id_get(&ctx, &id));

    TEST_ASSERT_EQUAL_HEX8(LSM6DSR_WHO_AM_I, last_read_reg);
    TEST_ASSERT_EQUAL_HEX8(LSM6DSR_ID, id);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_device_id_reads_who_am_i);
    return UNITY_END();
}
