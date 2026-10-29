#include <string.h>
#include <unity.h>

#include "sensors/drivers/tdc7200/tdc7200.h"

/*
 * Workshop tests. Remove the TEST_IGNORE_MESSAGE line once the function under
 * test is implemented.
 *
 * fake_spi records what the driver sends and replies with canned bytes, so the
 * SPI framing can be checked without a chip.
 */

static uint8_t last_tx[8];
static size_t last_len;
static uint8_t reply[8];

static int fake_spi_transfer(void *handle, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)handle;
    last_len = len;
    memcpy(last_tx, tx, len);
    memcpy(rx, reply, len);
    return 0;
}

static const tdc7200_ctx_t ctx = {.spi_transfer = fake_spi_transfer, .handle = NULL};

void setUp(void)
{
    memset(last_tx, 0, sizeof(last_tx));
    memset(reply, 0, sizeof(reply));
    last_len = 0;
}

void tearDown(void) {}

void test_read_reg_frames_command(void)
{
    TEST_IGNORE_MESSAGE("TODO(workshop): fill in expected bytes from the datasheet SPI format");
    (void)ctx;
}

void test_write_reg_frames_command(void)
{
    TEST_IGNORE_MESSAGE("TODO(workshop): fill in expected bytes from the datasheet SPI format");
}

void test_time_of_flight_matches_datasheet_example(void)
{
    TEST_IGNORE_MESSAGE("TODO(workshop): use the datasheet worked example values");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_read_reg_frames_command);
    RUN_TEST(test_write_reg_frames_command);
    RUN_TEST(test_time_of_flight_matches_datasheet_example);
    return UNITY_END();
}
