#include <unity.h>

#include "sensors/drivers/ubx/ubx.h"

/*
 * Workshop tests. Remove the TEST_IGNORE_MESSAGE line once the function under
 * test is implemented.
 */

void setUp(void) {}

void tearDown(void) {}

void test_checksum_mon_ver_poll(void)
{
    TEST_IGNORE_MESSAGE("TODO(workshop): implement ubx_checksum");

    /* UBX-MON-VER poll: B5 62 0A 04 00 00 0E 34. Checksum covers class..payload. */
    const uint8_t covered[] = {0x0A, 0x04, 0x00, 0x00};
    uint8_t ck_a;
    uint8_t ck_b;

    ubx_checksum(covered, sizeof(covered), &ck_a, &ck_b);

    TEST_ASSERT_EQUAL_HEX8(0x0E, ck_a);
    TEST_ASSERT_EQUAL_HEX8(0x34, ck_b);
}

void test_parser_rejects_bad_checksum(void)
{
    TEST_IGNORE_MESSAGE("TODO(workshop): write once ubx_parser_feed exists");
}

void test_decode_nav_pvt(void)
{
    TEST_IGNORE_MESSAGE("TODO(workshop): needs a captured NAV-PVT frame");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_checksum_mon_ver_poll);
    RUN_TEST(test_parser_rejects_bad_checksum);
    RUN_TEST(test_decode_nav_pvt);
    return UNITY_END();
}
