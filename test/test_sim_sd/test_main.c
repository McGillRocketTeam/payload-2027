#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "ff.h"
#include "sim.h"
#include "sim_sd.h"

/*
 * The simulated SD card through the FatFs API, the same calls
 * modules/sd_card will make on the real card.
 */

#define ROOT ".pio/sim/test_sim_sd"

static FATFS fs;
static FIL fil;

static long host_size(const char *card_path)
{
    FILE *f = fopen(sim_sd_host_path(card_path), "rb");
    if (!f) {
        return -1;
    }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fclose(f);
    return n;
}

static void write_bytes(FIL *fp, size_t n, uint8_t value)
{
    uint8_t buf[256];
    memset(buf, value, sizeof(buf));
    while (n > 0) {
        UINT chunk = n > sizeof(buf) ? sizeof(buf) : (UINT)n;
        UINT bw;
        TEST_ASSERT_EQUAL_INT(FR_OK, f_write(fp, buf, chunk, &bw));
        TEST_ASSERT_EQUAL_UINT(chunk, bw);
        n -= chunk;
    }
}

void setUp(void)
{
    sim_reset();
    sim_sd_config_t cfg = sim_sd_default_config(ROOT);
    TEST_ASSERT_TRUE(sim_sd_init(&cfg));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_mount(&fs, "", 1));
}

void tearDown(void) {}

void test_file_lands_in_host_folder(void)
{
    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "0:/log0.bin", FA_WRITE | FA_CREATE_ALWAYS));
    UINT bw;
    TEST_ASSERT_EQUAL_INT(FR_OK, f_write(&fil, "PPS!", 4, &bw));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));

    TEST_ASSERT_EQUAL_INT32(4, host_size("log0.bin"));

    char back[4];
    UINT br;
    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "log0.bin", FA_READ));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_read(&fil, back, sizeof(back), &br));
    TEST_ASSERT_EQUAL_UINT(4, br);
    TEST_ASSERT_EQUAL_MEMORY("PPS!", back, 4);
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));
}

void test_open_modes(void)
{
    TEST_ASSERT_EQUAL_INT(FR_NO_FILE, f_open(&fil, "missing.bin", FA_READ));
    TEST_ASSERT_EQUAL_INT(FR_NO_PATH, f_open(&fil, "nodir/a.bin", FA_WRITE | FA_CREATE_ALWAYS));

    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "a.bin", FA_WRITE | FA_CREATE_NEW));
    write_bytes(&fil, 10, 0xAA);
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));
    TEST_ASSERT_EQUAL_INT(FR_EXIST, f_open(&fil, "a.bin", FA_WRITE | FA_CREATE_NEW));

    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "a.bin", FA_WRITE | FA_OPEN_APPEND));
    TEST_ASSERT_EQUAL_UINT32(10, f_tell(&fil));
    write_bytes(&fil, 5, 0xBB);
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));
    TEST_ASSERT_EQUAL_INT32(15, host_size("a.bin"));
}

void test_power_cut_keeps_only_synced_data(void)
{
    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "flight.bin", FA_WRITE | FA_CREATE_ALWAYS));
    write_bytes(&fil, 1000, 1);
    TEST_ASSERT_EQUAL_INT(FR_OK, f_sync(&fil));
    write_bytes(&fil, 500, 2); /* never synced */

    sim_sd_power_cut();

    TEST_ASSERT_EQUAL_INT32(1000, host_size("flight.bin"));
    UINT bw;
    TEST_ASSERT_EQUAL_INT(FR_INVALID_OBJECT, f_write(&fil, "x", 1, &bw));
    TEST_ASSERT_EQUAL_INT(FR_NOT_READY, f_open(&fil, "flight.bin", FA_READ));

    /* Reboot: card comes back, firmware mounts and opens a new file. */
    sim_sd_insert();
    TEST_ASSERT_EQUAL_INT(FR_OK, f_mount(&fs, "", 1));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "flight1.bin", FA_WRITE | FA_CREATE_NEW));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));
}

void test_injected_write_error(void)
{
    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "e.bin", FA_WRITE | FA_CREATE_ALWAYS));
    sim_inject_next(&sim_sd.base, SIM_FAULT_BUS_ERROR, 1);
    UINT bw;
    TEST_ASSERT_EQUAL_INT(FR_DISK_ERR, f_write(&fil, "abc", 3, &bw));
    TEST_ASSERT_EQUAL_UINT(0, bw);
    TEST_ASSERT_EQUAL_INT(FR_OK, f_write(&fil, "abc", 3, &bw)); /* retry works */
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));
}

void test_card_missing(void)
{
    sim_inject(&sim_sd.base, SIM_FAULT_OFF);
    TEST_ASSERT_EQUAL_INT(FR_NOT_READY, f_mount(&fs, "", 1));
    TEST_ASSERT_EQUAL_INT(FR_NOT_READY, f_open(&fil, "a.bin", FA_WRITE | FA_CREATE_ALWAYS));
}

void test_write_stall_costs_time(void)
{
    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "s.bin", FA_WRITE | FA_CREATE_ALWAYS));
    sim_inject_next(&sim_sd.base, SIM_FAULT_STALL, 1)->stall_ns = SIM_MS(250);
    sim_time_ns_t before = sim_now_ns();
    write_bytes(&fil, 16, 0);
    TEST_ASSERT_EQUAL_UINT64(SIM_MS(250), sim_now_ns() - before);
    TEST_ASSERT_EQUAL_UINT64(SIM_MS(250), sim_sd.longest_op_ns);
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));
}

void test_corrupt_write_flips_one_bit(void)
{
    uint8_t data[64] = {0};
    uint8_t back[64];
    UINT bw;
    UINT br;
    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "c.bin", FA_WRITE | FA_READ | FA_CREATE_ALWAYS));
    sim_inject_next(&sim_sd.base, SIM_FAULT_CORRUPT, 1);
    TEST_ASSERT_EQUAL_INT(FR_OK, f_write(&fil, data, sizeof(data), &bw));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_lseek(&fil, 0));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_read(&fil, back, sizeof(back), &br));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));

    int bits = 0;
    for (size_t i = 0; i < sizeof(back); i++) {
        bits += __builtin_popcount(back[i]);
    }
    TEST_ASSERT_EQUAL_INT(1, bits);
}

void test_full_card_gives_short_write(void)
{
    sim_sd_config_t cfg = sim_sd_default_config(ROOT);
    cfg.capacity_bytes = 100;
    TEST_ASSERT_TRUE(sim_sd_init(&cfg));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_mount(&fs, "", 1));

    uint8_t buf[150] = {0};
    UINT bw;
    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "full.bin", FA_WRITE | FA_CREATE_ALWAYS));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_write(&fil, buf, sizeof(buf), &bw));
    TEST_ASSERT_EQUAL_UINT(100, bw);
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));
}

void test_write_throughput_costs_time(void)
{
    sim_sd_config_t cfg = sim_sd_default_config(ROOT);
    cfg.write_bytes_per_s = 1e6; /* 1 MB/s */
    TEST_ASSERT_TRUE(sim_sd_init(&cfg));
    TEST_ASSERT_EQUAL_INT(FR_OK, f_mount(&fs, "", 1));

    TEST_ASSERT_EQUAL_INT(FR_OK, f_open(&fil, "t.bin", FA_WRITE | FA_CREATE_ALWAYS));
    write_bytes(&fil, 50000, 0); /* 50 KB: one second of LOG_ACCEL data */
    TEST_ASSERT_UINT64_WITHIN(SIM_US(1), SIM_MS(50), sim_now_ns());
    TEST_ASSERT_EQUAL_INT(FR_OK, f_close(&fil));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_file_lands_in_host_folder);
    RUN_TEST(test_open_modes);
    RUN_TEST(test_power_cut_keeps_only_synced_data);
    RUN_TEST(test_injected_write_error);
    RUN_TEST(test_card_missing);
    RUN_TEST(test_write_stall_costs_time);
    RUN_TEST(test_corrupt_write_flips_one_bit);
    RUN_TEST(test_full_card_gives_short_write);
    RUN_TEST(test_write_throughput_costs_time);
    return UNITY_END();
}
