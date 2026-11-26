#ifndef SIM_DEVICES_SIM_LSM6DSR_H_
#define SIM_DEVICES_SIM_LSM6DSR_H_

/*
 * Simulated ST LSM6DSR IMU, per its data sheet (DS11976 Rev 2, docs/) and ST's
 * register driver. Plugs into stmdev_ctx_t, so lsm6dsr_reg.c and
 * modules/sensors/imu.c run against it unchanged:
 *
 *   sim_lsm6dsr_t imu;
 *   sim_lsm6dsr_init(&imu, NULL);
 *   imu.cfg.accel_g[2] = sim_signal_step(1.0, 6.0, 2.0);   launch at t = 2 s
 *   stmdev_ctx_t ctx = sim_lsm6dsr_ctx(&imu);
 *   INT1 / INT2 pins: imu.int1, imu.int2 (attach the MCU's EXTI)
 *
 * Modelled:
 *   - register file 00h-7Fh, reset values (WHO_AM_I 6Bh, CTRL3_C 04h),
 *     SW_RESET / BOOT, IF_INC auto-increment
 *   - output data rates 1.6-6667 Hz, full scales and sensitivities (Table 2),
 *     int16 clipping, STATUS_REG XLDA / GDA / TDA, temperature 25 C + LSB/256
 *   - sensor errors (Table 2): accel noise 60 ug/sqrt(Hz), zero-g offset
 *     (+-10 mg) and its drift (+-0.1 mg/C), gyro noise 5 mdps/sqrt(Hz),
 *     zero-rate level (+-1 dps), gyro sensitivity tolerance (+-1 %); noise
 *     bandwidth taken as ODR/2; per-unit values drawn by
 *     sim_lsm6dsr_typical_unit(), all 0 in the default config
 *   - FIFO (6.6): 512 words (3 kbytes of data), tag + 6 data bytes; accel /
 *     gyro batching (BDR), temperature batching (ODR_T_BATCH); Bypass, FIFO
 *     and Continuous modes; WTM[8:0], STOP_ON_WTM; DIFF_FIFO, WTM_IA, OVR_IA
 *     (completely filled), FULL_IA (full at the next sample), OVR_LATCHED
 *     (cleared by reading FIFO_STATUS2); TAG_CNT time slot; batch counter
 *     (CNT_BDR_TH, COUNTER_BDR_IA)
 *   - INT1 / INT2 (9.12, 9.13): data-ready, FIFO threshold / overrun / full,
 *     batch counter; INT2_on_INT1; H_LACTIVE polarity; data-ready latched
 *     until read, or pulsed 75 us (dataready_pulsed)
 * Not modelled: FIFO compression, timestamp and CFG-change words, the
 * trigger-based modes (Continuous-to-FIFO, Bypass-to-Continuous / -FIFO:
 * logged as unmodelled), filters, OIS, embedded functions, open-drain drive.
 * UNVERIFIED (not stated in the data sheet): a FIFO word is popped when its
 * last data byte (7Eh) is read; TAG_PARITY is the even parity of the tag's
 * upper 7 bits; words with the same time slot are queued gyro, accel,
 * temperature; a temperature word carries the reading in X; batching faster
 * than the sensor's ODR batches at the ODR.
 *
 * Faults (SPI has no ACK): OFF -> reads return the MISO idle level (cfg.miso),
 * so only a WHO_AM_I check can notice; BUS_ERROR -> -1; CORRUPT -> bit flip;
 * STUCK -> measurements frozen.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sensors/drivers/lsm6dsr/lsm6dsr_reg.h"
#include "sim_device.h"
#include "sim_pin.h"
#include "sim_signal.h"

#define SIM_LSM6DSR_FIFO_WORDS 512

typedef struct {
    sim_signal_t accel_g[3];    /* X, Y, Z specific force, g */
    sim_signal_t gyro_dps[3];   /* X, Y, Z, degrees per second */
    sim_signal_t temperature_c;
    uint8_t who_am_i;           /* change to test a wrong part */
    sim_miso_t miso;            /* what reads return while OFF */

    /* Sensor errors (data sheet Table 2). */
    double accel_noise_g_rthz;  /* 60e-6 typical */
    double accel_offset_g[3];   /* zero-g level, up to +-0.010 */
    double accel_offset_tc_g[3];/* per degC from 25 C, up to +-0.0001 */
    double gyro_noise_dps_rthz; /* 5e-3 typical */
    double gyro_offset_dps[3];  /* zero-rate level, up to +-1 */
    double gyro_gain_error;     /* up to +-0.01 */
    uint64_t seed;
} sim_lsm6dsr_config_t;

typedef struct sim_lsm6dsr sim_lsm6dsr_t;

/* Optional override, see sim_tmp102_hook_t. */
typedef bool (*sim_lsm6dsr_hook_t)(sim_lsm6dsr_t *dev, bool is_read, uint8_t reg, uint8_t *buf,
                                   uint16_t len, int32_t *ret);

typedef struct {
    uint8_t tag;
    uint8_t data[6];
} sim_lsm6dsr_word_t;

struct sim_lsm6dsr {
    sim_device_t base;
    sim_lsm6dsr_config_t cfg;
    sim_lsm6dsr_hook_t hook;
    void *user;
    sim_pin_t int1;
    sim_pin_t int2;

    uint8_t regs[0x80];
    sim_time_ns_t xl_start_ns, g_start_ns;   /* when the ODR was last set */
    uint64_t xl_sample, g_sample, t_sample;  /* index of the newest sample */
    uint64_t xl_read, g_read, t_read;        /* newest sample index read by the driver */

    /* FIFO. */
    sim_lsm6dsr_word_t fifo[SIM_LSM6DSR_FIFO_WORDS];
    int fifo_head, fifo_count;
    sim_time_ps_t fifo_start_ps;
    uint64_t fifo_k[3];          /* next batch index per stream: G, XL, T */
    sim_time_ps_t last_slot_ps;
    uint8_t tag_cnt;
    uint8_t stream_last[3][6]; /* last batched data per stream (STUCK fault) */
    bool stream_has_last[3];
    bool ovr_latched;
    uint32_t bdr_counter;
    bool counter_ia;
    uint32_t gen;
    sim_time_ps_t pulse_until_ps;

    /* Statistics. */
    uint32_t xl_samples_missed;  /* output-register samples never read (bypass mode) */
    uint32_t fifo_overruns;
};

sim_lsm6dsr_config_t sim_lsm6dsr_default_config(void);
/* One unit drawn from the data sheet's limits: noise at typical, offsets,
 * offset drift and gyro gain error uniform within their tolerances. */
void sim_lsm6dsr_typical_unit(sim_lsm6dsr_config_t *cfg, uint64_t seed);

void sim_lsm6dsr_init(sim_lsm6dsr_t *dev, const sim_lsm6dsr_config_t *cfg);
stmdev_ctx_t sim_lsm6dsr_ctx(sim_lsm6dsr_t *dev);

int32_t sim_lsm6dsr_read_reg(void *handle, uint8_t reg, uint8_t *buf, uint16_t len);
int32_t sim_lsm6dsr_write_reg(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len);

/* Output data rate in Hz for an ODR code (0 = off). */
double sim_lsm6dsr_odr_hz(uint8_t code);

#endif /* SIM_DEVICES_SIM_LSM6DSR_H_ */
