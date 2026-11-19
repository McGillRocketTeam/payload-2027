#ifndef SIM_DEVICES_SIM_TMP102_H_
#define SIM_DEVICES_SIM_TMP102_H_

/*
 * Simulated TI TMP102 on I2C. Plugs into tmp102_ctx_t:
 *
 *   sim_tmp102_t tmp;
 *   sim_tmp102_init(&tmp, NULL);                       defaults: 0x48, 22 C
 *   tmp.cfg.temperature = sim_signal_ramp(20.0, 0.1);
 *   tmp102_ctx_t ctx = sim_tmp102_ctx(&tmp);
 *
 * Models (TMP102 datasheet, SBOS397):
 *   - registers: TEMP 0x00 (RO), CONFIG 0x01 (reset 0x60A0), TLOW 0x02
 *     (0x4B00), THIGH 0x03 (0x5000); 16-bit, MSB first; pointer register
 *   - 12-bit 0.0625 C/LSB, or 13-bit in extended mode (EM)
 *   - TEMP only updates once per conversion period (CR: 0.25/1/4/8 Hz)
 *   - shutdown mode (SD) freezes TEMP; one-shot (OS) does one conversion
 *   - wrong I2C address -> NACK
 * Not modelled yet: ALERT pin, thermostat/fault queue, SMBus alert.
 *
 * Faults (sim_device.h): OFF -> NACK (-1); BUS_ERROR -> timeout (-2);
 * CORRUPT -> a bit flipped in the reply; STUCK -> TEMP frozen.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensors/drivers/tmp102/tmp102.h"
#include "sim_device.h"
#include "sim_signal.h"

#define SIM_TMP102_REG_TEMP   0x00
#define SIM_TMP102_REG_CONFIG 0x01
#define SIM_TMP102_REG_TLOW   0x02
#define SIM_TMP102_REG_THIGH  0x03

#define SIM_I2C_NACK    (-1)
#define SIM_I2C_TIMEOUT (-2)

typedef struct {
    uint8_t addr;               /* 7-bit address (ADD0 wiring), default 0x48 */
    sim_signal_t temperature;   /* degrees C */
} sim_tmp102_config_t;

typedef struct sim_tmp102 sim_tmp102_t;

/*
 * Optional override, called before the model handles a transfer. Return true
 * if you handled it (buf filled for reads), false to fall through to the model.
 * Use it to make the chip answer anything you want for one test.
 */
typedef bool (*sim_tmp102_hook_t)(sim_tmp102_t *dev, bool is_read, uint8_t reg, uint8_t *buf,
                                  size_t len, int *ret);

struct sim_tmp102 {
    sim_device_t base;
    sim_tmp102_config_t cfg;
    sim_tmp102_hook_t hook;
    void *user;

    uint16_t regs[4];
    uint8_t pointer;
    sim_time_ns_t last_conversion_ns;
    bool converted_once;
    double frozen_c;
};

sim_tmp102_config_t sim_tmp102_default_config(void);
void sim_tmp102_init(sim_tmp102_t *dev, const sim_tmp102_config_t *cfg); /* cfg may be NULL */

/* A tmp102_ctx_t wired to this simulated chip. */
tmp102_ctx_t sim_tmp102_ctx(sim_tmp102_t *dev);

/* Raw bus functions, if you build the ctx yourself. handle = sim_tmp102_t*. */
int sim_tmp102_i2c_read(void *handle, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);
int sim_tmp102_i2c_write(void *handle, uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len);

/* Encode a temperature the way the chip would (for test expectations). */
uint16_t sim_tmp102_encode(double celsius, bool extended);

#endif /* SIM_DEVICES_SIM_TMP102_H_ */
