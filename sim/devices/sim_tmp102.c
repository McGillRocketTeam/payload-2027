#include "sim_tmp102.h"

#include <math.h>
#include <string.h>

/* CONFIG bits (16-bit view: byte 1 is the high byte). */
#define CFG_OS (1u << 15)
#define CFG_SD (1u << 8)
#define CFG_CR_SHIFT 6
#define CFG_CR_MASK  (3u << CFG_CR_SHIFT)
#define CFG_AL (1u << 5)
#define CFG_EM (1u << 4)
#define CFG_RO_MASK  (0x6000u | CFG_AL) /* R1 R0 and AL are read-only */

#define CONFIG_RESET 0x60A0u
#define TLOW_RESET   0x4B00u
#define THIGH_RESET  0x5000u

static const sim_time_ns_t period_ns[4] = {
    SIM_MS(4000), /* 0.25 Hz */
    SIM_MS(1000), /* 1 Hz */
    SIM_MS(250),  /* 4 Hz (default) */
    SIM_MS(125),  /* 8 Hz */
};

uint16_t sim_tmp102_encode(double c, bool extended)
{
    double lo = -55.0;
    double hi = extended ? 150.0 : 127.9375;
    if (c < lo) {
        c = lo;
    }
    if (c > hi) {
        c = hi;
    }
    int32_t counts = (int32_t)lround(c / 0.0625);
    if (extended) {
        return (uint16_t)(((uint32_t)counts << 3) | 0x0001u);
    }
    return (uint16_t)((uint32_t)counts << 4);
}

static void reset_regs(sim_tmp102_t *dev)
{
    dev->regs[SIM_TMP102_REG_TEMP] = 0;
    dev->regs[SIM_TMP102_REG_CONFIG] = CONFIG_RESET;
    dev->regs[SIM_TMP102_REG_TLOW] = TLOW_RESET;
    dev->regs[SIM_TMP102_REG_THIGH] = THIGH_RESET;
    dev->pointer = 0;
    dev->converted_once = false;
    dev->last_conversion_ns = sim_now_ns();
}

static void on_power_on(sim_device_t *base)
{
    reset_regs((sim_tmp102_t *)base);
}

sim_tmp102_config_t sim_tmp102_default_config(void)
{
    sim_tmp102_config_t c = {
        .addr = 0x48,
        .temperature = sim_signal_const(22.0),
    };
    return c;
}

void sim_tmp102_init(sim_tmp102_t *dev, const sim_tmp102_config_t *cfg)
{
    memset(dev, 0, sizeof(*dev));
    sim_device_init(&dev->base, "tmp102");
    dev->base.on_power_on = on_power_on;
    dev->cfg = cfg ? *cfg : sim_tmp102_default_config();
    reset_regs(dev);
}

tmp102_ctx_t sim_tmp102_ctx(sim_tmp102_t *dev)
{
    tmp102_ctx_t ctx = {
        .i2c_read = sim_tmp102_i2c_read,
        .i2c_write = sim_tmp102_i2c_write,
        .addr = dev->cfg.addr,
        .handle = dev,
    };
    return ctx;
}

static void convert(sim_tmp102_t *dev, sim_time_ns_t at)
{
    uint16_t cfg = dev->regs[SIM_TMP102_REG_CONFIG];
    double c;
    if (sim_device_is_stuck(&dev->base) && dev->converted_once) {
        c = dev->frozen_c;
    } else {
        c = sim_signal_eval(&dev->cfg.temperature, at);
        dev->frozen_c = c;
    }
    dev->regs[SIM_TMP102_REG_TEMP] = sim_tmp102_encode(c, (cfg & CFG_EM) != 0);
    dev->last_conversion_ns = at;
    dev->converted_once = true;
}

/* Bring TEMP up to date with the conversions that happened since last access. */
static void update(sim_tmp102_t *dev)
{
    uint16_t cfg = dev->regs[SIM_TMP102_REG_CONFIG];
    sim_time_ns_t now = sim_now_ns();

    if (cfg & CFG_SD) {
        return; /* shut down: TEMP holds its last value */
    }
    sim_time_ns_t period = period_ns[(cfg & CFG_CR_MASK) >> CFG_CR_SHIFT];
    if (!dev->converted_once) {
        /* First conversion completes ~26 ms after power-up. */
        if (now - dev->last_conversion_ns >= SIM_MS(26)) {
            convert(dev, dev->last_conversion_ns + SIM_MS(26));
        }
        return;
    }
    if (now - dev->last_conversion_ns >= period) {
        sim_time_ns_t n = (now - dev->last_conversion_ns) / period;
        convert(dev, dev->last_conversion_ns + n * period);
    }
}

static int begin(sim_tmp102_t *dev, uint8_t addr, bool *corrupt)
{
    *corrupt = false;
    sim_fault_mode_t f = sim_device_begin(&dev->base);
    if (f == SIM_FAULT_OFF) {
        return SIM_I2C_NACK;
    }
    if (f == SIM_FAULT_BUS_ERROR) {
        return SIM_I2C_TIMEOUT;
    }
    if (addr != dev->cfg.addr) {
        return SIM_I2C_NACK;
    }
    *corrupt = (f == SIM_FAULT_CORRUPT);
    return 0;
}

int sim_tmp102_i2c_read(void *handle, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len)
{
    sim_tmp102_t *dev = handle;
    bool corrupt;
    int ret = begin(dev, addr, &corrupt);
    if (ret != 0) {
        sim_device_trace(&dev->base, "read  reg 0x%02X -> NACK/err %d", reg, ret);
        return ret;
    }
    if (dev->hook && dev->hook(dev, true, reg, buf, len, &ret)) {
        return ret;
    }
    if (reg > 3) {
        return SIM_I2C_NACK;
    }

    update(dev);
    dev->pointer = reg;
    uint16_t v = dev->regs[reg];
    /* 16-bit register, MSB first; further bytes repeat it (chip behaviour). */
    for (size_t i = 0; i < len; i++) {
        buf[i] = (i % 2 == 0) ? (uint8_t)(v >> 8) : (uint8_t)v;
    }
    if (corrupt) {
        sim_corrupt(buf, len);
    }
    sim_device_trace(&dev->base, "read  reg 0x%02X -> 0x%04X", reg, v);
    return 0;
}

int sim_tmp102_i2c_write(void *handle, uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len)
{
    sim_tmp102_t *dev = handle;
    bool corrupt;
    int ret = begin(dev, addr, &corrupt);
    if (ret != 0) {
        sim_device_trace(&dev->base, "write reg 0x%02X -> NACK/err %d", reg, ret);
        return ret;
    }
    if (dev->hook && dev->hook(dev, false, reg, (uint8_t *)buf, len, &ret)) {
        return ret;
    }
    if (reg > 3) {
        return SIM_I2C_NACK;
    }

    update(dev);
    dev->pointer = reg;
    if (len == 0 || reg == SIM_TMP102_REG_TEMP) {
        return 0; /* pointer-only write, or write to read-only TEMP: ignored */
    }

    uint8_t b[2] = {buf[0], len > 1 ? buf[1] : (uint8_t)dev->regs[reg]};
    if (corrupt) {
        sim_corrupt(b, sizeof(b));
    }
    uint16_t v = (uint16_t)((b[0] << 8) | b[1]);

    if (reg == SIM_TMP102_REG_CONFIG) {
        uint16_t old = dev->regs[reg];
        v = (uint16_t)((v & ~CFG_RO_MASK) | (old & CFG_RO_MASK));
        bool one_shot = (v & CFG_OS) && (v & CFG_SD);
        v &= (uint16_t)~CFG_OS; /* OS reads back 0 once the conversion is done */
        dev->regs[reg] = v;
        if (one_shot) {
            convert(dev, sim_now_ns());
        }
        if ((old & CFG_SD) && !(v & CFG_SD)) {
            dev->last_conversion_ns = sim_now_ns(); /* leaving shutdown restarts the timer */
        }
    } else {
        dev->regs[reg] = v;
    }
    sim_device_trace(&dev->base, "write reg 0x%02X <- 0x%04X", reg, v);
    return 0;
}
