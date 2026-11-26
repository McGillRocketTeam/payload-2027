#include "sim_lsm6dsr.h"

#include <math.h>
#include <string.h>

#include "sim_log.h"

#define STATUS_XLDA 0x01
#define STATUS_GDA  0x02
#define STATUS_TDA  0x04

#define CTRL3_SW_RESET  0x01
#define CTRL3_IF_INC    0x04
#define CTRL3_H_LACTIVE 0x20
#define CTRL3_BOOT      0x80
#define CTRL4_INT2_ON_INT1 0x20

#define R_COUNTER_BDR_REG2 0x0C

/* INT1_CTRL / INT2_CTRL bits (data sheet 9.12, 9.13). */
#define INT_DRDY_XL   0x01
#define INT_DRDY_G    0x02
#define INT2_DRDY_T   0x04
#define INT_FIFO_TH   0x08
#define INT_FIFO_OVR  0x10
#define INT_FIFO_FULL 0x20
#define INT_CNT_BDR   0x40

#define FIFO_BYPASS 0
#define FIFO_FIFO 1
#define FIFO_CONTINUOUS 6

#define TEMP_ODR_HZ 52.0
#define DRDY_PULSE_PS (75ULL * 1000000ULL) /* 75 us (9.10) */

enum { S_G = 0, S_XL = 1, S_T = 2 };
static const uint8_t TAG_OF[3] = {0x01, 0x02, 0x03}; /* gyro NC, accel NC, temperature */

static const double odr_table[16] = {
    0.0, 12.5, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1666.0, 3332.0, 6667.0, 1.6,
};

double sim_lsm6dsr_odr_hz(uint8_t code)
{
    return code < 16 ? odr_table[code] : 0.0;
}

/* Batch data rates (FIFO_CTRL3): as the ODRs, but code 11 is 6.5 Hz. */
static double bdr_hz(uint8_t code)
{
    return code == 11 ? 6.5 : sim_lsm6dsr_odr_hz(code);
}

static bool is_read_only(uint8_t reg)
{
    return reg == LSM6DSR_WHO_AM_I || (reg >= LSM6DSR_STATUS_REG && reg <= 0x2D) ||
           (reg >= LSM6DSR_FIFO_STATUS1 && reg <= 0x3B) || reg >= LSM6DSR_FIFO_DATA_OUT_TAG;
}

static void schedule_next(sim_lsm6dsr_t *dev);

static void fifo_restart(sim_lsm6dsr_t *dev)
{
    dev->fifo_start_ps = sim_now_ps();
    memset(dev->fifo_k, 0, sizeof(dev->fifo_k));
    dev->last_slot_ps = 0;
    memset(dev->stream_has_last, 0, sizeof(dev->stream_has_last));
}

static void fifo_clear(sim_lsm6dsr_t *dev)
{
    dev->fifo_head = dev->fifo_count = 0;
    dev->ovr_latched = false;
    fifo_restart(dev);
}

static void reset_regs(sim_lsm6dsr_t *dev)
{
    memset(dev->regs, 0, sizeof(dev->regs));
    dev->regs[LSM6DSR_WHO_AM_I] = dev->cfg.who_am_i;
    dev->regs[LSM6DSR_CTRL3_C] = CTRL3_IF_INC;
    dev->xl_start_ns = dev->g_start_ns = sim_now_ns();
    dev->xl_sample = dev->g_sample = dev->t_sample = 0;
    dev->xl_read = dev->g_read = dev->t_read = 0;
    fifo_clear(dev);
    dev->bdr_counter = 0;
    dev->counter_ia = false;
    dev->tag_cnt = 0;
    dev->gen++;
}

static void on_power_on(sim_device_t *base)
{
    reset_regs((sim_lsm6dsr_t *)base);
}

sim_lsm6dsr_config_t sim_lsm6dsr_default_config(void)
{
    sim_lsm6dsr_config_t c;
    memset(&c, 0, sizeof(c));
    c.accel_g[0] = sim_signal_const(0.0);
    c.accel_g[1] = sim_signal_const(0.0);
    c.accel_g[2] = sim_signal_const(1.0);
    c.gyro_dps[0] = c.gyro_dps[1] = c.gyro_dps[2] = sim_signal_const(0.0);
    c.temperature_c = sim_signal_const(25.0);
    c.who_am_i = LSM6DSR_ID;
    return c;
}

void sim_lsm6dsr_typical_unit(sim_lsm6dsr_config_t *c, uint64_t seed)
{
    c->seed = seed;
    c->accel_noise_g_rthz = 60e-6;  /* Table 2, high-performance mode */
    c->gyro_noise_dps_rthz = 5e-3;
    for (int i = 0; i < 3; i++) {
        c->accel_offset_g[i] = 0.010 * (2.0 * sim_hash_uniform(seed, 10 + i) - 1.0);
        c->accel_offset_tc_g[i] = 0.0001 * (2.0 * sim_hash_uniform(seed, 20 + i) - 1.0);
        c->gyro_offset_dps[i] = 1.0 * (2.0 * sim_hash_uniform(seed, 30 + i) - 1.0);
    }
    c->gyro_gain_error = 0.01 * (2.0 * sim_hash_uniform(seed, 40) - 1.0);
}

void sim_lsm6dsr_init(sim_lsm6dsr_t *dev, const sim_lsm6dsr_config_t *cfg)
{
    memset(dev, 0, sizeof(*dev));
    sim_device_init(&dev->base, "lsm6dsr");
    dev->base.on_power_on = on_power_on;
    dev->cfg = cfg ? *cfg : sim_lsm6dsr_default_config();
    sim_pin_init(&dev->int1, "imu_int1", false);
    sim_pin_init(&dev->int2, "imu_int2", false);
    reset_regs(dev);
}

stmdev_ctx_t sim_lsm6dsr_ctx(sim_lsm6dsr_t *dev)
{
    stmdev_ctx_t ctx = {0};
    ctx.write_reg = sim_lsm6dsr_write_reg;
    ctx.read_reg = sim_lsm6dsr_read_reg;
    ctx.handle = dev;
    return ctx;
}

/* ------------------------------------------------------------- samples */

static double xl_mg_per_lsb(uint8_t ctrl1)
{
    switch ((ctrl1 >> 2) & 3) {
    case 1: return 0.488; /* 16 g */
    case 2: return 0.122; /* 4 g */
    case 3: return 0.244; /* 8 g */
    default: return 0.061; /* 2 g */
    }
}

static double g_mdps_per_lsb(uint8_t ctrl2)
{
    if (ctrl2 & 0x01) {
        return 140.0; /* 4000 dps */
    }
    if (ctrl2 & 0x02) {
        return 4.375; /* 125 dps */
    }
    static const double fs[4] = {8.75, 17.5, 35.0, 70.0};
    return fs[(ctrl2 >> 2) & 3];
}

static void put_i16(uint8_t *p, double lsb)
{
    double r = round(lsb);
    if (r > 32767.0) {
        r = 32767.0;
    }
    if (r < -32768.0) {
        r = -32768.0;
    }
    int16_t v = (int16_t)r;
    p[0] = (uint8_t)((uint16_t)v & 0xFF);
    p[1] = (uint8_t)((uint16_t)v >> 8);
}

static double xl_odr(const sim_lsm6dsr_t *d)
{
    return sim_lsm6dsr_odr_hz(d->regs[LSM6DSR_CTRL1_XL] >> 4);
}

static double g_odr(const sim_lsm6dsr_t *d)
{
    double f = sim_lsm6dsr_odr_hz(d->regs[LSM6DSR_CTRL2_G] >> 4);
    return f == 1.6 ? 0.0 : f; /* 1.6 Hz is accelerometer low-power only */
}

/* Accelerometer reading (6 bytes, LSB) at time t, with Table 2 errors. */
static void xl_bytes(sim_lsm6dsr_t *d, sim_time_ns_t t, uint64_t idx, uint8_t *out)
{
    const sim_lsm6dsr_config_t *c = &d->cfg;
    double scale = xl_mg_per_lsb(d->regs[LSM6DSR_CTRL1_XL]);
    double rms = c->accel_noise_g_rthz * sqrt(xl_odr(d) / 2.0);
    double dt_c = sim_signal_eval_clean(&c->temperature_c, t) - 25.0;
    for (int i = 0; i < 3; i++) {
        double g = sim_signal_eval(&c->accel_g[i], t) + c->accel_offset_g[i] +
                   c->accel_offset_tc_g[i] * dt_c;
        if (rms > 0.0) {
            g += rms * sim_hash_gauss(c->seed ^ 0xACC, idx * 3 + (uint64_t)i);
        }
        put_i16(out + 2 * i, g * 1000.0 / scale);
    }
}

static void g_bytes(sim_lsm6dsr_t *d, sim_time_ns_t t, uint64_t idx, uint8_t *out)
{
    const sim_lsm6dsr_config_t *c = &d->cfg;
    double scale = g_mdps_per_lsb(d->regs[LSM6DSR_CTRL2_G]);
    double rms = c->gyro_noise_dps_rthz * sqrt(g_odr(d) / 2.0);
    for (int i = 0; i < 3; i++) {
        double dps = sim_signal_eval(&c->gyro_dps[i], t) * (1.0 + c->gyro_gain_error) +
                     c->gyro_offset_dps[i];
        if (rms > 0.0) {
            dps += rms * sim_hash_gauss(c->seed ^ 0x6E0, idx * 3 + (uint64_t)i);
        }
        put_i16(out + 2 * i, dps * 1000.0 / scale);
    }
}

static void t_bytes(sim_lsm6dsr_t *d, sim_time_ns_t t, uint8_t *out)
{
    put_i16(out, (sim_signal_eval(&d->cfg.temperature_c, t) - 25.0) * 256.0);
}

/* Index of the newest sample at `now` for a stream started at `start`. */
static uint64_t sample_index(double odr, sim_time_ns_t start, sim_time_ns_t now)
{
    if (odr <= 0.0 || now < start) {
        return 0;
    }
    return (uint64_t)floor((double)(now - start) * odr / 1e9);
}

static int fifo_mode(const sim_lsm6dsr_t *d)
{
    return d->regs[LSM6DSR_FIFO_CTRL4] & 0x07;
}

/* Refresh output registers and STATUS_REG to the current sim time. */
static void update_outputs(sim_lsm6dsr_t *dev)
{
    sim_time_ns_t now = sim_now_ns();
    bool stuck = sim_device_is_stuck(&dev->base);
    uint8_t status = 0;

    double xo = xl_odr(dev);
    if (xo > 0.0) {
        uint64_t k = sample_index(xo, dev->xl_start_ns, now) + 1; /* first sample at t0 */
        if (k > dev->xl_sample) {
            /* Every sample between the last one read and the new one is lost. */
            uint64_t lost = k - dev->xl_sample - 1;
            if (dev->xl_sample > dev->xl_read) {
                lost++;
            }
            if (lost > 0 && fifo_mode(dev) == FIFO_BYPASS) {
                dev->xl_samples_missed += (uint32_t)lost;
                sim_log(dev->base.name, "error", "xl_samples_missed", (double)lost,
                        "not read before overwritten");
            }
            dev->xl_sample = k;
            if (!stuck) {
                sim_time_ns_t t = dev->xl_start_ns + (sim_time_ns_t)((double)(k - 1) * 1e9 / xo);
                xl_bytes(dev, t, k, &dev->regs[LSM6DSR_OUTX_L_A]);
            }
        }
        if (dev->xl_sample > dev->xl_read) {
            status |= STATUS_XLDA;
        }
    }

    double go = g_odr(dev);
    if (go > 0.0) {
        uint64_t k = sample_index(go, dev->g_start_ns, now) + 1;
        if (k > dev->g_sample) {
            dev->g_sample = k;
            if (!stuck) {
                sim_time_ns_t t = dev->g_start_ns + (sim_time_ns_t)((double)(k - 1) * 1e9 / go);
                g_bytes(dev, t, k, &dev->regs[LSM6DSR_OUTX_L_G]);
            }
        }
        if (dev->g_sample > dev->g_read) {
            status |= STATUS_GDA;
        }
    }

    if (xo > 0.0 || go > 0.0) {
        uint64_t k = sample_index(TEMP_ODR_HZ, 0, now) + 1;
        if (k > dev->t_sample) {
            dev->t_sample = k;
            if (!stuck) {
                t_bytes(dev, now, &dev->regs[LSM6DSR_OUT_TEMP_L]);
            }
        }
        if (dev->t_sample > dev->t_read) {
            status |= STATUS_TDA;
        }
    }
    dev->regs[LSM6DSR_STATUS_REG] = status;
}

/* ---------------------------------------------------------------- FIFO */

static double stream_rate(const sim_lsm6dsr_t *d, int s)
{
    uint8_t c3 = d->regs[LSM6DSR_FIFO_CTRL3];
    switch (s) {
    case S_XL: {
        double odr = xl_odr(d);
        double b = bdr_hz(c3 & 0x0F);
        return odr <= 0.0 || b <= 0.0 ? 0.0 : fmin(odr, b);
    }
    case S_G: {
        double odr = g_odr(d);
        double b = bdr_hz(c3 >> 4);
        return odr <= 0.0 || b <= 0.0 ? 0.0 : fmin(odr, b);
    }
    default: {
        static const double tb[4] = {0.0, 52.0, 12.5, 1.6};
        bool on = xl_odr(d) > 0.0 || g_odr(d) > 0.0;
        return on ? tb[(d->regs[LSM6DSR_FIFO_CTRL4] >> 4) & 3] : 0.0;
    }
    }
}

static sim_time_ps_t stream_time(const sim_lsm6dsr_t *d, uint64_t k, double rate)
{
    return d->fifo_start_ps + (sim_time_ps_t)llround((double)(k + 1) * 1e12 / rate);
}

static int wtm(const sim_lsm6dsr_t *d)
{
    return d->regs[LSM6DSR_FIFO_CTRL1] | ((d->regs[LSM6DSR_FIFO_CTRL2] & 1) << 8);
}

static int depth(const sim_lsm6dsr_t *d)
{
    bool stop_on_wtm = (d->regs[LSM6DSR_FIFO_CTRL2] & 0x80) != 0;
    return stop_on_wtm && wtm(d) > 0 ? wtm(d) : SIM_LSM6DSR_FIFO_WORDS;
}

static uint32_t cnt_threshold(const sim_lsm6dsr_t *d)
{
    return (uint32_t)(d->regs[LSM6DSR_COUNTER_BDR_REG1] & 0x07) << 8 | d->regs[R_COUNTER_BDR_REG2];
}

/* Earliest next batch event: returns the stream, or -1 if none is batching. */
static int next_stream(const sim_lsm6dsr_t *d, const uint64_t *k, sim_time_ps_t *t_out)
{
    int best = -1;
    sim_time_ps_t bt = 0;
    for (int s = 0; s < 3; s++) {
        double r = stream_rate(d, s);
        if (r <= 0.0) {
            continue;
        }
        sim_time_ps_t t = stream_time(d, k[s], r);
        if (best < 0 || t < bt) { /* ties: gyro, accel, temperature */
            best = s;
            bt = t;
        }
    }
    *t_out = bt;
    return best;
}

static void push_word(sim_lsm6dsr_t *d, int s, sim_time_ps_t t, uint64_t k)
{
    int mode = fifo_mode(d);
    bool full = d->fifo_count >= depth(d);
    if (full && mode == FIFO_FIFO) {
        return; /* FIFO mode: stops when full */
    }
    if (full) {
        /* Continuous: the oldest word is overwritten. */
        d->fifo_head = (d->fifo_head + 1) % SIM_LSM6DSR_FIFO_WORDS;
        d->fifo_count--;
        d->fifo_overruns++;
        if (!d->ovr_latched) {
            sim_log(d->base.name, "error", "fifo_overrun", d->fifo_count,
                    "oldest samples overwritten: FIFO not read in time");
        }
        d->ovr_latched = true;
    }
    if (t != d->last_slot_ps) {
        d->tag_cnt = (uint8_t)((d->tag_cnt + 1) & 3); /* new time slot */
        d->last_slot_ps = t;
    }
    sim_lsm6dsr_word_t *w = &d->fifo[(d->fifo_head + d->fifo_count) % SIM_LSM6DSR_FIFO_WORDS];
    uint8_t tag = (uint8_t)(TAG_OF[s] << 3 | d->tag_cnt << 1);
    uint8_t x = tag;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    w->tag = (uint8_t)(tag | (x & 1)); /* UNVERIFIED parity rule */
    memset(w->data, 0, sizeof(w->data));
    sim_time_ns_t tns = t / SIM_PS_PER_NS;
    if (sim_device_is_stuck(&d->base) && d->stream_has_last[s]) {
        memcpy(w->data, d->stream_last[s], 6); /* this sensor's last value */
    } else if (s == S_XL) {
        xl_bytes(d, tns, (1ULL << 40) + k, w->data);
    } else if (s == S_G) {
        g_bytes(d, tns, (1ULL << 40) + k, w->data);
    } else {
        t_bytes(d, tns, w->data);
    }
    memcpy(d->stream_last[s], w->data, 6);
    d->stream_has_last[s] = true;
    d->fifo_count++;

    int trig = (d->regs[LSM6DSR_COUNTER_BDR_REG1] & 0x20) ? S_G : S_XL;
    if (s == trig) {
        d->bdr_counter++;
        uint32_t th = cnt_threshold(d);
        if (th > 0 && d->bdr_counter >= th) {
            d->counter_ia = true;
            d->bdr_counter = 0;
        }
    }
}

/* Bring the FIFO up to the current time. */
static void fifo_update(sim_lsm6dsr_t *d)
{
    int mode = fifo_mode(d);
    if (mode != FIFO_FIFO && mode != FIFO_CONTINUOUS) {
        return;
    }
    sim_time_ps_t now = sim_now_ps();
    if (mode == FIFO_FIFO && d->fifo_count >= depth(d)) {
        /* Full in FIFO mode: nothing more is stored, just keep the clock. */
        for (int s = 0; s < 3; s++) {
            double r = stream_rate(d, s);
            if (r > 0.0 && now > d->fifo_start_ps) {
                d->fifo_k[s] = (uint64_t)floor((double)(now - d->fifo_start_ps) * r / 1e12);
            }
        }
        return;
    }
    for (;;) {
        sim_time_ps_t t;
        int s = next_stream(d, d->fifo_k, &t);
        if (s < 0 || t > now) {
            return;
        }
        push_word(d, s, t, d->fifo_k[s]);
        d->fifo_k[s]++;
    }
}

/* Time at which the FIFO will hold `level` words (if nobody reads). */
static sim_time_ps_t time_of_level(sim_lsm6dsr_t *d, int level)
{
    uint64_t k[3];
    memcpy(k, d->fifo_k, sizeof(k));
    int count = d->fifo_count;
    sim_time_ps_t t = 0;
    while (count < level) {
        int s = next_stream(d, k, &t);
        if (s < 0) {
            return 0;
        }
        k[s]++;
        count++;
    }
    return t;
}

/* ---------------------------------------------------------- interrupts */

typedef struct {
    bool wtm, ovr, full;
} fifo_flags_t;

static fifo_flags_t fifo_flags(const sim_lsm6dsr_t *d)
{
    fifo_flags_t f;
    f.wtm = wtm(d) > 0 && d->fifo_count >= wtm(d);
    f.ovr = d->fifo_count >= depth(d);       /* completely filled */
    f.full = d->fifo_count >= depth(d) - 1;  /* full at the next sample */
    return f;
}

static uint8_t int_sources(sim_lsm6dsr_t *d, bool int2)
{
    uint8_t st = d->regs[LSM6DSR_STATUS_REG];
    fifo_flags_t f = fifo_flags(d);
    bool pulsed = (d->regs[LSM6DSR_COUNTER_BDR_REG1] & 0x80) != 0;
    bool in_pulse = sim_now_ps() < d->pulse_until_ps;
    uint8_t s = 0;
    if (pulsed ? in_pulse : (st & STATUS_XLDA)) {
        s |= INT_DRDY_XL;
    }
    if (pulsed ? in_pulse : (st & STATUS_GDA)) {
        s |= INT_DRDY_G;
    }
    if (int2 && (st & STATUS_TDA)) {
        s |= INT2_DRDY_T;
    }
    if (f.wtm) {
        s |= INT_FIFO_TH;
    }
    if (f.ovr) {
        s |= INT_FIFO_OVR;
    }
    if (f.full) {
        s |= INT_FIFO_FULL;
    }
    if (d->counter_ia) {
        s |= INT_CNT_BDR;
    }
    return s;
}

static void evaluate_pins(sim_lsm6dsr_t *d)
{
    bool a1 = (d->regs[LSM6DSR_INT1_CTRL] & 0x7B & int_sources(d, false)) != 0;
    bool a2 = (d->regs[LSM6DSR_INT2_CTRL] & 0x7F & int_sources(d, true)) != 0;
    if (d->regs[LSM6DSR_CTRL4_C] & CTRL4_INT2_ON_INT1) {
        a1 = a1 || a2;
        a2 = false;
    }
    bool low_active = (d->regs[LSM6DSR_CTRL3_C] & CTRL3_H_LACTIVE) != 0;
    sim_pin_set(&d->int1, a1 != low_active);
    sim_pin_set(&d->int2, a2 != low_active);
}

static void refresh(sim_lsm6dsr_t *d);

static void pin_event(void *ctx, uint32_t tag)
{
    sim_lsm6dsr_t *d = ctx;
    if (tag != d->gen) {
        return;
    }
    refresh(d);
    schedule_next(d);
}

/* First sample time strictly after now, rounded up so that the sample is
 * really there when the event fires. */
static sim_time_ps_t next_sample_ps(sim_time_ns_t start_ns, double odr)
{
    uint64_t k = sample_index(odr, start_ns, sim_now_ns()) + 1;
    sim_time_ps_t t;
    do {
        t = (start_ns + (sim_time_ns_t)ceil((double)k * 1e9 / odr)) * SIM_PS_PER_NS;
        k++;
    } while (t <= sim_now_ps());
    return t;
}

/* Next time an interrupt line can change on its own, and an event there. */
static void schedule_next(sim_lsm6dsr_t *d)
{
    d->gen++;
    evaluate_pins(d);
    uint8_t routed = d->regs[LSM6DSR_INT1_CTRL] | d->regs[LSM6DSR_INT2_CTRL];
    if (!d->base.powered || routed == 0) {
        return;
    }
    sim_time_ps_t now = sim_now_ps();
    sim_time_ps_t next = 0;
#define CONSIDER(t)                                    \
    do {                                               \
        sim_time_ps_t t_ = (t);                        \
        if (t_ > now && (next == 0 || t_ < next)) {    \
            next = t_;                                 \
        }                                              \
    } while (0)

    bool pulsed = (d->regs[LSM6DSR_COUNTER_BDR_REG1] & 0x80) != 0;
    if (routed & (INT_DRDY_XL | INT_DRDY_G | INT2_DRDY_T)) {
        double xo = xl_odr(d);
        uint8_t st = d->regs[LSM6DSR_STATUS_REG];
        if (xo > 0.0 && (pulsed || !(st & STATUS_XLDA))) {
            CONSIDER(next_sample_ps(d->xl_start_ns, xo));
        }
        double go = g_odr(d);
        if (go > 0.0 && (pulsed || !(st & STATUS_GDA))) {
            CONSIDER(next_sample_ps(d->g_start_ns, go));
        }
        if (pulsed && d->pulse_until_ps > now) {
            CONSIDER(d->pulse_until_ps);
        }
    }
    int mode = fifo_mode(d);
    if ((mode == FIFO_FIFO || mode == FIFO_CONTINUOUS) &&
        (routed & (INT_FIFO_TH | INT_FIFO_OVR | INT_FIFO_FULL | INT_CNT_BDR))) {
        fifo_flags_t f = fifo_flags(d);
        if ((routed & INT_FIFO_TH) && !f.wtm && wtm(d) > 0) {
            CONSIDER(time_of_level(d, wtm(d)));
        }
        if ((routed & INT_FIFO_FULL) && !f.full) {
            CONSIDER(time_of_level(d, depth(d) - 1));
        }
        if ((routed & INT_FIFO_OVR) && !f.ovr) {
            CONSIDER(time_of_level(d, depth(d)));
        }
        if ((routed & INT_CNT_BDR) && !d->counter_ia && cnt_threshold(d) > 0) {
            sim_time_ps_t t;
            int s = next_stream(d, d->fifo_k, &t);
            if (s >= 0) {
                CONSIDER(t); /* step through batch events */
            }
        }
    }
#undef CONSIDER
    if (next) {
        sim_schedule_ps(next, pin_event, d, d->gen);
    }
}

/* Data-ready pulses (pulsed mode): 75 us after each new sample. */
static void note_new_samples(sim_lsm6dsr_t *d, uint64_t xl_before, uint64_t g_before)
{
    if ((d->regs[LSM6DSR_COUNTER_BDR_REG1] & 0x80) &&
        (d->xl_sample > xl_before || d->g_sample > g_before)) {
        d->pulse_until_ps = sim_now_ps() + DRDY_PULSE_PS;
    }
}

/* ------------------------------------------------------------------ bus */

static void refresh(sim_lsm6dsr_t *d)
{
    uint64_t xb = d->xl_sample;
    uint64_t gb = d->g_sample;
    update_outputs(d);
    note_new_samples(d, xb, gb);
    fifo_update(d);
}

int32_t sim_lsm6dsr_read_reg(void *handle, uint8_t reg, uint8_t *buf, uint16_t len)
{
    sim_lsm6dsr_t *dev = handle;
    sim_fault_mode_t f = sim_device_begin(&dev->base);
    int32_t ret = 0;

    if (f == SIM_FAULT_OFF) {
        sim_spi_idle_fill(dev->cfg.miso, buf, len); /* nobody drives MISO */
        return 0;
    }
    if (f == SIM_FAULT_BUS_ERROR) {
        return -1;
    }
    if (dev->hook && dev->hook(dev, true, reg, buf, len, &ret)) {
        return ret;
    }

    refresh(dev); /* one consistent snapshot per transaction */
    bool inc = (dev->regs[LSM6DSR_CTRL3_C] & CTRL3_IF_INC) != 0;
    uint8_t a = reg & 0x7F;
    for (uint16_t i = 0; i < len; i++) {
        if (a == LSM6DSR_FIFO_STATUS1) {
            buf[i] = (uint8_t)(dev->fifo_count & 0xFF);
        } else if (a == LSM6DSR_FIFO_STATUS2) {
            fifo_flags_t ff = fifo_flags(dev);
            buf[i] = (uint8_t)(((dev->fifo_count >> 8) & 3) | (dev->ovr_latched ? 0x08 : 0) |
                               (dev->counter_ia ? 0x10 : 0) | (ff.full ? 0x20 : 0) |
                               (ff.ovr ? 0x40 : 0) | (ff.wtm ? 0x80 : 0));
            dev->ovr_latched = false; /* reset when this register is read */
            dev->counter_ia = false;
        } else if (a > 0x7E) {
            buf[i] = 0; /* 0x7F: reserved */
        } else if (a >= LSM6DSR_FIFO_DATA_OUT_TAG) {
            const sim_lsm6dsr_word_t *w = &dev->fifo[dev->fifo_head];
            bool empty = dev->fifo_count == 0;
            buf[i] = empty ? 0 : (a == LSM6DSR_FIFO_DATA_OUT_TAG ? w->tag : w->data[a - 0x79]);
            if (a == 0x7E && !empty) { /* UNVERIFIED: word popped after its last byte */
                dev->fifo_head = (dev->fifo_head + 1) % SIM_LSM6DSR_FIFO_WORDS;
                dev->fifo_count--;
            }
        } else {
            buf[i] = dev->regs[a];
        }
        if (a >= LSM6DSR_OUTX_L_A && a <= 0x2D) {
            dev->xl_read = dev->xl_sample;
        } else if (a >= LSM6DSR_OUTX_L_G && a <= 0x27) {
            dev->g_read = dev->g_sample;
        } else if (a == LSM6DSR_OUT_TEMP_L || a == LSM6DSR_OUT_TEMP_L + 1) {
            dev->t_read = dev->t_sample;
        }
        if (inc) {
            /* FIFO output rolls back from 0x7E to 0x78 for multi-word bursts. */
            a = a == 0x7E ? LSM6DSR_FIFO_DATA_OUT_TAG : (uint8_t)((a + 1) & 0x7F);
        }
    }
    if (f == SIM_FAULT_CORRUPT) {
        sim_corrupt(buf, len);
    }
    update_outputs(dev); /* clears data-ready flags that were just read */
    schedule_next(dev);
    sim_device_trace(&dev->base, "read  0x%02X x%u -> 0x%02X...", reg, len, len ? buf[0] : 0);
    return 0;
}

static void on_fifo_ctrl4(sim_lsm6dsr_t *d, uint8_t old, uint8_t v)
{
    int mode = v & 0x07;
    if (mode != FIFO_BYPASS && mode != FIFO_FIFO && mode != FIFO_CONTINUOUS) {
        sim_log(d->base.name, "error", "unmodelled", mode,
                "trigger-based FIFO modes are not simulated (treated as bypass)");
    }
    if (mode == FIFO_BYPASS || (old & 0x07) == FIFO_BYPASS) {
        fifo_clear(d); /* bypass resets the FIFO; leaving it starts batching */
    } else {
        fifo_restart(d);
    }
}

int32_t sim_lsm6dsr_write_reg(void *handle, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    sim_lsm6dsr_t *dev = handle;
    sim_fault_mode_t f = sim_device_begin(&dev->base);
    int32_t ret = 0;

    if (f == SIM_FAULT_OFF) {
        return 0; /* written into the void */
    }
    if (f == SIM_FAULT_BUS_ERROR) {
        return -1;
    }
    if (dev->hook && dev->hook(dev, false, reg, (uint8_t *)buf, len, &ret)) {
        return ret;
    }

    refresh(dev);
    bool inc = (dev->regs[LSM6DSR_CTRL3_C] & CTRL3_IF_INC) != 0;
    uint8_t a = reg & 0x7F;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t v = buf[i];
        if (f == SIM_FAULT_CORRUPT && i == 0) {
            sim_corrupt(&v, 1);
        }
        if (!is_read_only(a)) {
            uint8_t old = dev->regs[a];
            dev->regs[a] = v;
            if (a == LSM6DSR_CTRL1_XL && (old >> 4) != (v >> 4)) {
                dev->xl_start_ns = sim_now_ns();
                dev->xl_sample = dev->xl_read = 0;
                fifo_restart(dev);
            } else if (a == LSM6DSR_CTRL2_G && (old >> 4) != (v >> 4)) {
                dev->g_start_ns = sim_now_ns();
                dev->g_sample = dev->g_read = 0;
                fifo_restart(dev);
            } else if (a == LSM6DSR_CTRL3_C && (v & (CTRL3_SW_RESET | CTRL3_BOOT))) {
                reset_regs(dev); /* bits self-clear: reset value has them at 0 */
                schedule_next(dev);
                sim_device_trace(&dev->base, "software reset");
                return 0;
            } else if (a == LSM6DSR_FIFO_CTRL4) {
                on_fifo_ctrl4(dev, old, v);
            } else if (a == LSM6DSR_FIFO_CTRL3) {
                fifo_restart(dev);
            } else if (a == LSM6DSR_FIFO_CTRL2 && (v & 0x40)) {
                sim_log(dev->base.name, "error", "unmodelled", v, "FIFO compression is not simulated");
            } else if (a == LSM6DSR_COUNTER_BDR_REG1 && (v & 0x40)) {
                dev->bdr_counter = 0;
                dev->regs[a] = (uint8_t)(v & ~0x40); /* RST_COUNTER_BDR self-clears */
            }
        }
        if (inc) {
            a = (uint8_t)((a + 1) & 0x7F);
        }
    }
    schedule_next(dev);
    sim_device_trace(&dev->base, "write 0x%02X x%u <- 0x%02X...", reg, len, len ? buf[0] : 0);
    return 0;
}
