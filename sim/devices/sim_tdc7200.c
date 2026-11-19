#include "sim_tdc7200.h"

#include <math.h>
#include <string.h>

#include "sim_log.h"

/* Register addresses (data sheet Table 1). */
#define R_CONFIG1 0x00
#define R_CONFIG2 0x01
#define R_INT_STATUS 0x02
#define R_INT_MASK 0x03
#define R_COARSE_OVF_H 0x04
#define R_COARSE_OVF_L 0x05
#define R_CLOCK_OVF_H 0x06
#define R_CLOCK_OVF_L 0x07
#define R_STOP_MASK_H 0x08
#define R_STOP_MASK_L 0x09
#define R_TIME1 0x10
#define R_CAL1 0x1B
#define R_CAL2 0x1C

/* CONFIG1 bits */
#define C1_START_MEAS 0x01
#define C1_MEAS_MODE_SHIFT 1
#define C1_START_EDGE 0x08
#define C1_STOP_EDGE 0x10
#define C1_TRIGG_EDGE 0x20
#define C1_PARITY_EN 0x40
#define C1_FORCE_CAL 0x80

/* INT_STATUS bits */
#define IS_NEW_MEAS 0x01
#define IS_COARSE_OVF 0x02
#define IS_CLOCK_OVF 0x04
#define IS_STARTED 0x08
#define IS_COMPLETE 0x10

/* Data sheet 8.4.7 / 7.7 start-up times. */
#define SPI_READY_S 100e-6
#define LDO_SET1_S 300e-6
#define LDO_SET2_S 1.5e-3

static const uint8_t RESET[10] = {0x00, 0x40, 0x00, 0x07, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00};

sim_tdc7200_config_t sim_tdc7200_default_config(sim_osc_t *clock, double clock_hz)
{
    sim_tdc7200_config_t c;
    memset(&c, 0, sizeof(c));
    c.clock = clock;
    c.clock_hz = clock_hz;
    c.lsb_s = 55e-12;        /* data sheet 7.5, typical */
    c.noise_rms_s = 18e-12;  /* with LSB quantisation: ~35 ps std in mode 2 (7.5) */
    c.offset_s = 0.0;        /* UNVERIFIED */
    c.miso = SIM_MISO_PULL_UP;
    return c;
}

static void reset_regs(sim_tdc7200_t *d)
{
    memset(d->regs, 0, sizeof(d->regs));
    for (int i = 0; i < 10; i++) {
        d->regs[i] = RESET[i];
    }
    d->state = TDC_IDLE;
    d->gen++;
}

void sim_tdc7200_init(sim_tdc7200_t *dev, const char *name, const sim_tdc7200_config_t *cfg)
{
    memset(dev, 0, sizeof(*dev));
    sim_device_init(&dev->base, name);
    dev->cfg = *cfg;
    sim_pin_init(&dev->intb, name, true);
    sim_pin_init(&dev->trigg, name, false);
    reset_regs(dev);
    dev->enabled = false;
}

tdc7200_ctx_t sim_tdc7200_ctx(sim_tdc7200_t *dev)
{
    tdc7200_ctx_t ctx = {.spi_transfer = sim_tdc7200_spi_transfer, .handle = dev};
    return ctx;
}

/* ------------------------------------------------------------- helpers */

static uint32_t reg16(const sim_tdc7200_t *d, int hi)
{
    return d->regs[hi] << 8 | d->regs[hi + 1];
}

static int meas_mode(const sim_tdc7200_t *d)
{
    return ((d->regs[R_CONFIG1] >> C1_MEAS_MODE_SHIFT) & 3) == 1 ? 2 : 1;
}

static int num_stops(const sim_tdc7200_t *d)
{
    int n = (int)(d->regs[R_CONFIG2] & 7) + 1;
    return n > 5 ? 1 : n; /* 101-111: single stop */
}

static int avg_cycles(const sim_tdc7200_t *d)
{
    return 1 << ((d->regs[R_CONFIG2] >> 3) & 7);
}

static int cal2_periods(const sim_tdc7200_t *d)
{
    static const int p[4] = {2, 10, 20, 40};
    return p[(d->regs[R_CONFIG2] >> 6) & 3];
}

static double local_time(sim_tdc7200_t *d, double t)
{
    return d->cfg.clock ? t + sim_osc_error_at(d->cfg.clock, (sim_time_ps_t)llround(t * 1e12)) : t;
}

/* True time of the first CLOCK rising edge strictly after true time t. */
static double clock_edge_after(sim_tdc7200_t *d, double t)
{
    double f = d->cfg.clock_hz;
    double k = floor(local_time(d, t) * f) + 1.0;
    double edge = d->cfg.clock ? sim_osc_true_time(d->cfg.clock, k / f) : k / f;
    while (edge <= t) {
        k += 1.0;
        edge = d->cfg.clock ? sim_osc_true_time(d->cfg.clock, k / f) : k / f;
    }
    return edge;
}

/* Real (true-time) CLOCK period now. */
static double clock_period(sim_tdc7200_t *d)
{
    double y = d->cfg.clock ? sim_osc_freq_at(d->cfg.clock, sim_now_ps()) : 0.0;
    return 1.0 / (d->cfg.clock_hz * (1.0 + y));
}

/* LDO not settled yet: up to 0.3 % of tCLOCK error (data sheet 8.4.7). */
static double ldo_error(sim_tdc7200_t *d)
{
    double since = (double)(sim_now_ps() - d->enable_ps) / 1e12;
    if (since >= LDO_SET2_S) {
        return 0.0;
    }
    double scale = since < LDO_SET1_S ? 1.0 : (LDO_SET2_S - since) / (LDO_SET2_S - LDO_SET1_S);
    return 0.003 * scale / d->cfg.clock_hz;
}

/* One fine (ring oscillator) measurement of dt seconds, in LSB counts. */
static double ring(sim_tdc7200_t *d, double dt)
{
    double noise = d->cfg.noise_rms_s * sim_hash_gauss(d->cfg.seed ^ 0x7DC, d->noise_index++);
    double counts = (dt + d->cfg.offset_s + noise + ldo_error(d)) / d->cfg.lsb_s;
    return counts < 0.0 ? 0.0 : counts;
}

static uint32_t clamp23(double v)
{
    double r = round(v);
    return r > 0x7FFFFF ? 0x7FFFFF : (uint32_t)r;
}

static uint32_t with_parity(const sim_tdc7200_t *d, uint32_t v)
{
    v &= 0x7FFFFF;
    if (d->regs[R_CONFIG1] & C1_PARITY_EN) {
        uint32_t x = v;
        x ^= x >> 16;
        x ^= x >> 8;
        x ^= x >> 4;
        x ^= x >> 2;
        x ^= x >> 1;
        if (x & 1) {
            v |= 0x800000; /* even parity over all 24 bits */
        }
    }
    return v;
}

static void update_intb(sim_tdc7200_t *d)
{
    bool active = (d->regs[R_INT_STATUS] & d->regs[R_INT_MASK] & 0x07) != 0;
    sim_pin_set(&d->intb, !active); /* active low */
}

static void set_trigg(sim_tdc7200_t *d, bool active)
{
    bool falling = (d->regs[R_CONFIG1] & C1_TRIGG_EDGE) != 0;
    sim_pin_set(&d->trigg, falling ? !active : active);
}

/* -------------------------------------------------------- measurement */

static void arm(sim_tdc7200_t *d)
{
    d->state = TDC_ARMED;
    set_trigg(d, true); /* TRIGG, then START is enabled */
}

static void start_new_measurement(sim_tdc7200_t *d)
{
    if (d->cfg.clock_hz < 1e6 || d->cfg.clock_hz > 16e6) {
        sim_log(d->base.name, "error", "clock_out_of_range", d->cfg.clock_hz,
                "data sheet: 1-16 MHz");
    }
    d->gen++;
    d->regs[R_INT_STATUS] = 0;
    for (int a = R_TIME1; a <= R_CAL2; a++) {
        d->regs[a] = 0;
    }
    memset(d->time_acc, 0, sizeof(d->time_acc));
    memset(d->cc_acc, 0, sizeof(d->cc_acc));
    d->cycle = 0;
    update_intb(d);
    arm(d);
}

static void write_results(sim_tdc7200_t *d, bool calibrate)
{
    int avg = avg_cycles(d);
    int cycles = d->cycle > 0 ? d->cycle : 1;
    (void)avg;
    for (int i = 0; i < 6; i++) {
        d->regs[R_TIME1 + 2 * i] = with_parity(d, clamp23(d->time_acc[i] / cycles));
    }
    for (int i = 0; i < 5; i++) {
        d->regs[R_TIME1 + 1 + 2 * i] = with_parity(d, d->cc_acc[i] & 0x7FFFFF);
    }
    if (calibrate) {
        double tp = clock_period(d);
        d->regs[R_CAL1] = with_parity(d, clamp23(ring(d, tp)));
        d->regs[R_CAL2] = with_parity(d, clamp23(ring(d, cal2_periods(d) * tp)));
    }
}

static void complete_event(void *ctx, uint32_t tag)
{
    sim_tdc7200_t *d = ctx;
    if (tag != d->gen || d->state != TDC_CALIBRATING || !d->enabled) {
        return;
    }
    if (d->cycle < avg_cycles(d)) {
        arm(d); /* next averaging cycle: TRIGG after the calibration */
        return;
    }
    write_results(d, true);
    d->regs[R_INT_STATUS] |= IS_NEW_MEAS | IS_COMPLETE;
    d->regs[R_CONFIG1] &= ~(uint32_t)C1_START_MEAS;
    d->state = TDC_IDLE;
    d->measurements++;
    update_intb(d);
}

static double cal_time(sim_tdc7200_t *d)
{
    /* UNVERIFIED: calibration runs after the last STOP; ~(N + 3) periods. */
    return (cal2_periods(d) + 3) * clock_period(d);
}

static void overflow_event(void *ctx, uint32_t tag)
{
    sim_tdc7200_t *d = ctx;
    if (tag != d->gen || d->state != TDC_RUNNING || !d->enabled) {
        return;
    }
    bool mode2 = meas_mode(d) == 2;
    d->regs[R_INT_STATUS] |= mode2 ? IS_CLOCK_OVF : IS_COARSE_OVF;
    d->overflows++;
    sim_log(d->base.name, "error", mode2 ? "clock_counter_overflow" : "coarse_counter_overflow",
            d->stops_seen, "STOP %d of %d never came within range", d->stops_seen + 1,
            num_stops(d));
    /* Running measurement stops immediately; calibration only with FORCE_CAL. */
    d->cycle++;
    write_results(d, (d->regs[R_CONFIG1] & C1_FORCE_CAL) != 0);
    d->regs[R_CONFIG1] &= ~(uint32_t)C1_START_MEAS;
    d->state = TDC_IDLE;
    update_intb(d);
}

static void on_start(void *ctx, bool rising, sim_time_ps_t t_ps)
{
    sim_tdc7200_t *d = ctx;
    bool want_rising = !(d->regs[R_CONFIG1] & C1_START_EDGE);
    if (!d->enabled || d->state != TDC_ARMED || rising != want_rising) {
        return;
    }
    double t = (double)t_ps / 1e12;
    d->state = TDC_RUNNING;
    d->start_ps = t_ps;
    d->last_stop_ps = t_ps;
    d->stops_seen = 0;
    d->regs[R_INT_STATUS] |= IS_STARTED;
    set_trigg(d, false); /* TRIGG reset after START */

    double overflow_at;
    if (meas_mode(d) == 2) {
        d->t_clk1_s = clock_edge_after(d, t);
        d->time_acc[0] += ring(d, d->t_clk1_s - t); /* TIME1: START -> CLOCK */
        /* Clock counter overflows when it passes CLOCK_CNTR_OVF. */
        overflow_at = d->t_clk1_s + (reg16(d, R_CLOCK_OVF_H) + 1) * clock_period(d);
    } else {
        /* Coarse counter: TIMEn / 63 >= COARSE_CNTR_OVF. */
        overflow_at = t + 63.0 * reg16(d, R_COARSE_OVF_H) * d->cfg.lsb_s;
    }
    sim_schedule_ps((sim_time_ps_t)llround(overflow_at * 1e12), overflow_event, d, d->gen);
}

static void stop_edge(sim_tdc7200_t *d, bool rising, sim_time_ps_t t_ps);

/* The delay line: the edge reaches the STOP pin stop_delay_s later. */
static void delayed_stop_event(void *ctx, uint32_t tag)
{
    stop_edge(ctx, tag != 0, sim_event_time_ps());
}

static void on_stop(void *ctx, bool rising, sim_time_ps_t t_ps)
{
    sim_tdc7200_t *d = ctx;
    if (d->cfg.stop_delay_s > 0.0) {
        sim_time_ps_t t = t_ps + (sim_time_ps_t)llround(d->cfg.stop_delay_s * 1e12);
        sim_schedule_ps(t, delayed_stop_event, d, rising ? 1u : 0u);
        return;
    }
    stop_edge(d, rising, t_ps);
}

static void stop_edge(sim_tdc7200_t *d, bool rising, sim_time_ps_t t_ps)
{
    bool want_rising = !(d->regs[R_CONFIG1] & C1_STOP_EDGE);
    if (!d->enabled || d->state != TDC_RUNNING || rising != want_rising) {
        return;
    }
    double t = (double)t_ps / 1e12;
    double since_prev = (double)(t_ps - d->last_stop_ps) / 1e12;
    bool first = d->stops_seen == 0;
    int i = d->stops_seen;

    if (meas_mode(d) == 2) {
        double after = clock_edge_after(d, t);
        uint32_t cc = (uint32_t)llround((local_time(d, after) - local_time(d, d->t_clk1_s)) *
                                        d->cfg.clock_hz);
        uint32_t mask = reg16(d, R_STOP_MASK_H);
        if (mask > 0 && cc <= mask) {
            return; /* inside the STOP mask window: ignored */
        }
        if (since_prev < 2.0 * clock_period(d)) {
            sim_log(d->base.name, "error", "timing_violation", since_prev,
                    "mode 2 needs >= 2 tCLOCK between edges");
        }
        d->time_acc[i + 1] += ring(d, after - t); /* TIME(n+1): STOP -> CLOCK */
        d->cc_acc[i] += cc;
    } else {
        double min = first ? 12e-9 : 67e-9;
        if (since_prev < min) {
            sim_log(d->base.name, "error", "timing_violation", since_prev,
                    "mode 1 minimum %s", first ? "START-STOP 12 ns" : "STOP-STOP 67 ns");
        }
        if ((double)(t_ps - d->start_ps) / 1e12 > 500e-9) {
            sim_log(d->base.name, "error", "mode1_range", (double)(t_ps - d->start_ps) / 1e12,
                    "mode 1 is specified to 500 ns");
        }
        d->time_acc[i] += ring(d, (double)(t_ps - d->start_ps) / 1e12); /* START -> STOP n */
    }
    if (first && sim_log_is_open()) {
        sim_log(d->base.name, "truth", "tof_s", (double)(t_ps - d->start_ps) / 1e12,
                "START -> STOP 1, cycle %d", d->cycle + 1);
    }
    d->last_stop_ps = t_ps;
    d->stops_seen++;
    if (d->stops_seen < num_stops(d)) {
        return;
    }
    /* Last STOP: calibration, then the next cycle or the interrupt. */
    d->cycle++;
    d->state = TDC_CALIBRATING;
    d->gen++; /* cancels the pending overflow event */
    sim_schedule_ps(sim_now_ps() + (sim_time_ps_t)llround(cal_time(d) * 1e12), complete_event, d,
                    d->gen);
}

void sim_tdc7200_attach_start(sim_tdc7200_t *dev, sim_pin_t *pin)
{
    sim_pin_listen(pin, on_start, dev);
}

void sim_tdc7200_attach_stop(sim_tdc7200_t *dev, sim_pin_t *pin)
{
    sim_pin_listen(pin, on_stop, dev);
}

/* ---------------------------------------------------------- ENABLE, SPI */

void sim_tdc7200_set_enable(void *handle, bool high)
{
    sim_tdc7200_t *d = handle;
    if (high == d->enabled) {
        return;
    }
    d->enabled = high;
    if (high) {
        d->enable_ps = sim_now_ps();
    }
    reset_regs(d); /* low: shut down, settings lost; high: defaults */
    sim_pin_set(&d->trigg, false);
    update_intb(d);
    sim_log(d->base.name, "state", "enable", high, NULL);
}

static void write_reg(sim_tdc7200_t *d, uint8_t a, uint8_t v)
{
    if (a > R_STOP_MASK_L) {
        return; /* results are read-only; unused addresses ignored */
    }
    if (a == R_INT_STATUS) {
        d->regs[a] &= ~(uint32_t)(v & 0x1F); /* write 1 to clear */
        update_intb(d);
        return;
    }
    d->regs[a] = v;
    if (a == R_CONFIG1 && (v & C1_START_MEAS)) {
        start_new_measurement(d);
    }
    if (a == R_INT_MASK) {
        update_intb(d);
    }
}

int sim_tdc7200_spi_transfer(void *handle, const uint8_t *tx, uint8_t *rx, size_t len)
{
    sim_tdc7200_t *d = handle;
    sim_fault_mode_t f = sim_device_begin(&d->base);
    if (f == SIM_FAULT_BUS_ERROR) {
        return -1;
    }
    sim_spi_idle_fill(d->cfg.miso, rx, len); /* DOUT is high-Z unless driven */
    if (f == SIM_FAULT_OFF || !d->enabled || len == 0) {
        return 0;
    }
    if ((double)(sim_now_ps() - d->enable_ps) / 1e12 < SPI_READY_S) {
        sim_log(d->base.name, "error", "spi_not_ready", 0, "SPI used < 100 us after ENABLE");
        return 0;
    }

    uint8_t cmd = tx[0];
    bool inc = (cmd & 0x80) != 0;
    bool write = (cmd & 0x40) != 0;
    uint8_t a = cmd & 0x3F;

    if (write) {
        for (size_t i = 1; i < len; i++) {
            write_reg(d, a, tx[i]);
            if (!inc) {
                break; /* bytes beyond the register are ignored */
            }
            a++;
        }
        sim_device_trace(&d->base, "write 0x%02X x%zu", cmd & 0x3F, len - 1);
        return 0;
    }

    if (a == R_INT_STATUS && (d->state == TDC_RUNNING || d->state == TDC_ARMED)) {
        sim_log(d->base.name, "error", "polled_int_status", 0,
                "data sheet 8.4.6: polling INT_STATUS interferes with the measurement; use INTB");
    }
    size_t i = 1;
    while (i < len) {
        if (a <= R_STOP_MASK_L) {
            rx[i++] = (uint8_t)d->regs[a];
        } else if (a >= R_TIME1 && a <= R_CAL2) {
            uint32_t v = d->regs[a];
            for (int b = 2; b >= 0 && i < len; b--) {
                rx[i++] = (uint8_t)(v >> (8 * b)); /* MSB first */
            }
        } else {
            break; /* nothing at this address */
        }
        if (!inc) {
            break;
        }
        a++;
    }
    if (f == SIM_FAULT_CORRUPT && len > 1) {
        sim_corrupt(rx + 1, len - 1);
    }
    sim_device_trace(&d->base, "read  0x%02X x%zu", cmd & 0x3F, len - 1);
    return 0;
}
