#include "sim_mcp79511.h"

#include <math.h>
#include <string.h>

#include "sim_log.h"

/* Instructions (data sheet Table 4-1). */
#define I_EEREAD 0x03
#define I_EEWRITE 0x02
#define I_EEWRDI 0x04
#define I_EEWREN 0x06
#define I_SRREAD 0x05
#define I_SRWRITE 0x01
#define I_READ 0x13
#define I_WRITE 0x12
#define I_UNLOCK 0x14
#define I_IDWRITE 0x32
#define I_IDREAD 0x33
#define I_CLRRAM 0x54

/* RTCC registers. */
#define R_HSEC 0x00
#define R_SEC 0x01
#define R_MIN 0x02
#define R_HOUR 0x03
#define R_WKDAY 0x04
#define R_DATE 0x05
#define R_MTH 0x06
#define R_YEAR 0x07
#define R_CONTROL 0x08
#define R_OSCTRIM 0x09

#define SEC_ST 0x80
#define HOUR_TRIMSIGN 0x80
#define HOUR_12 0x40
#define HOUR_PM 0x20
#define WKDAY_OSCRUN 0x20
#define WKDAY_PWRFAIL 0x10
#define MTH_LPYR 0x20
#define CTL_SQWEN 0x40
#define CTL_EXTOSC 0x08
#define CTL_CRSTRIM 0x04

#define ST_WIP 0x01
#define ST_WEL 0x02

#define F_OSC 32768.0
#define T_WC_S 5e-3   /* internal write cycle, max */
#define T_OSF_S 1e-3  /* oscillator timeout */

sim_mcp79511_config_t sim_mcp79511_default_config(sim_osc_t *input)
{
    sim_mcp79511_config_t c;
    memset(&c, 0, sizeof(c));
    c.input = input;
    c.input_present = true;
    /* Placeholder node address; the first three bytes are a Microchip OUI. */
    const uint8_t eui[6] = {0xD8, 0x80, 0x39, 0x00, 0x00, 0x01};
    memcpy(c.eui48, eui, 6);
    c.miso = SIM_MISO_PULL_UP;
    return c;
}

mcp79511_ctx_t sim_mcp79511_ctx(sim_mcp79511_t *dev)
{
    mcp79511_ctx_t ctx = {.spi_transfer = sim_mcp79511_spi_transfer, .handle = dev};
    return ctx;
}

/* ------------------------------------------------------------- BCD time */

static int bcd2(uint8_t v)
{
    return (v >> 4) * 10 + (v & 0x0F);
}

static uint8_t to_bcd(int v)
{
    return (uint8_t)(((v / 10) << 4) | (v % 10));
}

static bool leap(int yy)
{
    return yy % 4 == 0; /* valid 2001-2399 per the data sheet */
}

static int days_in(int month, int yy)
{
    static const int d[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    return month == 2 && leap(yy) ? 29 : d[(month - 1) % 12];
}

static void refresh_lpyr(sim_mcp79511_t *d);

/* One-second increment with the data sheet's carry rules (5.3.1). */
static void increment_second(sim_mcp79511_t *d)
{
    uint8_t *r = d->rtcc;
    int sec = bcd2(r[R_SEC] & 0x7F) + 1;
    if (sec < 60) {
        r[R_SEC] = (uint8_t)((r[R_SEC] & SEC_ST) | to_bcd(sec));
        return;
    }
    r[R_SEC] &= SEC_ST;
    int min = bcd2(r[R_MIN] & 0x7F) + 1;
    if (min < 60) {
        r[R_MIN] = to_bcd(min);
        return;
    }
    r[R_MIN] = 0;

    bool h12 = (r[R_HOUR] & HOUR_12) != 0;
    uint8_t keep = r[R_HOUR] & (HOUR_TRIMSIGN | HOUR_12);
    bool new_day = false;
    if (h12) {
        int h = bcd2(r[R_HOUR] & 0x1F);
        bool pm = (r[R_HOUR] & HOUR_PM) != 0;
        h = h % 12 + 1; /* 12 -> 1, 11 -> 12 */
        if (h == 12) {
            new_day = pm; /* 11:59:59 PM -> 12:00:00 AM */
            pm = !pm;
        }
        r[R_HOUR] = (uint8_t)(keep | (pm ? HOUR_PM : 0) | to_bcd(h));
    } else {
        int h = bcd2(r[R_HOUR] & 0x3F) + 1;
        new_day = h == 24;
        r[R_HOUR] = (uint8_t)(keep | to_bcd(new_day ? 0 : h));
    }
    if (!new_day) {
        return;
    }
    int wk = (r[R_WKDAY] & 7) + 1;
    r[R_WKDAY] = (uint8_t)((r[R_WKDAY] & ~7) | (wk > 7 ? 1 : wk));
    int yy = bcd2(r[R_YEAR]);
    int month = bcd2(r[R_MTH] & 0x1F);
    int date = bcd2(r[R_DATE] & 0x3F) + 1;
    if (date <= days_in(month, yy)) {
        r[R_DATE] = to_bcd(date);
        return;
    }
    r[R_DATE] = 0x01;
    month++;
    if (month > 12) {
        month = 1;
        yy = (yy + 1) % 100; /* 99 -> 00, no carry */
        r[R_YEAR] = to_bcd(yy);
    }
    r[R_MTH] = (uint8_t)((r[R_MTH] & 0x20) | to_bcd(month));
    refresh_lpyr(d);
}

static void refresh_lpyr(sim_mcp79511_t *d)
{
    d->rtcc[R_MTH] = (uint8_t)((d->rtcc[R_MTH] & 0x1F) | (leap(bcd2(d->rtcc[R_YEAR])) ? MTH_LPYR : 0));
}

/* -------------------------------------------------------- clock counting */

static double local_time(sim_mcp79511_t *d, double t)
{
    return d->cfg.input ? t + sim_osc_error_at(d->cfg.input, (sim_time_ps_t)llround(t * 1e12)) : t;
}

static double true_time_of_cycle(sim_mcp79511_t *d, double cycle)
{
    double local = d->start_local_s + cycle / F_OSC;
    return d->cfg.input ? sim_osc_true_time(d->cfg.input, local) : local;
}

/* Input cycles counted since the clock started. */
static double cycles_now(sim_mcp79511_t *d)
{
    return floor((local_time(d, sim_now_s()) - d->start_local_s) * F_OSC);
}

/* Trim applied to the second that starts now (+ = more cycles). */
static double trim_cycles(sim_mcp79511_t *d)
{
    int n = d->rtcc[R_OSCTRIM];
    if (n == 0) {
        return 0.0;
    }
    bool every_second = (d->rtcc[R_CONTROL] & CTL_CRSTRIM) != 0;
    /* UNVERIFIED: the once-a-minute trim lands on the second ending at :00. */
    bool this_one = every_second || bcd2(d->rtcc[R_SEC] & 0x7F) == 59;
    if (!this_one) {
        return 0.0;
    }
    /* TRIMSIGN 1 = add cycles: the counter fills sooner, the second is shorter. */
    double c = 2.0 * n;
    return (d->rtcc[R_HOUR] & HOUR_TRIMSIGN) ? -c : c;
}

static void tick_event(void *ctx, uint32_t tag);

static void schedule_tick(sim_mcp79511_t *d)
{
    double t = true_time_of_cycle(d, d->tick_cycle);
    sim_time_ps_t ps = (sim_time_ps_t)llround(t * 1e12);
    if (ps <= sim_now_ps()) {
        ps = sim_now_ps() + 1;
    }
    sim_schedule_ps(ps, tick_event, d, d->gen);
}

static bool square_wave_1hz(const sim_mcp79511_t *d)
{
    return (d->rtcc[R_CONTROL] & CTL_SQWEN) &&
           ((d->rtcc[R_CONTROL] & CTL_CRSTRIM) || (d->rtcc[R_CONTROL] & 0x03) == 0x00);
}

static void tick_event(void *ctx, uint32_t tag)
{
    sim_mcp79511_t *d = ctx;
    if (tag != d->gen || !d->running) {
        return;
    }
    sim_time_ps_t t = sim_event_time_ps();
    double length = F_OSC + trim_cycles(d); /* this coming second, before increment */
    increment_second(d);
    d->ticks++;
    d->last_tick_cycle = d->tick_cycle;
    d->tick_cycle += length;

    if (square_wave_1hz(d)) {
        sim_pin_set_exact(&d->mfp, true, t);
        double half = true_time_of_cycle(d, d->last_tick_cycle + length / 2.0);
        sim_pin_set_at(&d->mfp, false, (sim_time_ps_t)llround(half * 1e12));
        if (sim_log_is_open()) {
            double ts = (double)t / 1e12;
            sim_log(d->base.name, "truth", "pps_offset_s", ts - round(ts), "tick %llu",
                    (unsigned long long)d->ticks);
        }
    }
    schedule_tick(d);
}

static void oscrun_event(void *ctx, uint32_t tag)
{
    sim_mcp79511_t *d = ctx;
    if (tag != d->gen) {
        return;
    }
    if (d->running) {
        d->rtcc[R_WKDAY] |= WKDAY_OSCRUN;   /* after 32 clock cycles */
    } else {
        d->rtcc[R_WKDAY] &= (uint8_t)~WKDAY_OSCRUN; /* after T_OSF */
    }
}

/* Start or stop counting after anything that changes ST / EXTOSC / power. */
static void update_running(sim_mcp79511_t *d)
{
    bool ext = (d->rtcc[R_CONTROL] & CTL_EXTOSC) != 0;
    bool st = (d->rtcc[R_SEC] & SEC_ST) != 0;
    if (st && !ext) {
        sim_log(d->base.name, "error", "st_with_external_clock", 0,
                "board feeds a clock on X1: set EXTOSC, not ST");
    }
    bool run = d->base.powered && ext && d->cfg.input_present;
    if (run == d->running) {
        return;
    }
    d->running = run;
    d->gen++;
    if (run) {
        d->start_local_s = local_time(d, sim_now_s());
        d->last_tick_cycle = 0.0;
        d->tick_cycle = F_OSC + trim_cycles(d); /* UNVERIFIED: first tick 1 s later */
        schedule_tick(d);
        sim_schedule_ps(sim_now_ps() + (sim_time_ps_t)llround(32.0 / F_OSC * 1e12), oscrun_event,
                        d, d->gen);
    } else {
        sim_schedule_ps(sim_now_ps() + (sim_time_ps_t)llround(T_OSF_S * 1e12), oscrun_event, d,
                        d->gen);
    }
    sim_log(d->base.name, "state", "oscillator", run, NULL);
}

static uint8_t read_hsec(sim_mcp79511_t *d)
{
    if (!d->running) {
        return d->rtcc[R_HSEC];
    }
    /* 4.096 kHz: 24 hundredths of 41 clocks then one of 40 = 250 ms (5.3.2). */
    double since = cycles_now(d) - d->last_tick_cycle;
    int t4096 = (int)(since / 8.0);
    if (t4096 < 0) {
        t4096 = 0;
    }
    int quarter = t4096 / 1024;
    int q = t4096 % 1024;
    int inc = q < 984 ? q / 41 : 24;
    int h = quarter * 25 + inc;
    return to_bcd(h > 99 ? 99 : h);
}

/* -------------------------------------------------------------- power */

static void por(sim_mcp79511_t *d)
{
    memset(d->rtcc, 0, sizeof(d->rtcc));
    d->rtcc[R_WKDAY] = 0x01;
    d->rtcc[R_DATE] = 0x01;
    d->rtcc[R_MTH] = 0x01;
    for (size_t i = 0; i < sizeof(d->sram); i++) {
        d->sram[i] = (uint8_t)sim_rand_u32(); /* SRAM is not initialised by POR */
    }
    d->status &= 0x0C; /* BP bits are nonvolatile, WEL / WIP cleared */
    d->unlock_step = 0;
    d->running = false;
    d->gen++;
}

void sim_mcp79511_init(sim_mcp79511_t *dev, const char *name, const sim_mcp79511_config_t *cfg)
{
    memset(dev, 0, sizeof(*dev));
    sim_device_init(&dev->base, name);
    dev->cfg = *cfg;
    sim_pin_init(&dev->mfp, name, true);
    memset(dev->eeprom, 0xFF, sizeof(dev->eeprom));
    memset(dev->id, 0xFF, sizeof(dev->id));
    memcpy(dev->id + 2, cfg->eui48, 6);
    por(dev);
}

void sim_mcp79511_power(sim_mcp79511_t *dev, bool on)
{
    if (on == dev->base.powered) {
        return;
    }
    sim_device_power(&dev->base, on);
    if (on) {
        por(dev); /* no VBAT modelled: every power-up is a POR */
    }
    update_running(dev);
}

/* ---------------------------------------------------------------- SPI */

static void write_rtcc(sim_mcp79511_t *d, uint8_t a, uint8_t v)
{
    switch (a) {
    case R_WKDAY: {
        uint8_t keep = d->rtcc[a] & WKDAY_OSCRUN; /* read-only */
        uint8_t pwrfail = (d->rtcc[a] & WKDAY_PWRFAIL) & v; /* can only be cleared */
        d->rtcc[a] = (uint8_t)(keep | pwrfail | (v & 0x0F));
        break;
    }
    case R_MTH:
        d->rtcc[a] = v & 0x1F;
        refresh_lpyr(d);
        break;
    case R_YEAR:
        d->rtcc[a] = v;
        refresh_lpyr(d);
        break;
    case R_CONTROL:
        d->rtcc[a] = v & 0x7F;
        if ((v & CTL_SQWEN) && !(v & CTL_CRSTRIM) && (v & 0x03) != 0) {
            sim_log(d->base.name, "error", "unmodelled", v & 0x03,
                    "only the 1 Hz square wave is simulated");
        }
        if ((v & 0x30) != 0) {
            sim_log(d->base.name, "error", "unmodelled", v, "alarms are not simulated");
        }
        if (!(v & CTL_SQWEN)) {
            sim_pin_set(&d->mfp, true);
        }
        break;
    default:
        d->rtcc[a] = v;
        break;
    }
    if (a == R_SEC || a == R_CONTROL) {
        update_running(d);
    }
}

static uint8_t read_rtcc(sim_mcp79511_t *d, uint8_t a)
{
    if (a == R_HSEC) {
        return read_hsec(d);
    }
    if (a == R_CONTROL) {
        return (uint8_t)(d->rtcc[a] | 0x80); /* bit 7 reads as 1 */
    }
    return d->rtcc[a];
}

static bool wip(sim_mcp79511_t *d)
{
    if (sim_now_ps() >= d->wip_until_ps) {
        d->status &= (uint8_t)~ST_WIP;
    }
    return (d->status & ST_WIP) != 0;
}

static void begin_write_cycle(sim_mcp79511_t *d)
{
    d->status = (uint8_t)((d->status | ST_WIP) & ~ST_WEL);
    d->wip_until_ps = sim_now_ps() + (sim_time_ps_t)llround(T_WC_S * 1e12);
}

static bool protected_addr(const sim_mcp79511_t *d, uint8_t a)
{
    switch ((d->status >> 2) & 3) {
    case 1: return a >= 0x60;
    case 2: return a >= 0x40;
    case 3: return true;
    default: return false;
    }
}

int sim_mcp79511_spi_transfer(void *handle, const uint8_t *tx, uint8_t *rx, size_t len)
{
    sim_mcp79511_t *d = handle;
    sim_fault_mode_t f = sim_device_begin(&d->base);
    if (f == SIM_FAULT_BUS_ERROR) {
        return -1;
    }
    sim_spi_idle_fill(d->cfg.miso, rx, len); /* SO is high-Z unless shifting data out */
    if (f == SIM_FAULT_OFF || len == 0) {
        return 0;
    }
    uint8_t ins = tx[0];
    bool busy = wip(d);

    switch (ins) {
    case I_READ:
    case I_WRITE: {
        if (len < 2) {
            break;
        }
        uint8_t a = tx[1];
        if (a >= 0x60) {
            break; /* unimplemented: device does not respond */
        }
        bool rtcc = a < 0x20;
        for (size_t i = 2; i < len; i++) {
            if (ins == I_READ) {
                rx[i] = rtcc ? read_rtcc(d, a) : d->sram[a - 0x20];
            } else if (rtcc) {
                write_rtcc(d, a, tx[i]);
            } else {
                d->sram[a - 0x20] = tx[i];
            }
            a++;
            if (rtcc && a == 0x20) {
                a = 0x00; /* each block wraps on itself */
            } else if (!rtcc && a == 0x60) {
                a = 0x20;
            }
        }
        break;
    }
    case I_CLRRAM:
        memset(d->sram, 0, sizeof(d->sram));
        break;
    case I_SRREAD:
        if (len >= 2) {
            rx[1] = (uint8_t)(d->status & 0x0F);
        }
        break;
    case I_EEWREN:
        if (!busy) {
            d->status |= ST_WEL;
        }
        break;
    case I_EEWRDI:
        d->status &= (uint8_t)~ST_WEL;
        break;
    case I_SRWRITE:
        if (!busy && (d->status & ST_WEL) && len >= 2) {
            d->status = (uint8_t)((d->status & ~0x0C) | (tx[1] & 0x0C));
            begin_write_cycle(d);
        }
        break;
    case I_EEREAD:
        if (!busy && len >= 2) {
            uint8_t a = tx[1] & 0x7F; /* 80h-FFh map back to 00h-7Fh */
            for (size_t i = 2; i < len; i++) {
                rx[i] = d->eeprom[a];
                a = (uint8_t)((a + 1) & 0x7F);
            }
        }
        break;
    case I_EEWRITE:
        if (busy) {
            sim_log(d->base.name, "error", "eeprom_busy", 0, "EEWRITE during write cycle");
            break;
        }
        if (!(d->status & ST_WEL) || len < 3) {
            sim_log(d->base.name, "error", "eeprom_not_enabled", 0, "EEWRITE without EEWREN");
            break;
        }
        {
            uint8_t a = tx[1] & 0x7F;
            uint8_t page = a & 0x78;
            for (size_t i = 2; i < len; i++) {
                if (!protected_addr(d, a)) {
                    d->eeprom[a] = tx[i];
                }
                a = (uint8_t)(page | ((a + 1) & 0x07)); /* 8-byte page wraps */
            }
            begin_write_cycle(d);
        }
        break;
    case I_UNLOCK:
        if (len >= 2 && (d->status & ST_WEL)) {
            if (d->unlock_step == 0 && tx[1] == 0x55) {
                d->unlock_step = 1;
            } else if (d->unlock_step == 1 && tx[1] == 0xAA) {
                d->unlock_step = 2;
            } else {
                d->unlock_step = 0;
                d->status &= (uint8_t)~ST_WEL; /* sequence not followed: WEL reset */
            }
        }
        break;
    case I_IDWRITE:
        if (!busy && d->unlock_step == 2 && (d->status & ST_WEL) && len >= 3) {
            uint8_t a = tx[1] & 0x0F;
            uint8_t page = a & 0x08;
            for (size_t i = 2; i < len; i++) {
                d->id[a] = tx[i];
                a = (uint8_t)(page | ((a + 1) & 0x07));
            }
            begin_write_cycle(d);
        } else {
            sim_log(d->base.name, "error", "id_locked", 0, "IDWRITE without EEWREN + UNLOCK 55h, AAh");
            d->status &= (uint8_t)~ST_WEL;
        }
        d->unlock_step = 0; /* locked again after each write */
        break;
    case I_IDREAD:
        if (len >= 2) {
            uint8_t a = tx[1] & 0x0F;
            for (size_t i = 2; i < len; i++) {
                rx[i] = d->id[a];
                a = (uint8_t)((a + 1) & 0x0F);
            }
        }
        break;
    default:
        sim_log(d->base.name, "error", "bad_instruction", ins, NULL);
        break;
    }
    if (f == SIM_FAULT_CORRUPT && len > 2) {
        sim_corrupt(rx + 2, len - 2);
    }
    sim_device_trace(&d->base, "instr 0x%02X x%zu", ins, len);
    return 0;
}
