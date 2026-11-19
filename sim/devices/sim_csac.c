#include "sim_csac.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_log.h"

#define MODE_ANALOG 0x0001
#define MODE_PHASE  0x0004
#define MODE_AUTOSYNC 0x0008
#define MODE_DISC   0x0010
#define MODE_ULP    0x0020
#define MODE_CKSUM  0x0040
#define MODE_PPS_MASK (MODE_PHASE | MODE_AUTOSYNC | MODE_DISC)

#define BITE_HOLD_S 5.0
#define SYNC_TIMEOUT_S 3.0

static const char *HEADERS =
    "Status,Alarm,SN,Mode,Contrast,LaserI,TCXO,HeatP,Sig,Temp,Steer,ATune,Phase,DiscOK,TOD,"
    "LTime,Ver";

sim_csac_config_t sim_csac_default_config(uint64_t seed)
{
    sim_csac_config_t c;
    memset(&c, 0, sizeof(c));
    c.serial = "2611CS00001";
    c.firmware = "1.09";
    c.warmup_s = 180.0;       /* guide 2.2 "after 3 minutes"; data sheet <180 s */
    c.reply_delay_s = 0.005;
    c.mode = 0x0000;
    c.temperature_c = sim_signal_const(35.0);

    /*
     * Free-running TCXO before lock, user guide 2.4: spec +-20e-6, but "typically
     * <1e-8 during acquisition" (last good tuning voltage restored), and
     * temperature sensitivity ~ +-30e-9 / degC.
     */
    double sign = sim_hash_uniform(seed, 3) < 0.5 ? -1.0 : 1.0;
    c.clock.free_run.y0 = 2e-8 * (sim_hash_uniform(seed, 1) - 0.5);
    c.clock.free_run.tempco = sign * 30e-9;
    c.clock.free_run.t_ref_c = 25.0;
    c.clock.free_run.adev_1s = 1e-9;      /* UNVERIFIED: not in guide or data sheet */
    c.clock.free_run.seed = seed;

    /*
     * Locked, SA.45s data sheet (900-00744, Space CSAC):
     *   ADEV 3e-10 @1 s, 1e-10 @10 s, 3e-11 @100 s, 1e-11 @1000 s: pure white FM,
     *     so random-walk FM is kept small enough to stay under the 1000 s point
     *   aging <9e-10 / month (<1e-8 / year)
     *   +-5e-10 maximum change over -10..70 C: tempco drawn within +-5e-10 / 45 C
     *   offset at shipment +-5e-11, retrace +-5e-10 (48 h off)
     *   acceleration sensitivity: NOT SPECIFIED (only "maintains lock" at 7.7 grms)
     *     -> 0 until we have a number; set g_sens to study it
     */
    c.clock.locked.y0 = 5e-11 * (2.0 * sim_hash_uniform(seed, 2) - 1.0);
    c.clock.locked.adev_1s = 3e-10;
    c.clock.locked.aging_per_day = 9e-10 / 30.0;
    c.clock.locked.rw_per_sqrt_s = 1e-13;
    c.clock.locked.tempco = (5e-10 / 45.0) * (2.0 * sim_hash_uniform(seed, 4) - 1.0);
    c.clock.locked.t_ref_c = 25.0;
    c.clock.locked.seed = seed + 1;
    c.clock.retrace = 5e-10;
    c.clock.steer_range = 1e-6;           /* data sheet: digital tuning +-1e-6, 1e-12 resolution */

    c.clock.rf_hz = 10e6;             /* 2.8: 1PPS snaps to the nearest RF edge */
    c.clock.meter_res_s = 450e-12;    /* 2.9: high-resolution phase meter */
    c.clock.meter_fine_range_s = 1e-6;/* 2.13: beyond ~+-1 us, extended-range meter */
    c.clock.meter_coarse_res_s = 100e-9;
    c.clock.steer_res = 1e-12;        /* 2.6: hardware steering resolution */
    c.clock.disc_limit = 1e-6;        /* 2.9: no software limit; hardware range +-1e-6 */
    c.clock.jam_limit_s = 1e-6;       /* 2.9: >1 us after an outage -> sync */
    c.clock.jam_restarts_servo = false;
    c.clock.pps_width_ps = 100ULL * 1000000ULL; /* 100 us */
    c.bite_when_off = false;
    return c;
}

/* ------------------------------------------------------------- replies */

static uint8_t xor_sum(const char *s, size_t n)
{
    uint8_t x = 0;
    for (size_t i = 0; i < n; i++) {
        x ^= (uint8_t)s[i];
    }
    return x;
}

/* Send one or more lines ("a\nb") as CRLF lines, with checksum if enabled. */
static void reply_at(sim_csac_t *d, sim_time_ns_t t, const char *text)
{
    char out[512];
    size_t n = 0;
    const char *p = text;
    while (*p && n < sizeof(out) - 16) {
        const char *e = strchr(p, '\n');
        size_t len = e ? (size_t)(e - p) : strlen(p);
        if (len > sizeof(out) - 16 - n) {
            len = sizeof(out) - 16 - n;
        }
        memcpy(out + n, p, len);
        n += len;
        if (d->mode & MODE_CKSUM) {
            n += (size_t)snprintf(out + n, sizeof(out) - n, "*%02X", xor_sum(p, len));
        }
        out[n++] = '\r';
        out[n++] = '\n';
        p += len + (e ? 1 : 0);
        if (!e) {
            break;
        }
    }
    sim_uart_send_at(&d->uart, t, (const uint8_t *)out, n);
}

static void reply(sim_csac_t *d, const char *text)
{
    reply_at(d, sim_now_ns() + (sim_time_ns_t)(d->cfg.reply_delay_s * 1e9), text);
}

/* ---------------------------------------------------------------- state */

static void apply_mode(sim_csac_t *d)
{
    sim_ref_mode_t m = SIM_REF_OFF;
    if (d->mode & MODE_DISC) {
        m = SIM_REF_DISCIPLINE;
    } else if (d->mode & MODE_PHASE) {
        m = SIM_REF_MEASURE;
    } else if (d->mode & MODE_AUTOSYNC) {
        m = SIM_REF_AUTOSYNC;
    }
    sim_clock_set_mode(&d->clk, m);
    d->clk.tau_s = d->tau_s;
    d->clk.cable_s = d->cable_100ps * 100e-12;
    d->clk.threshold_s = d->threshold_ns * 1e-9;
}

static void set_status(sim_csac_t *d, int status)
{
    if (status != d->status) {
        sim_log(d->base.name, "state", "status", status, NULL);
    }
    d->status = status;
}

static void bite_low_event(void *ctx, uint32_t tag)
{
    sim_csac_t *d = ctx;
    if (tag == d->gen && d->status == 0) {
        sim_pin_set(&d->bite, false);
        sim_log(d->base.name, "state", "bite", 0, "locked");
    }
}

static void status_event(void *ctx, uint32_t tag);

static void schedule_status(sim_csac_t *d, double step_s)
{
    d->step_s = step_s;
    sim_schedule_ps(sim_now_ps() + (sim_time_ps_t)(step_s * 1e12), status_event, d, d->gen);
}

static void status_event(void *ctx, uint32_t tag)
{
    sim_csac_t *d = ctx;
    if (tag != d->gen || !d->base.powered || d->status <= 0) {
        return;
    }
    set_status(d, d->status - 1);
    if (d->status > 0) {
        schedule_status(d, d->step_s);
        return;
    }
    d->alarm = 0;
    d->lock_ps = sim_now_ps();
    sim_clock_set_locked(&d->clk, true);
    sim_schedule_ps(sim_now_ps() + (sim_time_ps_t)(BITE_HOLD_S * 1e12), bite_low_event, d, d->gen);
}

static void bite_high_event(void *ctx, uint32_t tag)
{
    sim_csac_t *d = ctx;
    if (tag == d->gen && d->status != 0) {
        sim_pin_set(&d->bite, true);
        sim_log(d->base.name, "state", "bite", 1, "unlocked");
    }
}

void sim_csac_lose_lock(sim_csac_t *dev, double relock_s)
{
    if (!dev->base.powered) {
        return;
    }
    dev->gen++;
    set_status(dev, 3);
    dev->alarm = 0x0001; /* signal contrast low */
    sim_clock_set_locked(&dev->clk, false);
    /* "BITE provides an immediate (within 1 second) indication of lock failure" */
    sim_schedule_ps(sim_now_ps() + 500ULL * 1000000000ULL, bite_high_event, dev, dev->gen);
    schedule_status(dev, relock_s / 3.0);
}

static void on_own_pps(void *ctx, sim_time_ps_t t)
{
    sim_csac_t *d = ctx;
    (void)t;
    d->tod++;
    sim_clock_second(&d->clk);
    if (d->tod_query) {
        d->tod_query = false;
        char buf[16];
        snprintf(buf, sizeof(buf), "%u", d->tod);
        reply_at(d, sim_now_ns() + SIM_MS(5), buf); /* within 20 ms of the edge */
    }
    if (d->sync_reply && d->clk.sync_result != 0) {
        d->sync_reply = false;
        reply(d, d->clk.sync_result > 0 ? "S" : "E");
    }
}

static void on_ref(void *ctx, bool rising, sim_time_ps_t t)
{
    sim_csac_t *d = ctx;
    (void)t;
    if (rising && d->sync_reply && d->clk.sync_result > 0) {
        d->sync_reply = false;
        reply(d, "S");
    }
}

/* ------------------------------------------------------------ commands */

static void telemetry(sim_csac_t *d, char *out, size_t n)
{
    sim_clock_t *c = &d->clk;
    char phase[16] = "---";
    char disc[8] = "---";
    if (d->mode & (MODE_PHASE | MODE_DISC)) {
        if (!sim_clock_ref_present(c) || !c->phase_valid) {
            snprintf(phase, sizeof(phase), "NEEDREFPPS");
        } else {
            snprintf(phase, sizeof(phase), "%ld", lround(c->phase_s * 1e9));
        }
    }
    if (d->mode & MODE_DISC) {
        snprintf(disc, sizeof(disc), "%d", (int)c->disc);
    }
    double steer_e12 = (c->steer_user + c->steer_disc) * 1e12;
    unsigned contrast = d->status == 0 ? 4381u + (unsigned)(sim_rand_u32() % 40u)
                                       : (unsigned)(8 - d->status) * 150u;
    double heat = d->status > 6 ? 35.0 : 17.62;
    uint32_t ltime =
        d->status == 0 ? (uint32_t)((sim_now_ps() - d->lock_ps) / SIM_PS_PER_S) : 0;
    snprintf(out, n, "%d,0x%04X,%s,0x%04X,%u,%.2f,%.3f,%.2f,%.3f,%.2f,%ld,---,%s,%s,%u,%u,%s",
             d->status, d->alarm, d->cfg.serial, d->mode, contrast, 0.86, 1.573, heat, 0.996,
             sim_signal_eval(&d->cfg.temperature_c, sim_now_ns()), lround(steer_e12), phase, disc,
             d->tod, ltime, d->cfg.firmware);
}

static bool parse_long(const char *s, long *v)
{
    char *end;
    if (*s == '\0') {
        return false;
    }
    *v = strtol(s, &end, 10);
    return *end == '\0';
}

static void steer_reply(sim_csac_t *d)
{
    char buf[48];
    snprintf(buf, sizeof(buf), "Steer = %ld",
             lround((d->clk.steer_user + d->clk.steer_disc) * 1e12));
    reply(d, buf);
}

static void set_user_steer(sim_csac_t *d, int64_t e15)
{
    d->steer_e15 = e15;
    sim_clock_set_user_steer(&d->clk, (double)e15 * 1e-15);
}

static void execute(sim_csac_t *d, const char *cmd)
{
    char buf[200];
    long v;
    sim_log(d->base.name, "cmd", cmd, 0, NULL);

    if (strcmp(cmd, "6") == 0) {
        reply(d, HEADERS);
    } else if (strcmp(cmd, "^") == 0) {
        telemetry(d, buf, sizeof(buf));
        reply(d, buf);
    } else if (strcmp(cmd, "F?") == 0) {
        steer_reply(d);
    } else if (strncmp(cmd, "FA", 2) == 0 && parse_long(cmd + 2, &v)) {
        /* 2.6: "If a larger value is sent, the maximum allowed steer is applied." */
        set_user_steer(d, v > 2000000000L ? 2000000000L : v < -2000000000L ? -2000000000L : v);
        steer_reply(d);
    } else if (strncmp(cmd, "FD", 2) == 0 && parse_long(cmd + 2, &v)) {
        set_user_steer(d, d->steer_e15 + (v > 20000000L ? 20000000L : v < -20000000L ? -20000000L : v));
        steer_reply(d);
    } else if (strcmp(cmd, "FL") == 0 && d->status == 0) {
        sim_clock_latch(&d->clk);
        d->steer_e15 = 0;
        reply(d, "Steer Latched \nSteer = 0");
    } else if (strcmp(cmd, "M?") == 0) {
        snprintf(buf, sizeof(buf), "0x%04X", d->mode);
        reply(d, buf);
    } else if (cmd[0] == 'M' && strlen(cmd) == 2 && strchr("AaMmSsDdUuCc", cmd[1])) {
        static const char *on = "AMSDUC";
        static const uint16_t bit[] = {MODE_ANALOG, MODE_PHASE, MODE_AUTOSYNC,
                                       MODE_DISC,   MODE_ULP,   MODE_CKSUM};
        char c = cmd[1];
        int i = (int)(strchr(on, toupper((unsigned char)c)) - on);
        if (isupper((unsigned char)c)) {
            if (bit[i] & MODE_PPS_MASK) {
                d->mode &= (uint16_t)~MODE_PPS_MASK; /* mutually exclusive */
            }
            d->mode |= bit[i];
        } else {
            d->mode &= (uint16_t)~bit[i];
        }
        apply_mode(d);
        snprintf(buf, sizeof(buf), "0x%04X", d->mode);
        reply(d, buf);
    } else if (strcmp(cmd, "S") == 0) {
        sim_clock_request_sync(&d->clk, SYNC_TIMEOUT_S);
        d->sync_reply = true;
    } else if (strcmp(cmd, "D?") == 0) {
        snprintf(buf, sizeof(buf), "%d", d->tau_s);
        reply(d, buf);
    } else if (strcmp(cmd, "DC?") == 0) {
        snprintf(buf, sizeof(buf), "%d", d->cable_100ps);
        reply(d, buf);
    } else if (strcmp(cmd, "DCL") == 0) {
        reply(d, "Phase comp latched");
    } else if (strncmp(cmd, "DC", 2) == 0 && parse_long(cmd + 2, &v) && labs(v) <= 1000) {
        d->cable_100ps = (int32_t)v;
        apply_mode(d);
        snprintf(buf, sizeof(buf), "%ld", v);
        reply(d, buf);
    } else if (cmd[0] == 'D' && parse_long(cmd + 1, &v) && v >= 10 && v <= 10000) {
        d->tau_s = (int32_t)v;
        apply_mode(d);
        snprintf(buf, sizeof(buf), "%ld", v);
        reply(d, buf);
    } else if (strcmp(cmd, "U?") == 0) {
        snprintf(buf, sizeof(buf), "%u,%u", d->ulp_sleep, d->ulp_wake);
        reply(d, buf);
    } else if (cmd[0] == 'U' && strchr(cmd, ',')) {
        unsigned long s = strtoul(cmd + 1, NULL, 10);
        unsigned long w = strtoul(strchr(cmd, ',') + 1, NULL, 10);
        if (s < 1800 || s > 65535 || w < 10 || w > 65535) {
            reply(d, "?");
            return;
        }
        d->ulp_sleep = (uint32_t)s;
        d->ulp_wake = (uint32_t)w;
        snprintf(buf, sizeof(buf), "%lu,%lu", s, w);
        reply(d, buf);
    } else if (strcmp(cmd, "T?") == 0) {
        d->tod_query = true; /* answered right after the next 1PPS */
    } else if ((strncmp(cmd, "TA", 2) == 0 || strncmp(cmd, "TD", 2) == 0) &&
               parse_long(cmd + 2, &v)) {
        d->tod = cmd[1] == 'A' ? (uint32_t)v : (uint32_t)((long)d->tod + v);
        snprintf(buf, sizeof(buf), "TimeOfDay = %u", d->tod);
        reply(d, buf);
    } else if (strcmp(cmd, "m?") == 0) {
        snprintf(buf, sizeof(buf), "%d", d->threshold_ns);
        reply(d, buf);
    } else if (cmd[0] == 'm' && parse_long(cmd + 1, &v) && v >= 1 && v <= 1000000000L) {
        d->threshold_ns = (int32_t)v;
        apply_mode(d);
        snprintf(buf, sizeof(buf), "%ld", v);
        reply(d, buf);
    } else if (cmd[0] == '>' && (strcmp(cmd, ">?") == 0 ||
                                 (parse_long(cmd + 1, &v) && v >= 1 && v <= 4))) {
        if (cmd[1] != '?') {
            d->pulse_mult = (int32_t)v;
            d->clk.pps.width_ps = (sim_time_ps_t)v * 100ULL * 1000000ULL;
        }
        snprintf(buf, sizeof(buf), "PPS Pulse Width = %d times ~100 usec", d->pulse_mult);
        reply(d, buf);
    } else {
        d->bad_commands++;
        sim_log(d->base.name, "error", "bad_command", 0, "%s", cmd);
        reply(d, "?");
    }
}

/* Handle a complete `!...` body (without ! and CRLF). */
static void finish_line(sim_csac_t *d)
{
    d->line[d->line_len] = '\0';
    char *body = d->line;
    if (d->mode & MODE_CKSUM) {
        char *star = strrchr(body, '*');
        if (!star || strlen(star + 1) != 2 ||
            strtoul(star + 1, NULL, 16) != xor_sum(body, (size_t)(star - body))) {
            sim_log(d->base.name, "error", "bad_checksum", 0, "%s", body);
            sim_uart_send_at(&d->uart, sim_now_ns() + SIM_MS(5), (const uint8_t *)"*\r\n", 3);
            return;
        }
        *star = '\0';
    }
    execute(d, body);
}

static void on_rx(sim_uart_t *u, const uint8_t *buf, size_t len, void *user)
{
    (void)u;
    sim_csac_t *d = user;
    if (d->status < 0) {
        return;
    }
    for (size_t i = 0; i < len; i++) {
        char c = (char)buf[i];
        if (!d->in_cmd) {
            if (c == '!') {
                d->in_cmd = true;
                d->line_len = 0;
            } else if (!(d->mode & MODE_CKSUM) && strchr("6^FMSDUTm", c) && c != '\0') {
                static const char *full[] = {"6", "^", "F?", "M?", "S", "D?", "U?", "T?", "m?"};
                execute(d, full[strchr("6^FMSDUTm", c) - "6^FMSDUTm"]);
            } else if (c != '\r' && c != '\n' && c != ' ') {
                reply(d, "?");
            }
            continue;
        }
        if (c == 0x1B) {
            d->in_cmd = false; /* ESC aborts */
        } else if (c == '\n') {
            if (d->line_len > 0 && d->line[d->line_len - 1] == '\r') {
                d->line_len--;
            }
            d->in_cmd = false;
            finish_line(d);
        } else if (d->line_len < sizeof(d->line) - 1) {
            d->line[d->line_len++] = c;
        }
    }
}

/* ---------------------------------------------------------------- setup */

void sim_csac_init(sim_csac_t *dev, const char *name, const sim_csac_config_t *cfg)
{
    memset(dev, 0, sizeof(*dev));
    sim_device_init(&dev->base, name);
    dev->cfg = *cfg;
    dev->base.powered = false;
    sim_uart_init(&dev->uart, name, 57600);
    dev->uart.on_rx = on_rx;
    dev->uart.user = dev;
    dev->uart.base.powered = false;
    sim_pin_init(&dev->bite, name, cfg->bite_when_off);
    sim_clock_init(&dev->clk, name, &cfg->clock);
    dev->clk.pps.on_edge = on_own_pps;
    dev->clk.pps.ctx = dev;

    dev->mode = cfg->mode;
    dev->tau_s = 100;
    dev->threshold_ns = 20;
    dev->pulse_mult = 1;
    dev->ulp_sleep = 3300;
    dev->ulp_wake = 300;
    dev->status = -1;
}

void sim_csac_attach_ref(sim_csac_t *dev, sim_pin_t *ref_pps)
{
    sim_clock_attach_ref(&dev->clk, ref_pps);
    sim_pin_listen(ref_pps, on_ref, dev); /* after the clock: sees the sync result */
}

void sim_csac_power(sim_csac_t *dev, bool on)
{
    if (on == dev->base.powered) {
        return;
    }
    sim_device_power(&dev->base, on);
    sim_device_power(&dev->uart.base, on);
    sim_clock_power(&dev->clk, on);
    dev->gen++;
    if (!on) {
        dev->status = -1;
        dev->in_cmd = false;
        dev->tod_query = dev->sync_reply = false;
        sim_pin_set(&dev->bite, dev->cfg.bite_when_off);
        return;
    }
    /* Volatile state restarts; the mode register and settings persist. */
    dev->steer_e15 = 0;
    dev->tod = 0;
    dev->alarm = 0;
    dev->power_on_ps = sim_now_ps();
    set_status(dev, 8);
    sim_pin_set(&dev->bite, true);
    apply_mode(dev);
    schedule_status(dev, dev->cfg.warmup_s / 8.0);
}
