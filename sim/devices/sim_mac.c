#include "sim_mac.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim_log.h"

#define ALM_ACQ_FAILED   (1u << 3)
#define ALM_TEMPERATURE  (1u << 16)
#define ALM_NO_PPS       (1u << 17)
#define ALM_DISC_RANGE   (1u << 18)

/* Guide 2.2: the baseplate rises ~15 C during start-up; the Temperature
 * parameter reads ~10-15 C above the baseplate; above +75 C baseplate the
 * heater shuts down and the unit "eventually loses lock". */
#define BASEPLATE_RISE_C   15.0
#define PCB_ABOVE_PLATE_C  12.5
#define MAX_BASEPLATE_C    75.0
#define OVERHEAT_UNLOCK_S  60.0 /* UNVERIFIED: "eventually" */

#define BSL_WAIT_S 3.0

/* ------------------------------------------------------------ parameters */

typedef enum { P_INT, P_FLOAT } ptype_t;

typedef struct {
    uint16_t id;
    const char *name;
    bool ro;
    ptype_t type;
    int64_t min, max;
} param_t;

static const param_t PARAMS[SIM_MAC_NPARAMS] = {
    {256, "Alarms", true, P_INT, 0, 0},
    {257, "PpsInDetected", true, P_INT, 0, 1},
    {258, "TotalRuntime", true, P_INT, 0, 0},
    {259, "TotalLocktime", true, P_INT, 0, 0},
    {263, "Locked", true, P_INT, 0, 1},
    {264, "TimeOfDay", false, P_INT, 0, 2147483647LL},
    {265, "DisciplineLocked", true, P_INT, 0, 1},
    {512, "PpsOffset", false, P_INT, -83886080LL, 83886080LL},
    {513, "PpsWidth", false, P_INT, 0, 83886080LL},
    {515, "CableDelay", false, P_INT, -500000000LL, 500000000LL},
    {768, "Disciplining", false, P_INT, 0, 1},
    {769, "PpsSource", false, P_INT, 0, 1},
    {770, "TauPps0", false, P_INT, 10, 45000},
    {771, "PpsQErr", false, P_INT, -1000000LL, 1000000LL},
    {772, "PhaseLimit", false, P_INT, -1000000LL, 1000000LL},
    {773, "JamSyncing", true, P_INT, 0, 1},
    {774, "Phase", true, P_FLOAT, 0, 0},
    {775, "LastCorrection", true, P_INT, 0, 0},
    {777, "TauPps1", false, P_INT, 10, 45000},
    {778, "PhaseMetering", false, P_INT, 0, 1},
    {779, "DisciplineThresholdPps0", false, P_INT, 1, 1000},
    {780, "DisciplineThresholdPps1", false, P_INT, 1, 1000},
    {1288, "LaserTempSet", true, P_INT, 0, 0},
    {1289, "OscTuning", true, P_INT, 0, 0},
    {1291, "OvenCurrent", true, P_INT, 0, 0},
    {1292, "DCSignal", true, P_INT, 0, 0},
    {1293, "AnalogTuning", true, P_INT, 0, 0},
    {1296, "Temperature", true, P_INT, 0, 0},
    {1300, "DigitalTuning", false, P_INT, -20000000LL, 20000000LL},
    {1306, "PowerSupply", true, P_INT, 0, 0},
    {1312, "AnalogTuningEnabled", false, P_INT, 0, 1},
    {1321, "EffectiveTuning", true, P_INT, 0, 0},
    {1332, "LockProgress", true, P_INT, 0, 100},
};
/* DisciplineTuning (3142) is handled separately to keep the table aligned. */
#define ID_DISC_TUNING 3142

static int find_param(const char *key)
{
    char *end;
    long id = strtol(key, &end, 10);
    for (int i = 0; i < SIM_MAC_NPARAMS; i++) {
        if ((*end == '\0' && PARAMS[i].id == id) || strcmp(PARAMS[i].name, key) == 0) {
            return i;
        }
    }
    if ((*end == '\0' && id == ID_DISC_TUNING) || strcmp(key, "DisciplineTuning") == 0) {
        return SIM_MAC_NPARAMS; /* pseudo index */
    }
    return -1;
}

static bool locked(const sim_mac_t *d)
{
    return d->clk.locked;
}

double sim_mac_baseplate_c(const sim_mac_t *d)
{
    double since_on = (double)(sim_now_ps() - d->power_on_ps) / 1e12;
    double rise = since_on < d->cfg.warmup_s ? BASEPLATE_RISE_C * since_on / d->cfg.warmup_s
                                             : BASEPLATE_RISE_C;
    return sim_signal_eval_clean(&d->cfg.temperature_c, sim_now_ns()) + rise;
}

static double param_value(sim_mac_t *d, int i)
{
    sim_clock_t *c = &d->clk;
    if (i == SIM_MAC_NPARAMS) {
        return round(c->steer_disc * 1e15);
    }
    double now = sim_now_s();
    double since_on = (double)(sim_now_ps() - d->power_on_ps) / 1e12;
    switch (PARAMS[i].id) {
    case 256: return d->alarms;
    case 257: return sim_clock_ref_present(c);
    case 258: return floor(d->runtime_s + since_on);
    case 259: return floor(d->locktime_s + (locked(d) ? (double)(sim_now_ps() - d->lock_ps) / 1e12 : 0.0));
    case 263: return locked(d);
    case 264: return d->tod;
    case 265: return d->set.disciplining && c->disc == SIM_DISC_LOCKED;
    case 512: return (double)d->set.pps_offset_ns;
    case 513: return (double)d->set.pps_width_ns;
    case 515: return (double)d->set.cable_ns;
    case 768: return (double)d->set.disciplining;
    case 769: return (double)d->set.pps_source;
    case 770: return (double)d->set.tau0;
    case 771: return (double)d->set.pps_qerr_ps;
    case 772: return (double)d->set.phase_limit_ns;
    case 773: return c->sync_pending;
    case 774: return c->phase_valid ? c->phase_s * 1e9 : 0.0;
    case 775: return round(c->last_correction * 1e15);
    case 777: return (double)d->set.tau1;
    case 778: return (double)d->set.metering;
    case 779: return (double)d->set.thr0;
    case 780: return (double)d->set.thr1;
    case 1288: return 45000;
    case 1289: return 1650000;
    case 1291: return since_on < 60.0 ? 2000000 : 900000;
    case 1292: return 1200000;
    case 1293: return 2500;
    case 1296: return round((sim_mac_baseplate_c(d) + PCB_ABOVE_PLATE_C) * 1000.0);
    case 1300: return (double)d->digital_tuning;
    case 1306: return 12000;
    case 1312: return (double)d->set.analog_en;
    case 1321: return round((c->steer_user + c->steer_disc) * 1e15);
    case 1332: return locked(d) ? 100 : fmin(99.0, floor(100.0 * since_on / d->cfg.warmup_s));
    default: (void)now; return 0;
    }
}

static void format_value(sim_mac_t *d, int i, char *out, size_t n)
{
    double v = param_value(d, i);
    if (i < SIM_MAC_NPARAMS && PARAMS[i].type == P_FLOAT) {
        snprintf(out, n, "%.3f", v);
    } else {
        snprintf(out, n, "%lld", (long long)v);
    }
}

/* Push settings into the clock model. */
static void apply_settings(sim_mac_t *d)
{
    sim_clock_t *c = &d->clk;
    bool src1 = d->set.pps_source == 1;
    c->tau_s = (double)(src1 ? d->set.tau1 : d->set.tau0);
    c->threshold_s = (double)(src1 ? d->set.thr1 : d->set.thr0) * 1e-9;
    c->cfg.jam_limit_s = fabs((double)d->set.phase_limit_ns) * 1e-9;
    c->cable_s = (double)(d->set.cable_ns + d->set.pps_offset_ns) * 1e-9;
    c->pps.width_ps = (sim_time_ps_t)d->set.pps_width_ns * SIM_PS_PER_NS;
    sim_clock_set_mode(c, d->set.disciplining ? SIM_REF_DISCIPLINE
                          : d->set.metering   ? SIM_REF_MEASURE
                                              : SIM_REF_OFF);
}

static sim_mac_settings_t default_settings(void)
{
    sim_mac_settings_t s = {
        .pps_offset_ns = 0, .pps_width_ns = 20000, .cable_ns = 0, .pps_qerr_ps = 0,
        .phase_limit_ns = 1000, .disciplining = 0, .pps_source = 0, .tau0 = 100,
        .tau1 = 100, .metering = 0, .thr0 = 20, .thr1 = 20, .analog_en = 0,
    };
    return s;
}

/* ---------------------------------------------------------------- alarms */

static void update_alarms(sim_mac_t *d)
{
    uint32_t a = 0;
    if (d->acq_failed) {
        a |= ALM_ACQ_FAILED;
    }
    if (d->set.disciplining && !sim_clock_ref_present(&d->clk)) {
        a |= ALM_NO_PPS;
    }
    bool hot = sim_mac_baseplate_c(d) > MAX_BASEPLATE_C;
    if (locked(d) && hot) {
        a |= ALM_TEMPERATURE; /* only asserted while Locked = 1 */
        if (d->hot_since_s < 0.0) {
            d->hot_since_s = sim_now_s();
        } else if (sim_now_s() - d->hot_since_s > OVERHEAT_UNLOCK_S) {
            sim_log(d->base.name, "state", "overheat", sim_mac_baseplate_c(d), "lock lost");
            d->hot_since_s = -1.0;
            sim_mac_lose_lock(d, INFINITY); /* relocks when cooled: see on_own_pps */
            d->overheated = true;
        }
    } else if (!hot) {
        d->hot_since_s = -1.0;
    }
    if (d->set.disciplining && d->clk.disc_at_limit) {
        a |= ALM_DISC_RANGE;
    }
    if (a != d->alarms) {
        sim_log(d->base.name, "state", "alarms", a, "0x%X", a);
    }
    d->acked &= a; /* a resolved alarm can notify again next time */
    d->alarms = a;
    bool active = (a & ~d->acked) != 0;
    sim_pin_set(&d->alarm, d->cfg.alarm_active_high ? active : !active);
}

/* ------------------------------------------------------------- lifecycle */

static void lock_event(void *ctx, uint32_t tag)
{
    sim_mac_t *d = ctx;
    if (tag != d->gen || !d->base.powered) {
        return;
    }
    if (d->acq_failed) {
        update_alarms(d);
        return;
    }
    d->lock_ps = sim_now_ps();
    sim_clock_set_locked(&d->clk, true);
    sim_pin_set(&d->bite, false);
    sim_log(d->base.name, "state", "bite", 0, "locked");
}

static void app_up_event(void *ctx, uint32_t tag)
{
    sim_mac_t *d = ctx;
    if (tag != d->gen || !d->base.powered) {
        return;
    }
    d->app_running = true;
    sim_uart_send_str(&d->uart, "[>Microchip SA5X]\r\n");
}

static void start(sim_mac_t *d)
{
    d->gen++;
    d->app_running = false;
    d->power_on_ps = sim_now_ps();
    d->set = d->has_stored ? d->stored : default_settings();
    d->digital_tuning = 0;
    d->alarms = d->acked = 0;
    d->hot_since_s = -1.0;
    d->overheated = false;
    d->tod = 0;
    d->tod_pending = false;
    d->in_cmd = false;
    for (int i = 0; i < SIM_MAC_NPARAMS; i++) {
        d->upd_last[i] = NAN;
    }
    sim_clock_power(&d->clk, false);
    sim_clock_power(&d->clk, true);
    apply_settings(d);
    sim_pin_set(&d->bite, true);
    update_alarms(d);
    sim_uart_send_str(&d->uart, "[>Loading...]\r\n");
    sim_schedule_ps(sim_now_ps() + (sim_time_ps_t)(BSL_WAIT_S * 1e12), app_up_event, d, d->gen);
    sim_schedule_ps(sim_now_ps() + (sim_time_ps_t)(d->cfg.warmup_s * 1e12), lock_event, d, d->gen);
}

static void on_own_pps(void *ctx, sim_time_ps_t t)
{
    sim_mac_t *d = ctx;
    (void)t;
    if (d->tod_pending) {
        d->tod = d->tod_next;
        d->tod_pending = false;
    } else {
        d->tod++;
    }
    sim_clock_second(&d->clk);
    if (d->overheated && sim_mac_baseplate_c(d) <= MAX_BASEPLATE_C && !d->acq_failed) {
        d->overheated = false;
        d->gen++;
        sim_schedule_ps(sim_now_ps() + (sim_time_ps_t)(d->cfg.warmup_s * 1e12), lock_event, d, d->gen);
    }
    update_alarms(d);
}

void sim_mac_fail_acquisition(sim_mac_t *dev)
{
    dev->acq_failed = true;
    if (dev->clk.locked) {
        sim_mac_lose_lock(dev, INFINITY);
    }
    update_alarms(dev);
}

void sim_mac_lose_lock(sim_mac_t *dev, double relock_s)
{
    if (!dev->base.powered || !dev->clk.locked) {
        return;
    }
    dev->gen++;
    dev->locktime_s += (double)(sim_now_ps() - dev->lock_ps) / 1e12;
    sim_clock_set_locked(&dev->clk, false);
    sim_pin_set(&dev->bite, true);
    sim_log(dev->base.name, "state", "bite", 1, "lock lost");
    sim_time_ps_t t;
    if (dev->app_running && sim_ps_after_s(relock_s, &t)) {
        sim_schedule_ps(t, lock_event, dev, dev->gen);
    }
}

/* -------------------------------------------------------------- protocol */

static uint8_t xor_sum(const char *s, size_t n)
{
    uint8_t x = 0;
    for (size_t i = 0; i < n; i++) {
        x ^= (uint8_t)s[i];
    }
    return x;
}

typedef struct {
    int seq;      /* -1 = none */
    bool cksum;
} reqinfo_t;

static void respond(sim_mac_t *d, const reqinfo_t *r, char type, const char *value)
{
    char body[600];
    int n = 0;
    if (r->seq >= 0) {
        n = snprintf(body, sizeof(body), "#%02X", r->seq);
    }
    snprintf(body + n, sizeof(body) - (size_t)n, "%c%s", type, value);
    char out[640];
    if (r->cksum) {
        snprintf(out, sizeof(out), "[%s|%02X]\r\n", body, xor_sum(body, strlen(body)));
    } else {
        snprintf(out, sizeof(out), "[%s]\r\n", body);
    }
    sim_uart_send_at(&d->uart, sim_now_ns() + (sim_time_ns_t)(d->cfg.reply_delay_s * 1e9),
                     (const uint8_t *)out, strlen(out));
}

static void ok(sim_mac_t *d, const reqinfo_t *r, const char *v)
{
    respond(d, r, '=', v);
}

static void err(sim_mac_t *d, const reqinfo_t *r, int code, const char *what)
{
    char v[8];
    snprintf(v, sizeof(v), "%d", code);
    d->bad_commands++;
    sim_log(d->base.name, "error", "c3_error", code, "%s", what);
    respond(d, r, '!', v);
}

/* Set a writable parameter; returns 0 or a C3 error number. */
static int write_param(sim_mac_t *d, int i, double v)
{
    if (i == SIM_MAC_NPARAMS || PARAMS[i].ro) {
        return 102;
    }
    const param_t *p = &PARAMS[i];
    int64_t iv = (int64_t)llround(v);
    if (iv < p->min || iv > p->max) {
        return 101;
    }
    sim_mac_settings_t *s = &d->set;
    switch (p->id) {
    case 264: d->tod_next = (uint32_t)iv; d->tod_pending = true; return 0;
    case 512: {
        iv = (iv / 10) * 10; /* 10 ns steps */
        double delta = (double)(iv - s->pps_offset_ns) * 1e-9;
        s->pps_offset_ns = iv;
        d->clk.pps.epoch_s = fmod(d->clk.pps.epoch_s + delta + 1.0, 1.0);
        sim_pps_reschedule(&d->clk.pps);
        break;
    }
    case 513: s->pps_width_ns = (iv / 10) * 10; break;
    case 515: s->cable_ns = iv; break;
    case 768:
        if (iv && s->metering) {
            return 101; /* cannot run with phase metering */
        }
        s->disciplining = iv;
        break;
    case 769: s->pps_source = iv; break;
    case 770: s->tau0 = iv; break;
    case 771: s->pps_qerr_ps = iv; d->qerr_next_ps = iv; break;
    case 772: s->phase_limit_ns = iv; break;
    case 777: s->tau1 = iv; break;
    case 778:
        if (iv && s->disciplining) {
            return 101;
        }
        s->metering = iv;
        break;
    case 779: s->thr0 = iv; break;
    case 780: s->thr1 = iv; break;
    case 1300:
        d->digital_tuning = iv;
        sim_clock_set_user_steer(&d->clk, (double)iv * 1e-15);
        return 0;
    case 1312: s->analog_en = iv; break;
    default: return 102;
    }
    apply_settings(d);
    return 0;
}

static void cmd_upd(sim_mac_t *d, const reqinfo_t *r)
{
    char out[600] = "";
    size_t n = 0;
    for (int i = 0; i < SIM_MAC_NPARAMS && n < sizeof(out) - 40; i++) {
        double v = param_value(d, i);
        if (v != d->upd_last[i]) {
            d->upd_last[i] = v;
            char val[32];
            format_value(d, i, val, sizeof(val));
            n += (size_t)snprintf(out + n, sizeof(out) - n, ",%u,%s", PARAMS[i].id, val);
        }
    }
    ok(d, r, out);
}

static void execute(sim_mac_t *d, char *msg)
{
    /* msg = contents between { and } */
    reqinfo_t r = {.seq = -1, .cksum = false};
    char *bar = strrchr(msg, '|');
    if (bar) {
        r.cksum = true;
        if (strlen(bar + 1) != 2 || strtoul(bar + 1, NULL, 16) != xor_sum(msg, (size_t)(bar - msg))) {
            r.cksum = false; /* the guide's example reply is a bare [!3] */
            err(d, &r, 3, msg);
            return;
        }
        *bar = '\0';
    }
    char *args[4] = {0};
    int nargs = 0;
    char *comma = strchr(msg, ',');
    if (comma) {
        *comma = '\0';
        char *a = comma + 1;
        while (a && nargs < 4) {
            char *next = strchr(a, ',');
            if (next) {
                *next++ = '\0';
            }
            if (*a == '"') {
                a++;
                char *q = strchr(a, '"');
                if (q) {
                    *q = '\0';
                }
            }
            args[nargs++] = a;
            a = next;
        }
    }
    char *hash = strchr(msg, '#');
    if (hash) {
        *hash = '\0';
        r.seq = (int)strtol(hash + 1, NULL, 16);
    }
    const char *cmd = msg;
    sim_log(d->base.name, "cmd", cmd, nargs, "%s", nargs ? args[0] : "");

    char val[64];
    if (strcmp(cmd, "get") == 0 || strcmp(cmd, "set") == 0 || strcmp(cmd, "add") == 0) {
        bool is_get = cmd[0] == 'g';
        if (nargs < (is_get ? 1 : 2)) {
            err(d, &r, 2, cmd);
            return;
        }
        int i = find_param(args[0]);
        if (i < 0) {
            err(d, &r, 100, args[0]);
            return;
        }
        if (!is_get) {
            double v = strtod(args[1], NULL);
            if (cmd[0] == 'a') {
                v += param_value(d, i);
            }
            int e = write_param(d, i, v);
            if (e) {
                err(d, &r, e, args[0]);
                return;
            }
        }
        format_value(d, i, val, sizeof(val));
        ok(d, &r, val);
    } else if (strcmp(cmd, "upd") == 0) {
        cmd_upd(d, &r);
    } else if (strcmp(cmd, "ackalm") == 0) {
        if (nargs < 1) {
            err(d, &r, 2, cmd);
            return;
        }
        d->acked |= (uint32_t)strtoul(args[0], NULL, 10) & d->alarms;
        update_alarms(d);
        ok(d, &r, "1");
    } else if (strcmp(cmd, "sync") == 0) {
        if (d->set.disciplining || d->set.metering) {
            sim_clock_request_sync(&d->clk, 3.0);
        }
        ok(d, &r, "1");
    } else if (strcmp(cmd, "latch") == 0) {
        if (!locked(d)) {
            err(d, &r, 101, "latch while unlocked");
            return;
        }
        sim_clock_latch(&d->clk);
        d->digital_tuning = 0;
        ok(d, &r, "1");
    } else if (strcmp(cmd, "store") == 0) {
        d->stored = d->set;
        d->has_stored = true;
        ok(d, &r, "1");
    } else if (strcmp(cmd, "load") == 0) {
        if (d->has_stored) {
            d->set = d->stored;
            apply_settings(d);
        }
        ok(d, &r, d->has_stored ? "1" : "0");
    } else if (strcmp(cmd, "reset") == 0) {
        d->runtime_s += (double)(sim_now_ps() - d->power_on_ps) / 1e12;
        if (d->clk.locked) {
            d->locktime_s += (double)(sim_now_ps() - d->lock_ps) / 1e12;
        }
        start(d); /* no response */
    } else if (strcmp(cmd, "app?") == 0) {
        ok(d, &r, "clock");
    } else if (strcmp(cmd, "device?") == 0 || strcmp(cmd, "platform?") == 0) {
        ok(d, &r, "sa5x");
    } else if (strcmp(cmd, "describe?") == 0) {
        ok(d, &r, "Microchip SA5X");
    } else if (strcmp(cmd, "serial?") == 0) {
        ok(d, &r, d->cfg.serial);
    } else if (strcmp(cmd, "swrev?") == 0) {
        ok(d, &r, d->cfg.swrev);
    } else if (strcmp(cmd, "hwrev?") == 0) {
        ok(d, &r, "1");
    } else if (strcmp(cmd, "pn?") == 0) {
        ok(d, &r, "090-00253-000");
    } else if (strcmp(cmd, "id?") == 0) {
        char id[96];
        snprintf(id, sizeof(id), ",d:sa5x,a:clock,v:%s", d->cfg.swrev);
        ok(d, &r, id);
    } else if (strcmp(cmd, "help") == 0) {
        ok(d, &r, ",ackalm,add,describe?,get,help,hwrev?,id?,latch,load,pn?,reset,serial?,set,"
                  "store,swrev?,sync,upd");
    } else {
        err(d, &r, 1, cmd);
    }
}

static void on_rx(sim_uart_t *u, const uint8_t *buf, size_t len, void *user)
{
    (void)u;
    sim_mac_t *d = user;
    if (!d->app_running) {
        return; /* bootloader window / off */
    }
    for (size_t i = 0; i < len; i++) {
        char c = (char)buf[i];
        if (c == '{') {
            d->in_cmd = true;
            d->line_len = 0;
        } else if (!d->in_cmd) {
            continue;
        } else if (c == '}') {
            d->in_cmd = false;
            d->line[d->line_len] = '\0';
            execute(d, d->line);
        } else if (d->line_len < sizeof(d->line) - 1) {
            d->line[d->line_len++] = c;
        }
    }
}

/* ------------------------------------------------------------- 1PPS input */

static void ref_edge(sim_mac_t *d, int input, bool rising, sim_time_ps_t t)
{
    if (!rising || input != d->set.pps_source) {
        return;
    }
    if (d->qerr_next_ps != 0) {
        /* Known quantisation error of this pulse (GNSS receivers). */
        t = (sim_time_ps_t)((int64_t)t + d->qerr_next_ps);
        d->qerr_next_ps = 0;
    }
    sim_clock_ref_edge(&d->clk, t);
}

static void ref0(void *ctx, bool rising, sim_time_ps_t t)
{
    ref_edge(ctx, 0, rising, t);
}

static void ref1(void *ctx, bool rising, sim_time_ps_t t)
{
    ref_edge(ctx, 1, rising, t);
}

void sim_mac_attach_ref(sim_mac_t *dev, int input, sim_pin_t *ref_pps)
{
    sim_pin_listen(ref_pps, input ? ref1 : ref0, dev);
}

/* ----------------------------------------------------------------- setup */

sim_mac_config_t sim_mac_default_config(uint64_t seed)
{
    sim_mac_config_t c;
    memset(&c, 0, sizeof(c));
    c.serial = "2611MAC0001";
    c.swrev = "V1.6.0.0";
    c.warmup_s = 300.0;  /* guide 1.3: typically ~5 min; data sheet max 8 min (> -10 C) */
    c.reply_delay_s = 0.005;
    c.temperature_c = sim_signal_const(30.0);

    /* Free-running TCXO before lock, guide 1.3: +-2 ppm over its operating range. */
    c.clock.free_run.y0 = 4e-6 * (sim_hash_uniform(seed, 1) - 0.5);
    c.clock.free_run.adev_1s = 1e-9;      /* UNVERIFIED */
    c.clock.free_run.tempco = 5e-8;       /* UNVERIFIED */
    c.clock.free_run.t_ref_c = 25.0;
    c.clock.free_run.seed = seed;
    /*
     * Locked, SA53 column of the MAC-SA5X data sheet (DS00003348E):
     *   ADEV <3e-11 @1 s, <1e-11 @10 s, <3e-12 @100 s, <1e-12 @1000 s,
     *     <3e-12 @10000 s: white FM plus random-walk FM sized to the 1000 s and
     *     10000 s points (q = 2e-14 / sqrt(s))
     *   drift <1e-10 / month (<1.5e-9 / year)
     *   TempCo <1e-10 peak-to-peak over -40..+75 C: drawn within +-1e-10 / 115 C
     *   offset at shipment and retrace: +-5e-11
     *   acceleration sensitivity: NOT SPECIFIED (no loss of lock at 7.7 grms;
     *     <=4e-8 momentary under 30 g shock) -> 0 until we have a number
     * SA55 instead: ADEV 1.5e-11 / 5e-12 / 1.5e-12 / 5e-13 / 1.5e-12,
     *   drift 5e-11 / month, TempCo 5e-11 p-p.
     */
    c.clock.locked.y0 = 5e-11 * (2.0 * sim_hash_uniform(seed, 2) - 1.0);
    c.clock.locked.aging_per_day = 1e-10 / 30.0;
    c.clock.locked.adev_1s = 3e-11;
    c.clock.locked.rw_per_sqrt_s = 2e-14;
    c.clock.locked.tempco = (1e-10 / 115.0) * (2.0 * sim_hash_uniform(seed, 4) - 1.0);
    c.clock.locked.t_ref_c = 25.0;
    c.clock.locked.seed = seed + 1;
    c.clock.retrace = 5e-11;
    c.clock.steer_range = 1e-6;    /* data sheet: digital tuning +-1e-6; guide 3.3 net +-1e-6 */
    c.clock.rf_hz = 10e6;          /* 1.3: 1PPS within +-50 ns of the 10 MHz */
    c.clock.meter_res_s = 450e-12; /* 3.4: 450 ps */
    c.clock.steer_res = 1e-14;     /* guide 3.3: ~1e-14 realised (data sheet says 1e-15) */
    c.clock.disc_limit = 2e-8;     /* alarm bit 18: beyond the DigitalTuning range */
    c.clock.jam_limit_s = 1e-6;    /* PhaseLimit default 1000 ns */
    c.clock.jam_restarts_servo = true;
    c.clock.pps_width_ps = 20ULL * 1000000ULL; /* 20 us */
    c.bite_when_off = false;
    c.alarm_when_off = false;
    c.alarm_active_high = false; /* guide 1.2.3 / 4.4.3; the data sheet says the opposite */
    return c;
}

void sim_mac_init(sim_mac_t *dev, const char *name, const sim_mac_config_t *cfg)
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
    sim_pin_init(&dev->alarm, name, cfg->alarm_when_off);
    sim_clock_init(&dev->clk, name, &cfg->clock);
    dev->clk.pps.on_edge = on_own_pps;
    dev->clk.pps.ctx = dev;
    dev->set = default_settings();
}

void sim_mac_power(sim_mac_t *dev, bool on)
{
    if (on == dev->base.powered) {
        return;
    }
    sim_device_power(&dev->base, on);
    sim_device_power(&dev->uart.base, on);
    if (on) {
        start(dev);
        return;
    }
    dev->gen++;
    dev->runtime_s += (double)(sim_now_ps() - dev->power_on_ps) / 1e12;
    if (dev->clk.locked) {
        dev->locktime_s += (double)(sim_now_ps() - dev->lock_ps) / 1e12;
    }
    dev->app_running = false;
    sim_clock_power(&dev->clk, false);
    sim_pin_set(&dev->bite, dev->cfg.bite_when_off);
    sim_pin_set(&dev->alarm, dev->cfg.alarm_when_off);
}
