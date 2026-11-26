#include "sim_gps.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sim_log.h"

#define CLS_NAV 0x01
#define CLS_ACK 0x05
#define CLS_CFG 0x06
#define CLS_MON 0x0A
#define CLS_TIM 0x0D
#define ID_NAV_PVT 0x07
#define ID_TIM_TP 0x01
#define ID_MON_VER 0x04
#define ID_CFG_RST 0x04
#define ID_CFG_VALSET 0x8A
#define ID_CFG_VALGET 0x8B

#define GPS_EPOCH_UNIX 315964800.0 /* 1980-01-06 */
#define PRE_EDGE_S 0.001           /* decide each pulse 1 ms ahead */

#include "sim_gps_keys.inc"

/* Binary search: GPS_KEYS is sorted by key ID. */
static int key_index(uint32_t key)
{
    int lo = 0;
    int hi = SIM_GPS_NKEYS - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (GPS_KEYS[mid].key == key) {
            return mid;
        }
        if (GPS_KEYS[mid].key < key) {
            lo = mid + 1;
        } else {
            hi = mid - 1;
        }
    }
    return -1;
}

/* Value size in bytes, from the key ID's size field (bits 30..28). */
static size_t key_size(uint32_t key)
{
    static const size_t sz[8] = {0, 1, 1, 2, 4, 8, 0, 0};
    return sz[(key >> 28) & 7];
}

uint64_t sim_gps_get_key(const sim_gps_t *dev, uint32_t key)
{
    int i = key_index(key);
    return i < 0 ? 0 : dev->ram[i];
}

static uint32_t K(const sim_gps_t *d, uint32_t key)
{
    return (uint32_t)sim_gps_get_key(d, key);
}

static int32_t Ks(const sim_gps_t *d, uint32_t key) /* signed (I1/I2/I4) */
{
    uint64_t v = sim_gps_get_key(d, key);
    size_t sz = key_size(key);
    uint64_t sign = 1ULL << (8 * sz - 1);
    return (int32_t)(int64_t)((v ^ sign) - sign);
}

/* ------------------------------------------------------------- framing */

static void put_u16(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put_u32(uint8_t *p, uint32_t v)
{
    put_u16(p, v);
    put_u16(p + 2, v >> 16);
}

size_t sim_ubx_frame(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len, uint8_t *out)
{
    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = cls;
    out[3] = id;
    put_u16(out + 4, len);
    if (len) {
        memcpy(out + 6, payload, len);
    }
    uint8_t a = 0;
    uint8_t b = 0;
    for (size_t i = 2; i < 6u + len; i++) {
        a = (uint8_t)(a + out[i]);
        b = (uint8_t)(b + a);
    }
    out[6 + len] = a;
    out[7 + len] = b;
    return 8u + len;
}

static void send_ubx(sim_gps_t *d, uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len)
{
    if (!K(d, GPS_KEY_UART1OUTPROT_UBX)) {
        return;
    }
    uint8_t frame[1100];
    size_t n = sim_ubx_frame(cls, id, p, len, frame);
    sim_uart_send(&d->uart, frame, n);
}

static void send_nmea(sim_gps_t *d, const char *body)
{
    uint8_t x = 0;
    for (const char *p = body; *p; p++) {
        x ^= (uint8_t)*p;
    }
    char line[128];
    int n = snprintf(line, sizeof(line), "$%s*%02X\r\n", body, x);
    sim_uart_send(&d->uart, (const uint8_t *)line, (size_t)n);
}

static void ack(sim_gps_t *d, bool ok, uint8_t cls, uint8_t id)
{
    uint8_t p[2] = {cls, id};
    if (!ok) {
        d->naks++;
        sim_log(d->base.name, "error", "ubx_nak", id, "class 0x%02X id 0x%02X", cls, id);
    }
    send_ubx(d, CLS_ACK, ok ? 0x01 : 0x00, p, 2);
}

/* --------------------------------------------------------------- time */

static double gps_time_s(const sim_gps_t *d, double t)
{
    return d->cfg.gps_start_s + t;
}

static void civil_from_days(int64_t z, int *y, unsigned *m, unsigned *dd)
{
    /* Howard Hinnant's days -> civil date. */
    z += 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int64_t yy = (int64_t)yoe + era * 400;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153;
    *dd = doy - (153 * mp + 2) / 5 + 1;
    *m = mp < 10 ? mp + 3 : mp - 9;
    *y = (int)(yy + (*m <= 2));
}

typedef struct {
    int year;
    unsigned month, day, hour, min, sec;
    int32_t nano;
} utc_t;

static utc_t utc_at(const sim_gps_t *d, double t)
{
    /* Integration manual 3.12.6: the date/time fields are the time rounded to
     * the nearest 0.01 s; nano corrects them (-5 ms .. +995 ms). */
    double unix_s = gps_time_s(d, t) + GPS_EPOCH_UNIX - d->cfg.leap_s;
    double whole = floor(round(unix_s * 100.0) / 100.0);
    int64_t secs = (int64_t)whole;
    utc_t u;
    int64_t days = secs >= 0 ? secs / 86400 : (secs - 86399) / 86400;
    int64_t sod = secs - days * 86400;
    civil_from_days(days, &u.year, &u.month, &u.day);
    u.hour = (unsigned)(sod / 3600);
    u.min = (unsigned)((sod / 60) % 60);
    u.sec = (unsigned)(sod % 60);
    u.nano = (int32_t)llround((unix_s - whole) * 1e9);
    return u;
}

/* ---------------------------------------------------------------- fix */

static double accel_mag(const sim_gps_t *d, sim_time_ns_t t)
{
    double s = 0.0;
    bool any = false;
    for (int i = 0; i < 3; i++) {
        if (d->cfg.accel_g[i]) {
            double a = sim_signal_eval_clean(d->cfg.accel_g[i], t);
            s += a * a;
            any = true;
        }
    }
    return any ? sqrt(s) : 1.0;
}

/* Integration manual 3.1.7.1, tables 3 and 4. If a sanity check fails the
 * position solution is invalidated. Airborne models check altitude only. */
static const struct {
    uint8_t model;
    double max_alt_m, max_hvel, max_vvel;
    bool check_velocity;
    double max_g; /* only airborne models state one */
} DYN_MODELS[] = {
    {0, 12000, 310, 50, true, 0},      /* portable (default) */
    {2, 9000, 10, 6, true, 0},         /* stationary */
    {3, 9000, 30, 20, true, 0},        /* pedestrian */
    {4, 6000, 100, 15, true, 0},       /* automotive */
    {5, 500, 25, 5, true, 0},          /* at sea */
    {6, 80000, 100, 6400, false, 1.0}, /* airborne <1g */
    {7, 80000, 250, 10000, false, 2.0},/* airborne <2g */
    {8, 80000, 500, 20000, false, 4.0},/* airborne <4g */
    {9, 9000, 30, 20, true, 0},        /* wrist */
};

static bool conditions_ok(sim_gps_t *d, const char **why)
{
    sim_time_ns_t t = sim_now_ns();
    uint32_t dyn = K(d, GPS_KEY_NAVSPG_DYNMODEL);
    int m = 0;
    for (int i = 0; i < (int)(sizeof(DYN_MODELS) / sizeof(DYN_MODELS[0])); i++) {
        if (DYN_MODELS[i].model == dyn) {
            m = i;
        }
    }
    /* Non-airborne models give no g limit: use the data sheet's 4 g dynamics. */
    double g_limit = DYN_MODELS[m].max_g > 0 ? DYN_MODELS[m].max_g : d->cfg.max_accel_g;
    double vd = sim_signal_eval_clean(&d->cfg.vel_d, t);
    double vn = sim_signal_eval_clean(&d->cfg.vel_n, t);
    double ve = sim_signal_eval_clean(&d->cfg.vel_e, t);
    double alt = sim_signal_eval_clean(&d->cfg.alt_m, t);

    if (sim_signal_eval_clean(&d->cfg.sky, t) < 0.5) {
        *why = "no sky";
        return false;
    }
    if (accel_mag(d, t) > g_limit) {
        *why = "acceleration above dynamic model limit";
        return false;
    }
    if (alt > DYN_MODELS[m].max_alt_m) {
        *why = "altitude above dynamic model limit";
        return false;
    }
    if (DYN_MODELS[m].check_velocity &&
        (fabs(vd) > DYN_MODELS[m].max_vvel || sqrt(vn * vn + ve * ve) > DYN_MODELS[m].max_hvel)) {
        *why = "velocity above dynamic model limit";
        return false;
    }
    /* Data sheet table 1 operational limits (Airborne 4 g): 500 m/s, 80 km. */
    if (sqrt(vn * vn + ve * ve + vd * vd) > 500.0 || alt > 80000.0) {
        *why = "operational limit (500 m/s / 80 km)";
        return false;
    }
    return true;
}

static void update_fix(sim_gps_t *d)
{
    const char *why = "GNSS stopped";
    double now = sim_now_s();
    bool ok = !d->gnss_stopped && conditions_ok(d, &why);
    if (!ok) {
        if (d->fix) {
            d->fix = false;
            sim_log(d->base.name, "state", "fix", 0, "lost: %s", why);
        }
        d->ok_since_s = -1.0;
        return;
    }
    if (d->ok_since_s < 0.0) {
        d->ok_since_s = now;
    }
    /* Hot needs valid ephemeris; warm and cold both download it first
     * (integration manual 3.15: 18-36 s), so they take the cold-start time. */
    double need = d->ephemeris_valid ? d->cfg.hot_reacq_s : d->cfg.cold_ttff_s;
    if (!d->fix && now - d->ok_since_s >= need) {
        d->fix = true;
        d->ephemeris_valid = true;
        d->has_time = true;
        sim_log(d->base.name, "state", "fix", 3, "3D fix");
        if (!d->utc_known && d->utc_due_s < 0.0) {
            /* GPS sends the UTC parameters once every 12.5 minutes (IM 3.13.1.3). */
            d->utc_due_s = now + 750.0 * sim_hash_uniform(0x07C0, d->gen);
        }
    }
    if (!d->utc_known && d->utc_due_s >= 0.0 && now >= d->utc_due_s && d->fix) {
        d->utc_known = true;
        sim_log(d->base.name, "state", "utc_known", 1, "leap seconds / UTC parameters received");
    }
}

/* Receiver-clock quantisation of the pulse at second k (seconds, signed). */
static double qerr_offset(const sim_gps_t *d, double k)
{
    if (!d->cfg.qerr_enabled || d->cfg.rx_clock_hz <= 0.0) {
        return 0.0;
    }
    double f = d->cfg.rx_clock_hz * (1.0 + d->cfg.rx_clock_ppb * 1e-9);
    double cycles = k * f;
    return (round(cycles) - cycles) / f;
}

/* ----------------------------------------------------------- messages */

static void send_nav_pvt(sim_gps_t *d, double t_epoch)
{
    uint8_t p[92];
    memset(p, 0, sizeof(p));
    sim_time_ns_t t = (sim_time_ns_t)(t_epoch * 1e9);
    double gs = gps_time_s(d, t_epoch);
    uint32_t itow = (uint32_t)llround(fmod(gs, 604800.0) * 1000.0);
    utc_t u = utc_at(d, t_epoch);

    put_u32(p + 0, itow);
    put_u16(p + 4, (uint32_t)u.year);
    p[6] = (uint8_t)u.month;
    p[7] = (uint8_t)u.day;
    p[8] = (uint8_t)u.hour;
    p[9] = (uint8_t)u.min;
    p[10] = (uint8_t)u.sec;
    /* validDate | validTime once time is known; fullyResolved needs the leap
     * seconds (integration manual 3.12.5). */
    p[11] = (uint8_t)((d->has_time ? 0x03 : 0) | (d->utc_known ? 0x04 : 0));
    put_u32(p + 12, d->fix ? 20 : 0xFFFFFFFFu); /* tAcc ns */
    put_u32(p + 16, (uint32_t)u.nano);
    p[20] = d->fix ? 3 : 0;
    p[21] = d->fix ? 0x01 : 0x00; /* gnssFixOK */
    /* confirmedDate | confirmedTime (UNVERIFIED: tied to the UTC parameters
     * here). confirmedAvai is not supported in protocol 32.01. */
    p[22] = d->utc_known ? 0xC0 : 0x00;
    p[23] = d->fix ? (uint8_t)sim_signal_eval_clean(&d->cfg.num_sv, t) : 0;
    put_u32(p + 24, (uint32_t)(int32_t)llround(sim_signal_eval(&d->cfg.lon_deg, t) * 1e7));
    put_u32(p + 28, (uint32_t)(int32_t)llround(sim_signal_eval(&d->cfg.lat_deg, t) * 1e7));
    double alt = sim_signal_eval(&d->cfg.alt_m, t);
    put_u32(p + 32, (uint32_t)(int32_t)llround((alt + 30.0) * 1000.0)); /* ellipsoid ~ MSL + N */
    put_u32(p + 36, (uint32_t)(int32_t)llround(alt * 1000.0));
    put_u32(p + 40, d->fix ? 1500 : 0xFFFFFFFFu);
    put_u32(p + 44, d->fix ? 2500 : 0xFFFFFFFFu);
    double vn = sim_signal_eval(&d->cfg.vel_n, t);
    double ve = sim_signal_eval(&d->cfg.vel_e, t);
    double vd = sim_signal_eval(&d->cfg.vel_d, t);
    put_u32(p + 48, (uint32_t)(int32_t)llround(vn * 1000.0));
    put_u32(p + 52, (uint32_t)(int32_t)llround(ve * 1000.0));
    put_u32(p + 56, (uint32_t)(int32_t)llround(vd * 1000.0));
    put_u32(p + 60, (uint32_t)(int32_t)llround(sqrt(vn * vn + ve * ve) * 1000.0));
    double head = fmod(atan2(ve, vn) * 180.0 / M_PI + 360.0, 360.0);
    put_u32(p + 64, (uint32_t)(int32_t)llround(head * 1e5));
    put_u32(p + 68, d->fix ? 300 : 0xFFFFFFFFu);
    put_u32(p + 72, d->fix ? 5000000 : 0xFFFFFFFFu);
    put_u16(p + 76, d->fix ? 150 : 9999);
    put_u16(p + 78, d->fix ? 0 : 0x0001); /* flags3.invalidLlh */
    put_u32(p + 84, (uint32_t)(int32_t)llround(head * 1e5)); /* headVeh = headMot */
    send_ubx(d, CLS_NAV, ID_NAV_PVT, p, sizeof(p));
}

static void send_tim_tp(sim_gps_t *d, double t_next_pulse)
{
    uint8_t p[16];
    /* Time base per CFG-TP-TIMEGRID_TP1: 0 = UTC (default), 1 = GPS. UTC
     * week/tow count from the GPS epoch, minus the leap seconds. */
    bool utc = K(d, GPS_KEY_TP_TIMEGRID_TP1) == 0;
    double gs = gps_time_s(d, t_next_pulse) - (utc ? d->cfg.leap_s : 0);
    double week = floor(gs / 604800.0);
    double tow_ms = (gs - week * 604800.0) * 1000.0;
    uint32_t ms = (uint32_t)floor(tow_ms + 1e-9);
    /* qErr convention used here: ideal time = actual pulse + qErr. */
    double qerr_ps = -qerr_offset(d, t_next_pulse) * 1e12;
    put_u32(p + 0, ms);
    put_u32(p + 4, 0); /* towSubMS: pulses on whole seconds */
    put_u32(p + 8, (uint32_t)(int32_t)llround(qerr_ps));
    put_u16(p + 12, (uint32_t)week);
    /* bit0 timeBase, bit1 utc available, bit4 qErrInvalid, bit5 TpNotLocked */
    p[14] = (uint8_t)((utc ? 0x01 : 0) | (d->utc_known ? 0x02 : 0) | (d->fix ? 0 : 0x30));
    p[15] = utc ? 0x00 : 0x00; /* timeRefGnss 0 = GPS / utcStandard 0 = n/a */
    send_ubx(d, CLS_TIM, ID_TIM_TP, p, sizeof(p));
}

static void send_mon_ver(sim_gps_t *d)
{
    /* Interface description 1.2: SPG 4.04 = EXT CORE 4.04 (d964f4), protocol
     * 32.01. The ROM BASE revision is unknown, so that extension is left out
     * (the document allows entries to be missing). */
    static const char *ext[] = {"FWVER=SPG 4.04", "PROTVER=32.01", "MOD=NEO-M9N",
                                "GPS;GLO;GAL;BDS", "SBAS;QZSS"};
    uint8_t p[40 + 30 * 5];
    memset(p, 0, sizeof(p));
    snprintf((char *)p, 30, "EXT CORE 4.04 (d964f4)");
    snprintf((char *)p + 30, 10, "00190000");
    for (int i = 0; i < 5; i++) {
        snprintf((char *)p + 40 + 30 * i, 30, "%s", ext[i]);
    }
    send_ubx(d, CLS_MON, ID_MON_VER, p, sizeof(p));
}

static void nmea_coord(double v, bool lat, char *out, size_t n, char *hemi)
{
    double a = fabs(v);
    int deg = (int)a;
    double minutes = (a - deg) * 60.0;
    *hemi = lat ? (v >= 0 ? 'N' : 'S') : (v >= 0 ? 'E' : 'W');
    snprintf(out, n, lat ? "%02d%08.5f" : "%03d%08.5f", deg, minutes);
}

static void send_nmea_epoch(sim_gps_t *d, double t_epoch)
{
    if (!K(d, GPS_KEY_UART1OUTPROT_NMEA)) {
        return;
    }
    sim_time_ns_t t = (sim_time_ns_t)(t_epoch * 1e9);
    utc_t u = utc_at(d, t_epoch);
    char time[16];
    char date[8];
    snprintf(time, sizeof(time), "%02u%02u%02u.00", u.hour, u.min, u.sec);
    snprintf(date, sizeof(date), "%02u%02u%02u", u.day, u.month, (unsigned)(u.year % 100));
    char lat[16] = "";
    char lon[16] = "";
    char ns = ' ';
    char ew = ' ';
    if (d->fix) {
        nmea_coord(sim_signal_eval(&d->cfg.lat_deg, t), true, lat, sizeof(lat), &ns);
        nmea_coord(sim_signal_eval(&d->cfg.lon_deg, t), false, lon, sizeof(lon), &ew);
    }
    double vn = sim_signal_eval(&d->cfg.vel_n, t);
    double ve = sim_signal_eval(&d->cfg.vel_e, t);
    double kn = sqrt(vn * vn + ve * ve) * 1.943844;
    double head = fmod(atan2(ve, vn) * 180.0 / M_PI + 360.0, 360.0);
    int sv = d->fix ? (int)sim_signal_eval_clean(&d->cfg.num_sv, t) : 0;
    char b[120];
    char nsb[2] = {ns, 0};
    char ewb[2] = {ew, 0};
    if (!d->fix) {
        nsb[0] = ewb[0] = '\0';
    }

    if (K(d, GPS_KEY_MSGOUT_RMC_UART1)) {
        snprintf(b, sizeof(b), "GNRMC,%s,%c,%s,%s,%s,%s,%.3f,%.2f,%s,,,%c,V", d->has_time ? time : "",
                 d->fix ? 'A' : 'V', lat, nsb, lon, ewb, kn, head, d->has_time ? date : "",
                 d->fix ? 'A' : 'N');
        send_nmea(d, b);
    }
    if (K(d, GPS_KEY_MSGOUT_VTG_UART1)) {
        snprintf(b, sizeof(b), "GNVTG,%.2f,T,,M,%.3f,N,%.3f,K,%c", head, kn, kn * 1.852,
                 d->fix ? 'A' : 'N');
        send_nmea(d, b);
    }
    if (K(d, GPS_KEY_MSGOUT_GGA_UART1)) {
        double alt = sim_signal_eval(&d->cfg.alt_m, t);
        if (d->fix) {
            snprintf(b, sizeof(b), "GNGGA,%s,%s,%s,%s,%s,1,%02d,1.00,%.1f,M,30.0,M,,", time, lat, nsb,
                     lon, ewb, sv, alt);
        } else {
            snprintf(b, sizeof(b), "GNGGA,%s,,,,,0,00,99.99,,,,,,", d->has_time ? time : "");
        }
        send_nmea(d, b);
    }
    if (K(d, GPS_KEY_MSGOUT_GSA_UART1)) {
        snprintf(b, sizeof(b), "GNGSA,A,%d,,,,,,,,,,,,,%s,%s,%s,1", d->fix ? 3 : 1,
                 d->fix ? "1.50" : "99.99", d->fix ? "1.00" : "99.99", d->fix ? "1.10" : "99.99");
        send_nmea(d, b);
    }
    if (K(d, GPS_KEY_MSGOUT_GSV_UART1)) {
        snprintf(b, sizeof(b), "GPGSV,1,1,%02d,,,,,,,,,,,,,,,,,1", sv);
        send_nmea(d, b);
    }
    if (K(d, GPS_KEY_MSGOUT_GLL_UART1)) {
        snprintf(b, sizeof(b), "GNGLL,%s,%s,%s,%s,%s,%c,%c", lat, nsb, lon, ewb,
                 d->has_time ? time : "", d->fix ? 'A' : 'V', d->fix ? 'A' : 'N');
        send_nmea(d, b);
    }
}

/* ------------------------------------------------------------- schedule */

static void second_event(void *ctx, uint32_t tag);
static void epoch_event(void *ctx, uint32_t tag);

/* Pulse edges are placed ahead of time; tagging them with gen means a power-off
 * or reset in between cancels them, as it would stop the real timepulse. */
static void pps_edge_event(sim_gps_t *d, uint32_t tag, bool level)
{
    if (tag != d->gen || !d->base.powered) {
        return;
    }
    sim_pin_set_exact(&d->pps, level, sim_event_time_ps());
}

static void pps_high_event(void *ctx, uint32_t tag)
{
    pps_edge_event(ctx, tag, true);
}

static void pps_low_event(void *ctx, uint32_t tag)
{
    pps_edge_event(ctx, tag, false);
}

static void pps_set_at(sim_gps_t *d, bool level, sim_time_ps_t t)
{
    if (t <= sim_now_ps()) {
        sim_pin_set(&d->pps, level);
        return;
    }
    sim_schedule_ps(t, level ? pps_high_event : pps_low_event, d, d->gen);
}

static void schedule_second(sim_gps_t *d, double k)
{
    sim_schedule_ps((sim_time_ps_t)llround((k - PRE_EDGE_S) * 1e12), second_event, d, d->gen);
}

static double meas_period_s(const sim_gps_t *d)
{
    uint32_t ms = K(d, GPS_KEY_RATE_MEAS) * (K(d, GPS_KEY_RATE_NAV) ? K(d, GPS_KEY_RATE_NAV) : 1);
    return (ms ? ms : 1000) / 1000.0;
}

static void schedule_epoch(sim_gps_t *d)
{
    double p = meas_period_s(d);
    double next = (floor(sim_now_s() / p + 1e-9) + 1.0) * p;
    sim_schedule_ps((sim_time_ps_t)llround((next + d->cfg.nav_latency_s) * 1e12), epoch_event, d,
                    d->gen);
}

/* 1 ms before each whole second: update fix, decide and place the pulse. */
static void second_event(void *ctx, uint32_t tag)
{
    sim_gps_t *d = ctx;
    if (tag != d->gen || !d->base.powered) {
        return;
    }
    double k = round(sim_now_s() + PRE_EDGE_S);
    update_fix(d);
    bool locked = d->fix && K(d, GPS_KEY_TP_USE_LOCKED_TP1);
    uint32_t len_us = locked ? K(d, GPS_KEY_TP_LEN_LOCK_TP1) : K(d, GPS_KEY_TP_LEN_TP1);
    /* On the UTC grid (default) there is no pulse until the UTC parameters are
     * known: up to 12.5 min after a cold start (integration manual 3.13.1.3). */
    bool grid_ok = K(d, GPS_KEY_TP_TIMEGRID_TP1) != 0 || d->utc_known;
    if (K(d, GPS_KEY_TP_TP1_ENA) && len_us > 0 && d->has_time && grid_ok) {
        bool rising = K(d, GPS_KEY_TP_POL_TP1) != 0;
        /* User delay shifts the pulse; the receiver advances it by the configured
         * antenna cable delay, so only (real - configured) remains as error. */
        double err = qerr_offset(d, k) + Ks(d, GPS_KEY_TP_USER_DELAY_TP1) * 1e-9 +
                     (d->cfg.antenna_cable_delay_ns - Ks(d, GPS_KEY_TP_ANT_CABLEDELAY)) * 1e-9;
        if (d->cfg.pps_noise_rms_s > 0.0) {
            err += d->cfg.pps_noise_rms_s * sim_hash_gauss(0x6E05, (uint64_t)k);
        }
        sim_time_ps_t t = (sim_time_ps_t)llround((k + err) * 1e12);
        sim_time_ps_t w = (sim_time_ps_t)len_us * 1000000ULL;
        pps_set_at(d, rising, t);
        pps_set_at(d, !rising, t + w);
        if (sim_log_is_open()) {
            sim_log(d->base.name, "truth", "pps_offset_s", err, "edge %.0f", k);
        }
    }
    schedule_second(d, k + 1.0);
}

static void epoch_event(void *ctx, uint32_t tag)
{
    sim_gps_t *d = ctx;
    if (tag != d->gen || !d->base.powered) {
        return;
    }
    double p = meas_period_s(d);
    double t_epoch = round((sim_now_s() - d->cfg.nav_latency_s) / p) * p;
    uint64_t n = d->epoch++;
    uint32_t pvt = K(d, GPS_KEY_MSGOUT_NAV_PVT_UART1);
    uint32_t tp = K(d, GPS_KEY_MSGOUT_TIM_TP_UART1);
    bool whole_second = fabs(t_epoch - round(t_epoch)) < 1e-6;

    if (whole_second) {
        send_nmea_epoch(d, t_epoch);
    }
    if (pvt && n % pvt == 0) {
        send_nav_pvt(d, t_epoch);
    }
    if (tp && whole_second && n % tp == 0) {
        send_tim_tp(d, round(t_epoch) + 1.0);
    }
    schedule_epoch(d);
}

/* -------------------------------------------------------------- commands */

/* clear: navBbrMask bits (0xFFFF cold, 0x0001 warm, 0 hot). */
static void boot(sim_gps_t *d, uint16_t clear, bool keep_ram)
{
    d->gen++;
    if (!keep_ram) {
        memcpy(d->ram, d->saved, sizeof(d->ram)); /* RAM layer rebuilt from BBR/Flash */
    }
    d->uart.baud = K(d, GPS_KEY_UART1_BAUDRATE);
    d->fix = false;
    d->gnss_stopped = false;
    d->tx_active = false;
    d->ok_since_s = -1.0;
    d->epoch = 0;
    d->rx_state = 0;
    if (clear & 0x0001) {
        d->ephemeris_valid = false;
    }
    if (clear & 0x0080) {
        d->utc_known = false;
    }
    if (clear & 0x0100) {
        d->has_time = false; /* RTC cleared */
    }
    d->utc_due_s = -1.0;
    sim_pin_set(&d->pps, !K(d, GPS_KEY_TP_POL_TP1));
    if (K(d, GPS_KEY_UART1OUTPROT_NMEA) && !keep_ram) {
        /* Boot screen as TXT (interface description 1.2; TXT is a default
         * message, data sheet 5.5). Entries with unknown values are left out. */
        static const char *boot_lines[] = {
            "u-blox AG - www.u-blox.com", "HW UBX 9 00190000", "EXT CORE 4.04 (d964f4)",
            "FWVER=SPG 4.04",             "PROTVER=32.01",     "MOD=NEO-M9N",
            "GPS;GLO;GAL;BDS",            "SBAS;QZSS",
        };
        char line[80];
        for (size_t i = 0; i < sizeof(boot_lines) / sizeof(boot_lines[0]); i++) {
            snprintf(line, sizeof(line), "GNTXT,01,01,02,%s", boot_lines[i]);
            send_nmea(d, line);
        }
    }
    schedule_second(d, floor(sim_now_s() + PRE_EDGE_S) + 1.0);
    schedule_epoch(d);
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/* Things the model does not simulate: accepted (the receiver accepts them),
 * but logged so a test run shows it. */
static void warn_unmodelled(sim_gps_t *d)
{
    if (K(d, GPS_KEY_TP_PULSE_DEF) != 0 || K(d, GPS_KEY_TP_PULSE_LENGTH_DEF) != 1 ||
        K(d, GPS_KEY_TP_PERIOD_TP1) != 1000000 || K(d, GPS_KEY_TP_PERIOD_LOCK_TP1) != 1000000) {
        sim_log(d->base.name, "error", "unmodelled", 0,
                "timepulse other than 1 Hz period/length is not simulated");
    }
}

static void cfg_valset(sim_gps_t *d, const uint8_t *p, uint16_t len)
{
    if (len < 4 || p[0] > 1) {
        ack(d, false, CLS_CFG, ID_CFG_VALSET);
        return;
    }
    uint8_t layers = p[1] & 0x07;
    uint8_t action = p[0] == 1 ? (p[2] & 0x03) : 0;
    if (layers == 0) { /* "does not specify a layer to save a value to" */
        d->tx_active = false;
        ack(d, false, CLS_CFG, ID_CFG_VALSET);
        return;
    }
    if (action == 0 || action == 1 || !d->tx_active) {
        if (action >= 2 && !d->tx_active) { /* invalid transaction transition */
            ack(d, false, CLS_CFG, ID_CFG_VALSET);
            return;
        }
        memset(d->tx_set, 0, sizeof(d->tx_set));
        d->tx_layers = layers;
        d->tx_active = action == 1;
    } else if (layers != d->tx_layers) { /* layers changed within a transaction */
        d->tx_active = false;
        ack(d, false, CLS_CFG, ID_CFG_VALSET);
        return;
    }

    int pairs = 0;
    uint16_t i = 4;
    while (i + 4 <= len) {
        uint32_t key = rd_u32(p + i);
        size_t sz = key_size(key);
        int k = key_index(key);
        if (k < 0 || sz == 0 || i + 4 + sz > len || ++pairs > 64) {
            sim_log(d->base.name, "error", k < 0 ? "unknown_key" : "bad_valset", key, "0x%08X", key);
            d->tx_active = false;
            ack(d, false, CLS_CFG, ID_CFG_VALSET); /* nothing is applied */
            return;
        }
        uint64_t v = 0;
        for (size_t b = 0; b < sz; b++) {
            v |= (uint64_t)p[i + 4 + b] << (8 * b);
        }
        d->tx_val[k] = v;
        d->tx_set[k / 8] |= (uint8_t)(1u << (k % 8));
        i = (uint16_t)(i + 4 + sz);
    }
    if (action == 1 || action == 2) {
        ack(d, true, CLS_CFG, ID_CFG_VALSET); /* staged until action 3 */
        return;
    }
    d->tx_active = false;

    /* Validity is only checked when the RAM layer is written. */
    int kb = key_index(GPS_KEY_UART1_BAUDRATE);
    uint32_t baud = (d->tx_set[kb / 8] >> (kb % 8)) & 1 ? (uint32_t)d->tx_val[kb] : K(d, GPS_KEY_UART1_BAUDRATE);
    if ((layers & 0x01) && (baud < 4800 || baud > 921600)) { /* data sheet table 13 */
        sim_log(d->base.name, "error", "invalid_config", baud, "baud out of range");
        ack(d, false, CLS_CFG, ID_CFG_VALSET);
        return;
    }
    uint32_t old_baud = K(d, GPS_KEY_UART1_BAUDRATE);
    for (int k = 0; k < SIM_GPS_NKEYS; k++) {
        if (!((d->tx_set[k / 8] >> (k % 8)) & 1)) {
            continue;
        }
        if (layers & 0x01) {
            d->ram[k] = d->tx_val[k];
        }
        if (layers & 0x02) {
            d->saved[k] = d->tx_val[k];
        }
        if (layers & 0x04) {
            d->flash[k] = d->tx_val[k];
        }
    }
    sim_log(d->base.name, "cmd", "valset", layers, "%d keys", pairs);
    warn_unmodelled(d);
    ack(d, true, CLS_CFG, ID_CFG_VALSET);
    if (K(d, GPS_KEY_UART1_BAUDRATE) != old_baud) {
        d->uart.baud = K(d, GPS_KEY_UART1_BAUDRATE); /* after the ACK has gone out */
        sim_log(d->base.name, "state", "baud", d->uart.baud, NULL);
    }
}

static void cfg_valget(sim_gps_t *d, const uint8_t *p, uint16_t len)
{
    d->tx_active = false; /* any other CFG message cancels a transaction */
    uint8_t layer = len >= 4 ? p[1] : 0xFF;
    int nkeys = len >= 4 ? (len - 4) / 4 : 0;
    if (len < 4 || p[0] != 0 || (layer != 0 && layer != 1 && layer != 2 && layer != 7) ||
        nkeys > 64) {
        ack(d, false, CLS_CFG, ID_CFG_VALGET);
        return;
    }
    uint16_t skip = (uint16_t)(p[2] | p[3] << 8);
    uint8_t out[4 + 64 * 12];
    out[0] = 1;
    out[1] = layer;
    out[2] = p[2];
    out[3] = p[3];
    uint16_t n = 4;
    int emitted = 0;
    int seen = 0;
    for (int q = 0; q < nkeys; q++) {
        uint32_t key = rd_u32(p + 4 + 4 * q);
        uint32_t group = (key >> 16) & 0x0FFF;
        bool all = group == 0x0FFF;
        bool whole_group = (key & 0xFFFF) == 0xFFFF;
        if (!all && !whole_group && key_index(key) < 0) {
            ack(d, false, CLS_CFG, ID_CFG_VALGET); /* unknown key */
            return;
        }
        for (int k = 0; k < SIM_GPS_NKEYS; k++) {
            uint32_t kk = GPS_KEYS[k].key;
            bool match = all || (whole_group ? ((kk >> 16) & 0x0FFF) == group : kk == key);
            if (!match) {
                continue;
            }
            if (seen++ < skip || emitted == 64) {
                continue;
            }
            uint64_t v = layer == 0 ? d->ram[k]
                         : layer == 1 ? d->saved[k]
                         : layer == 2 ? d->flash[k]
                                      : GPS_KEYS[k].def;
            put_u32(out + n, kk);
            n += 4;
            for (size_t b = 0; b < key_size(kk); b++) {
                out[n++] = (uint8_t)(v >> (8 * b));
            }
            emitted++;
        }
    }
    send_ubx(d, CLS_CFG, ID_CFG_VALGET, out, n);
    ack(d, true, CLS_CFG, ID_CFG_VALGET);
}

/* UBX-CFG-RST: never acknowledged (interface description 3.10.21). */
static void cfg_rst(sim_gps_t *d, const uint8_t *p)
{
    d->tx_active = false;
    uint16_t mask = (uint16_t)(p[0] | p[1] << 8);
    uint8_t mode = p[2];
    const char *start = mask == 0 ? "hot" : mask == 1 ? "warm" : "cold";
    sim_log(d->base.name, "cmd", "reset", mode, "%s start, mode 0x%02X", start, mode);
    switch (mode) {
    case 0x08: /* controlled GNSS stop */
        d->gnss_stopped = true;
        if (d->fix) {
            d->fix = false;
            sim_log(d->base.name, "state", "fix", 0, "lost: GNSS stopped");
        }
        break;
    case 0x09: /* controlled GNSS start */
        d->gnss_stopped = false;
        if (mask & 0x0001) {
            d->ephemeris_valid = false;
        }
        if (mask & 0x0080) {
            d->utc_known = false;
        }
        d->ok_since_s = -1.0;
        break;
    case 0x02: /* software reset, GNSS only: configuration kept */
        boot(d, mask, true);
        break;
    default: /* 0x00 / 0x04 hardware reset, 0x01 software reset: reload config */
        boot(d, mask, false);
        break;
    }
}

static void handle_frame(sim_gps_t *d, uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len)
{
    if (!K(d, GPS_KEY_UART1INPROT_UBX)) {
        return;
    }
    if (cls == CLS_CFG && id == ID_CFG_VALSET) {
        cfg_valset(d, p, len);
    } else if (cls == CLS_CFG && id == ID_CFG_VALGET) {
        cfg_valget(d, p, len);
    } else if (cls == CLS_CFG && id == ID_CFG_RST && len == 4) {
        cfg_rst(d, p);
    } else if (len == 0 && cls == CLS_NAV && id == ID_NAV_PVT) {
        send_nav_pvt(d, floor(sim_now_s()));
    } else if (len == 0 && cls == CLS_TIM && id == ID_TIM_TP) {
        send_tim_tp(d, floor(sim_now_s()) + 1.0);
    } else if (len == 0 && cls == CLS_MON && id == ID_MON_VER) {
        send_mon_ver(d);
    } else if (cls == CLS_CFG) {
        d->tx_active = false;
        ack(d, false, cls, id);
    } else {
        sim_log(d->base.name, "error", "unhandled_ubx", id, "class 0x%02X id 0x%02X", cls, id);
    }
}

static void on_rx(sim_uart_t *u, const uint8_t *buf, size_t len, void *user)
{
    (void)u;
    sim_gps_t *d = user;
    for (size_t i = 0; i < len; i++) {
        uint8_t c = buf[i];
        switch (d->rx_state) {
        case 0: d->rx_state = c == 0xB5 ? 1 : 0; break;
        case 1: d->rx_state = c == 0x62 ? 2 : (c == 0xB5 ? 1 : 0); break;
        case 2: d->rx_cls = c; d->rx_cka = d->rx_ckb = 0; d->rx_state = 3; goto ck;
        case 3: d->rx_id = c; d->rx_state = 4; goto ck;
        case 4: d->rx_len = c; d->rx_state = 5; goto ck;
        case 5:
            d->rx_len |= (uint16_t)(c << 8);
            d->rx_pos = 0;
            d->rx_state = d->rx_len > sizeof(d->rx_buf) ? 0 : (d->rx_len ? 6 : 7);
            goto ck;
        case 6:
            d->rx_buf[d->rx_pos++] = c;
            if (d->rx_pos == d->rx_len) {
                d->rx_state = 7;
            }
            goto ck;
        case 7:
            d->rx_state = c == d->rx_cka ? 8 : 0;
            if (c != d->rx_cka) {
                d->bad_frames++;
                sim_log(d->base.name, "error", "ubx_checksum", d->rx_id, NULL);
            }
            break;
        case 8:
            d->rx_state = 0;
            if (c == d->rx_ckb) {
                handle_frame(d, d->rx_cls, d->rx_id, d->rx_buf, d->rx_len);
            } else {
                d->bad_frames++;
                sim_log(d->base.name, "error", "ubx_checksum", d->rx_id, NULL);
            }
            break;
        default: d->rx_state = 0; break;
        }
        continue;
    ck:
        d->rx_cka = (uint8_t)(d->rx_cka + c);
        d->rx_ckb = (uint8_t)(d->rx_ckb + d->rx_cka);
    }
}

/* ---------------------------------------------------------------- setup */

sim_gps_config_t sim_gps_default_config(void)
{
    sim_gps_config_t c;
    memset(&c, 0, sizeof(c));
    c.gps_start_s = 1500000000.0; /* mid-2027 GPS time, whole second */
    c.leap_s = 18;
    c.cold_ttff_s = 24.0;   /* data sheet table 1, GPS+GLO+GAL+BDS (default) */
    c.hot_reacq_s = 2.0;    /* data sheet table 1 hot start */
    c.max_accel_g = 4.0;
    c.nav_latency_s = 0.040; /* UNVERIFIED: not in the data sheet */
    c.lat_deg = sim_signal_const(47.9655); /* launch-site-like placeholder */
    c.lon_deg = sim_signal_const(-81.8710);
    c.alt_m = sim_signal_const(1400.0);
    c.vel_n = c.vel_e = c.vel_d = sim_signal_const(0.0);
    c.sky = sim_signal_const(1.0);
    c.num_sv = sim_signal_const(18.0);
    c.antenna_cable_delay_ns = 50.0; /* = the receiver's default compensation: exact PPS */
    c.rx_clock_hz = 48e6;    /* UNVERIFIED: timepulse granularity not in the data sheet */
    c.rx_clock_ppb = 37.0;   /* arbitrary: only sets how fast the sawtooth walks */
    return c;
}

void sim_gps_init(sim_gps_t *dev, const char *name, const sim_gps_config_t *cfg)
{
    memset(dev, 0, sizeof(*dev));
    sim_device_init(&dev->base, name);
    dev->cfg = *cfg;
    sim_uart_init(&dev->uart, name, 38400);
    dev->uart.on_rx = on_rx;
    dev->uart.user = dev;
    sim_pin_init(&dev->pps, name, cfg->pps_when_off);
    for (int i = 0; i < SIM_GPS_NKEYS; i++) {
        dev->saved[i] = dev->flash[i] = GPS_KEYS[i].def;
    }
    dev->base.powered = false;
    dev->uart.base.powered = false;
    dev->utc_due_s = -1.0;
    dev->off_since_s = -1e12; /* never powered: first start is cold */
}

void sim_gps_power(sim_gps_t *dev, bool on)
{
    if (on == dev->base.powered) {
        return;
    }
    sim_device_power(&dev->base, on);
    sim_device_power(&dev->uart.base, on);
    if (on) {
        /* Integration manual 3.15: with backup power (V_BCKP) BBR survives:
         * hot start if off for 4 h or less, warm after that (ephemeris out of
         * date). Without it: cold start, configuration from Flash/defaults. */
        uint16_t clear = 0xFFFF;
        if (dev->cfg.backup_power) {
            clear = (sim_now_s() - dev->off_since_s > 4.0 * 3600.0) ? 0x0001 : 0x0000;
        } else {
            for (int i = 0; i < SIM_GPS_NKEYS; i++) {
                dev->saved[i] = dev->flash[i];
            }
        }
        boot(dev, clear, false);
    } else {
        dev->gen++;
        dev->fix = false;
        dev->off_since_s = sim_now_s();
        sim_pin_set(&dev->pps, dev->cfg.pps_when_off);
    }
}
