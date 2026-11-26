#include "sim_board.h"

#include <string.h>

#include "sim_log.h"

static const char *TDC_NAMES[SIM_TDC_COUNT] = {"tdc_gps",   "tdc_csac2", "tdc_mac1",  "tdc_mac2",
                                               "tdc_tcxo1", "tdc_tcxo2", "tdc_mems1", "tdc_mems2"};

sim_board_config_t sim_board_default_config(sim_board_kind_t kind)
{
    sim_board_config_t c;
    memset(&c, 0, sizeof(c));
    c.kind = kind;
    c.seed = 2027;
    c.sd_root = ".pio/sim/sd";
    c.tdc_clock = SIM_TDC_CLOCK_CSAC_10MHZ;
    c.tdc_stop_delay_s = kind == SIM_BOARD_FLIGHT ? 1e-6 : 0.0; /* ASSUMED value */
    c.mcu_timer_hz = 84e6;
    c.mcu_crystal_ppm = 20.0;
    c.imu_thrust_axis = 2;
    return c;
}

/* Devices that copy a signal by value read the board's live one through this. */
static double live_signal(void *ctx, double t_s)
{
    return sim_signal_eval_clean((const sim_signal_t *)ctx, (sim_time_ns_t)(t_s * 1e9));
}

static void attach_env(sim_board_t *b, sim_osc_config_t *c)
{
    c->temperature_c = &b->temperature_c;
    for (int i = 0; i < 3; i++) {
        c->accel_g[i] = &b->accel_g[i];
    }
}

sim_pin_t *sim_board_pps(sim_board_t *b, sim_tdc_channel_t ch)
{
    switch (ch) {
    case SIM_TDC_GPS: return &b->gps.pps;
    case SIM_TDC_CSAC2: return &b->csac[1].clk.pps.pin;
    case SIM_TDC_MAC1: return &b->mac[0].clk.pps.pin;
    case SIM_TDC_MAC2: return &b->mac[1].clk.pps.pin;
    case SIM_TDC_TCXO1: return &b->rtc[0].mfp;
    case SIM_TDC_TCXO2: return &b->rtc[1].mfp;
    case SIM_TDC_MEMS1: return &b->rtc[2].mfp;
    case SIM_TDC_MEMS2: return &b->rtc[3].mfp;
    default: return NULL;
    }
}

void sim_board_init(sim_board_t *b, const sim_board_config_t *cfg)
{
    memset(b, 0, sizeof(*b));
    b->cfg = *cfg;
    uint64_t seed = cfg->seed;
    bool flight = cfg->kind == SIM_BOARD_FLIGHT;

    b->temperature_c = sim_signal_const(25.0);
    b->accel_g[0] = sim_signal_const(0.0);
    b->accel_g[1] = sim_signal_const(0.0);
    b->accel_g[2] = sim_signal_const(0.0);
    b->accel_g[cfg->imu_thrust_axis] = sim_signal_const(1.0); /* standing on the pad */

    /* MCU crystal and its timebase timer. */
    sim_osc_config_t xtal = sim_osc_ideal();
    xtal.y0 = cfg->mcu_crystal_ppm * 1e-6 * (2.0 * sim_hash_uniform(seed, 900) - 1.0);
    xtal.seed = seed + 900;
    sim_osc_init(&b->mcu_crystal, "mcu_xtal", &xtal, 0.0);
    sim_timer_init(&b->timer, "tim2", &b->mcu_crystal, cfg->mcu_timer_hz);

    /* GPS. */
    sim_gps_config_t gcfg = sim_gps_default_config();
    for (int i = 0; i < 3; i++) {
        gcfg.accel_g[i] = &b->accel_g[i];
    }
    sim_gps_init(&b->gps, "gps", &gcfg);

    /* Atomic clocks. */
    static const char *csac_names[2] = {"csac1", "csac2"};
    static const char *mac_names[2] = {"mac1", "mac2"};
    for (int i = 0; i < 2; i++) {
        sim_csac_config_t cc = sim_csac_default_config(seed + 100 + 10 * i);
        cc.temperature_c = sim_signal_func(live_signal, &b->temperature_c);
        attach_env(b, &cc.clock.free_run);
        attach_env(b, &cc.clock.locked);
        sim_csac_init(&b->csac[i], csac_names[i], &cc);

        sim_mac_config_t mc = sim_mac_default_config(seed + 200 + 10 * i);
        mc.temperature_c = sim_signal_func(live_signal, &b->temperature_c);
        attach_env(b, &mc.clock.free_run);
        attach_env(b, &mc.clock.locked);
        sim_mac_init(&b->mac[i], mac_names[i], &mc);
    }

    /* TCXO / MEMS 32.768 kHz sources and their RTCs. */
    static const char *osc_names[4] = {"tcxo1", "tcxo2", "mems1", "mems2"};
    static const char *rtc_names[4] = {"rtc_tcxo1", "rtc_tcxo2", "rtc_mems1", "rtc_mems2"};
    for (int i = 0; i < 4; i++) {
        sim_osc_config_t oc = i < 2 ? sim_osc_ecs327txo(seed + 300 + i)
                                    : sim_osc_mems_placeholder(seed + 300 + i); /* PENDING part */
        attach_env(b, &oc);
        sim_osc_t *o = i < 2 ? &b->tcxo[i] : &b->mems[i - 2];
        sim_osc_init(o, osc_names[i], &oc, 0.0);
        sim_mcp79511_config_t rc = sim_mcp79511_default_config(o);
        rc.eui48[5] = (uint8_t)(i + 1);
        sim_mcp79511_init(&b->rtc[i], rtc_names[i], &rc);
    }

    /* Sensors and storage. */
    sim_lsm6dsr_config_t ic = sim_lsm6dsr_default_config();
    sim_lsm6dsr_typical_unit(&ic, seed + 500); /* noise, offsets within Table 2 */
    for (int i = 0; i < 3; i++) {
        ic.accel_g[i] = sim_signal_func(live_signal, &b->accel_g[i]);
    }
    ic.temperature_c = sim_signal_func(live_signal, &b->temperature_c);
    sim_lsm6dsr_init(&b->imu, &ic);
    sim_tmp102_config_t tc = sim_tmp102_default_config();
    tc.temperature = sim_signal_func(live_signal, &b->temperature_c);
    sim_tmp102_init(&b->tmp, &tc);
    sim_sd_config_t sc = sim_sd_default_config(cfg->sd_root);
    sim_sd_init(&sc);

    /* TDCs. */
    sim_osc_t *tdc_ref = cfg->tdc_clock == SIM_TDC_CLOCK_CSAC_10MHZ ? &b->csac[0].clk.osc
                                                                    : &b->mcu_crystal;
    double tdc_hz = cfg->tdc_clock == SIM_TDC_CLOCK_CSAC_10MHZ ? 10e6 : 8e6;
    b->n_tdc = flight ? SIM_TDC_COUNT : 1;
    for (int i = 0; i < b->n_tdc; i++) {
        sim_tdc7200_config_t td = sim_tdc7200_default_config(tdc_ref, tdc_hz);
        td.seed = seed + 400 + i;
        td.stop_delay_s = cfg->tdc_stop_delay_s;
        sim_tdc7200_init(&b->tdc[i], TDC_NAMES[i], &td);
        sim_tdc7200_attach_start(&b->tdc[i], &b->csac[0].clk.pps.pin);
        sim_tdc7200_attach_stop(&b->tdc[i], sim_board_pps(b, (sim_tdc_channel_t)i));
    }

    /* 1PPS references: GPS into the atomic clocks it disciplines. */
    sim_csac_attach_ref(&b->csac[0], &b->gps.pps);
    if (flight) {
        sim_csac_attach_ref(&b->csac[1], &b->gps.pps);
        sim_mac_attach_ref(&b->mac[0], 0, &b->gps.pps);
        sim_mac_attach_ref(&b->mac[1], 0, &b->gps.pps);
    }

    /* Power at t = 0 (MACs stay off: they are unpowered on the pad). */
    sim_gps_power(&b->gps, true);
    sim_csac_power(&b->csac[0], true);
    if (flight) {
        sim_csac_power(&b->csac[1], true);
    } else {
        for (int i = 0; i < 4; i++) {
            sim_mcp79511_power(&b->rtc[i], false);
        }
    }
    sim_log("board", "state", flight ? "flight" : "ground_test", (double)b->n_tdc, "%d TDC(s)",
            b->n_tdc);
}
