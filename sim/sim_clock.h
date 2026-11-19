#ifndef SIM_SIM_CLOCK_H_
#define SIM_SIM_CLOCK_H_

/*
 * What the CSAC (SA.45s) and MAC (SA5x) have in common, written once:
 *
 *   - an oscillator with two regimes: free-running TCXO before atomic lock
 *     (ppm-level offset, poor stability) and atomic after lock
 *   - a 1PPS output derived from it (the RF divided down)
 *   - a phase meter on the 1PPS reference input (ref - own, quantised)
 *   - sync / JamSync: snap the output to the RF cycle nearest the next
 *     reference edge (residual error within half an RF cycle)
 *   - a 1PPS disciplining servo with time constant tau, cable-delay
 *     compensation, a "discipline OK" detector (|phase| < threshold for
 *     2 tau) and holdover when the reference disappears
 *   - frequency steering (user + servo), which acts only while locked
 *
 * The servo is a 2nd-order loop (phase + frequency) tuned to the behaviour
 * both user guides state: error down to ~1/e after one tau, 1/e^2 after two,
 * ~99 % gone after five (CSAC guide 5.1, MAC guide eq. 3-1). Its closed-loop
 * poles are at -1/tau (dominant) and -2.6/tau, which reproduces the CSAC
 * guide's worked example (Figure 5-2: a 50 ns error with tau = 20 s gets an
 * immediate ~-9e-9 steer). Microchip's exact algorithm is not published.
 */

#include <stdbool.h>
#include <stdint.h>

#include "sim_osc.h"
#include "sim_pps.h"

typedef enum {
    SIM_REF_OFF = 0,     /* reference input ignored */
    SIM_REF_MEASURE,     /* phase metering only */
    SIM_REF_DISCIPLINE,  /* steer phase and frequency to the reference */
    SIM_REF_AUTOSYNC,    /* re-sync to every reference edge (CSAC only) */
} sim_ref_mode_t;

typedef enum {
    SIM_DISC_ACQUIRING = 0,
    SIM_DISC_LOCKED = 1,
    SIM_DISC_HOLDOVER = 2,
} sim_disc_state_t;

typedef struct {
    sim_osc_config_t free_run;  /* TCXO before atomic lock */
    sim_osc_config_t locked;    /* atomic */
    double rf_hz;               /* 10 MHz: sync granularity */
    double meter_res_s;         /* phase meter resolution (450 ps) */
    double meter_fine_range_s;  /* beyond this, measure mode uses the coarse meter (0 = never) */
    double meter_coarse_res_s;  /* CSAC extended-range meter: ~100 ns */
    double steer_res;           /* hardware steering resolution (CSAC 1e-12, MAC 1e-14) */
    double disc_limit;          /* servo steering clamp (0 = no limit) */
    double steer_range;         /* total digital tuning range (1e-6 for both parts) */
    double retrace;             /* locked offset changes by up to +-this per power cycle */
    double jam_limit_s;         /* |phase| beyond this forces a re-sync */
    bool jam_restarts_servo;    /* MAC: yes. CSAC: keeps its loop variables */
    sim_time_ps_t pps_width_ps;
} sim_clock_config_t;

typedef struct {
    const char *name;
    sim_clock_config_t cfg;
    sim_osc_t osc;
    sim_pps_t pps;
    bool powered;
    bool locked;
    uint32_t power_ups;
    double locked_y0_cal;      /* calibrated (shipment / latched) offset */

    /* Reference input. */
    sim_ref_mode_t mode;
    sim_time_ps_t last_ref_ps;
    bool ref_seen;
    double phase_s;            /* last measurement: t_ref - t_own (signed) */
    bool phase_valid;
    double cable_s;            /* compensation, + = reference arrives late */

    /* Sync request (CSAC !S, MAC sync). */
    bool sync_pending;
    sim_time_ps_t sync_deadline_ps;
    int sync_result;           /* 0 pending/none, 1 done, -1 timed out */
    uint32_t syncs;

    /* Servo. */
    double tau_s;
    double threshold_s;
    double y_int;
    double steer_user;         /* fractional */
    double steer_disc;         /* fractional */
    double last_correction;
    bool disc_at_limit;        /* servo wanted more than disc_limit */
    bool disc_started;
    double ok_since_s;         /* < 0 = not within threshold */
    sim_disc_state_t disc;
} sim_clock_t;

void sim_clock_init(sim_clock_t *c, const char *name, const sim_clock_config_t *cfg);

/* Power: off stops the PPS. On restarts the TCXO with a random phase. */
void sim_clock_power(sim_clock_t *c, bool on);
/* Atomic lock gained or lost: switches regime and enables steering. */
void sim_clock_set_locked(sim_clock_t *c, bool locked);

/* Wire the 1PPS reference input to a pin. */
void sim_clock_attach_ref(sim_clock_t *c, sim_pin_t *ref);
/* Feed one reference rising edge (if not using attach_ref). */
void sim_clock_ref_edge(sim_clock_t *c, sim_time_ps_t t_ps);

void sim_clock_set_mode(sim_clock_t *c, sim_ref_mode_t mode);
void sim_clock_request_sync(sim_clock_t *c, double timeout_s);
void sim_clock_set_user_steer(sim_clock_t *c, double steer);
/* Fold the current steering into the calibration (latch). */
void sim_clock_latch(sim_clock_t *c);

/* Housekeeping once per own PPS edge: holdover detection, sync timeout. */
void sim_clock_second(sim_clock_t *c);

/* Has a reference edge been seen within the last ~1.5 s? */
bool sim_clock_ref_present(const sim_clock_t *c);

#endif /* SIM_SIM_CLOCK_H_ */
