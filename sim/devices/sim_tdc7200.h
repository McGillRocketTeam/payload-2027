#ifndef SIM_DEVICES_SIM_TDC7200_H_
#define SIM_DEVICES_SIM_TDC7200_H_

/*
 * Simulated TI TDC7200 time-to-digital converter, per its data sheet
 * (SNAS647D, docs/tdc7200.pdf). Firmware side:
 *   tdc7200_ctx_t.spi_transfer = sim_tdc7200_spi_transfer, handle = &tdc
 *   tdc_io_t.set_enable        = sim_tdc7200_set_enable
 *   INTB (active low)          = tdc.intb   (attach the MCU's EXTI)
 *   TRIGG                      = tdc.trigg
 * START and STOP are wired with sim_tdc7200_attach_start/stop to any sim_pin_t
 * (e.g. CSAC 1PPS on START, the oscillator under test on STOP). They see the
 * exact picosecond edge times. The reference CLOCK is an oscillator, so its
 * frequency error and drift enter the measurement like on the board.
 *
 * Modelled:
 *   - SPI command byte: bit7 auto-increment, bit6 write (1) / read (0),
 *     bits 5-0 address; 8-bit config registers 00h-09h, 24-bit result
 *     registers 10h-1Ch read MSB first; read-only results ignore writes;
 *     no auto-increment: bytes beyond the register are not driven (MISO idle)
 *   - register reset values (Table 1); ENABLE low = shutdown, registers lost;
 *     SPI usable 100 us after ENABLE rises; LDO settling (see below)
 *   - START_MEAS: clears INT_STATUS and results, TRIGG pulse, arms START
 *   - measurement mode 1 (ring oscillator + coarse counter, < 500 ns) and
 *     mode 2 (fine START->CLOCK, clock counter, fine STOP->CLOCK), the
 *     data sheet's TOF equations 1 and 2 hold exactly on the register values
 *   - 1 to 5 STOPs (NUM_STOP), START/STOP edge polarity, CLOCK STOP mask
 *   - coarse and clock counter overflow (measurement stops immediately),
 *     FORCE_CAL, calibration CALIBRATION1 (1 period) / CALIBRATION2 (2, 10,
 *     20 or 40 periods)
 *   - multi-cycle averaging (2-128): CLOCK_COUNTn summed, TIMEn averaged
 *   - INT_STATUS bits (write 1 to clear), INT_MASK, INTB, even parity
 *   - errors logged: polling INT_STATUS during a measurement (the data sheet
 *     says it interferes), edges closer than the data sheet minimums,
 *     reference clock outside 1-16 MHz
 *
 * Numbers: LSB 55 ps typical; mode 2 standard deviation 35 ps at 1 us, 50 ps
 * at 100 us; start-up 100 us / 300 us / 1.5 ms (data sheet 7.5, 8.4.7). The
 * ring noise below (18 ps per fine measurement, plus LSB quantisation) is
 * derived to reproduce the 35 ps figure.
 * UNVERIFIED: the internal offset delay (0), how long calibration takes after
 * the last STOP (modelled as CALIBRATION2_PERIODS + 3 clock periods), the
 * exact LDO-settling error curve (modelled as up to 0.3 % of tCLOCK before
 * 300 us, fading to 0 at 1.5 ms), how TIMEn are combined when averaging, and
 * what the chip does when the minimum edge spacing is violated (measured
 * anyway, and logged).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensors/drivers/tdc7200/tdc7200.h"
#include "sim_device.h"
#include "sim_osc.h"
#include "sim_pin.h"

#define SIM_TDC_NREGS 0x1D

typedef struct {
    sim_osc_t *clock;        /* reference clock oscillator (NULL: ideal) */
    double clock_hz;         /* nominal CLOCK frequency, 1-16 MHz (8 typical) */
    double lsb_s;            /* ring oscillator LSB, 55 ps typical */
    double noise_rms_s;      /* random error per fine measurement */
    double offset_s;         /* internal delay (cancels in mode 2), UNVERIFIED */
    sim_miso_t miso;
    uint64_t seed;
    /* Board delay line in front of STOP (flight hardware: a fixed known
     * delay, so a STOP slightly ahead of START still arrives after it; the
     * firmware subtracts it). 0 on the ground-test board. */
    double stop_delay_s;
} sim_tdc7200_config_t;

typedef struct {
    sim_device_t base;
    sim_tdc7200_config_t cfg;
    sim_pin_t intb;
    sim_pin_t trigg;

    uint32_t regs[SIM_TDC_NREGS];
    bool enabled;
    sim_time_ps_t enable_ps;

    /* Measurement in progress. */
    enum { TDC_IDLE, TDC_ARMED, TDC_RUNNING, TDC_CALIBRATING } state;
    uint32_t gen;
    sim_time_ps_t start_ps;
    double t_clk1_s;          /* first CLOCK edge after START (mode 2) */
    int stops_seen;
    int cycle;                /* averaging cycle */
    double time_acc[6];
    uint32_t cc_acc[5];
    sim_time_ps_t last_stop_ps;
    uint64_t noise_index;

    /* Statistics. */
    uint32_t measurements;
    uint32_t overflows;
} sim_tdc7200_t;

sim_tdc7200_config_t sim_tdc7200_default_config(sim_osc_t *clock, double clock_hz);
/* Starts with ENABLE low (shut down), as after power-up. */
void sim_tdc7200_init(sim_tdc7200_t *dev, const char *name, const sim_tdc7200_config_t *cfg);

void sim_tdc7200_attach_start(sim_tdc7200_t *dev, sim_pin_t *pin);
void sim_tdc7200_attach_stop(sim_tdc7200_t *dev, sim_pin_t *pin);

/* io functions. handle = sim_tdc7200_t*. */
int sim_tdc7200_spi_transfer(void *handle, const uint8_t *tx, uint8_t *rx, size_t len);
void sim_tdc7200_set_enable(void *handle, bool high);

/* A tdc7200_ctx_t wired to this chip. */
tdc7200_ctx_t sim_tdc7200_ctx(sim_tdc7200_t *dev);

#endif /* SIM_DEVICES_SIM_TDC7200_H_ */
