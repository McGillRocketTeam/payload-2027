# sim/: simulated payload hardware

Fake versions of the payload chips, clocks and SD card, for laptop builds only (`env:native`). They plug into the same I/O structs the board code fills in (`tmp102_ctx_t`, `stmdev_ctx_t`, `gps_io_t`, `csac_io_t`, `mac_io_t`, ...). Drivers and the whole pipeline (drivers → parse → pack → ping-pong → SD) run unchanged before any hardware exists. The chip models follow the datasheets and user guides, so a driver that passes against them is written to spec.

Nothing here is ever compiled for the flight board.

## Pieces

| File | What it is |
|---|---|
| `sim.h` | True time in picoseconds, the event queue (how interrupts happen), optional wall-clock pacing, seeded random numbers. |
| `sim_log.h` | The run log: one CSV per run with every fault, state change, error and ground-truth clock offset. |
| `sim_device.h` | Base for every device: power, fault injection, counters, MISO idle level. |
| `sim_signal.h` | What a sensor "measures": constant, ramp, sine, step, table, function or CSV replay, plus noise. |
| `sim_pin.h` | Digital lines (PPS, BITE, alarm, interrupts), one driver and many listeners, with exact edge times. |
| `sim_osc.h` | An oscillator's time error vs true time: offset, aging, white and random-walk FM noise, temperature and g sensitivity, steering. |
| `sim_pps.h` | A 1PPS output derived from an oscillator, with sync to the nearest RF cycle. |
| `sim_timer.h` | The MCU's 32-bit timebase timer with input capture. It is clocked by any oscillator (crystal or CSAC 10 MHz). |
| `sim_clock.h` | What the CSAC and MAC share: free-run/locked regimes, phase meter, sync, disciplining servo, holdover. |
| `sim_uart.h` | UART link with baud-rate timing, a finite receive buffer and overrun counting. |
| `sim_sd.h` + `fatfs/ff.h` | SD card behind the FatFs API, stored in a host folder. |
| `devices/sim_csac.*` | SA.45s CSAC (user guide DS50003041). |
| `devices/sim_mac.*` | SA5x MAC, C3 protocol (user guide DS50002938). |
| `devices/sim_gps.*` | NEO-M9N: NMEA/UBX, CFG-VALSET/VALGET, NAV-PVT, TIM-TP, timepulse, fix model. |
| `sim_board.*` | The whole payload board wired as in DR2 (flight: 8 TDCs, 2 CSAC, 2 MAC, 2 TCXO + 2 MEMS through 4 RTCs; ground test: GPS, CSAC 1, one TDC), sharing one environment (temperature, acceleration). |
| `sim_scenario.*` | Mission timeline on a board: pad wait, MACs on, launch, boost, coast, apogee, descent, landing; drives the IMU, GPS dynamics and oscillator g sensitivity; ready-made actions (GPS sky off/on, SD power cut, MACs on/off); runs the firmware superloop. |
| `devices/sim_mcp79511.*` | MCP79511 RTC: SPI instruction set, RTCC/SRAM/EEPROM/ID, timekeeping from the TCXO/MEMS clock, digital trimming, 1 Hz on MFP (data sheet DS20002300D). |
| `devices/sim_tdc7200.*` | TDC7200: SPI, modes 1 and 2, 1-5 STOPs, STOP mask, overflow, calibration, averaging, INTB/TRIGG, ENABLE start-up. CLOCK from any oscillator (data sheet SNAS647D). |
| `devices/sim_tmp102.*` | TMP102 on I2C. |
| `devices/sim_lsm6dsr.*` | LSM6DSR: registers, rates, full scales, noise / offsets / gain error, 512-word FIFO with tags and watermark, INT1 / INT2 (data-ready latched or pulsed, FIFO flags, batch counter). Works with ST's real driver (data sheet DS11976). |

Each device header lists exactly what is modelled and what is not.

## Time

Sim time is **true physical time**, the perfect reference. GPS seconds land exactly on sim seconds, and every oscillator's error is measured against it. Time only moves when a test calls `sim_advance_*()` or a driver calls `sim_delay_ms`. Runs are deterministic and as fast as the CPU allows: a 3-hour logging test takes seconds. `sim_set_pacing(1.0)` slows the sim to the wall clock, for watching live debug output.

## Interrupts: the event model

Pin edges (PPS, TDC INTB, BITE, alarm, IMU interrupt) and periodic device output (GPS messages) are **events** scheduled at exact picosecond times. While time advances, each event fires in order with sim time set to that instant. A handler attached to a pin (for example through `sim_timer_capture`) therefore runs at the edge, between main-loop steps, the way an interrupt does on the board.

```c
/* Board glue in a test: GPS PPS -> TIM2 CH1 capture -> the module's handler. */
static void on_pps_capture(void *ctx, int ch, uint32_t capture) { gps_on_pps(ctx, capture); }
sim_timer_capture(&tim2, 0, &gps.pps, true, on_pps_capture, &gps_module);
```

The firmware side needs two conventions, still to agree on:
- each module exposes its interrupt handler as a plain function (e.g. `gps_on_pps(dev, capture)`), which the HAL callback calls on the board and the sim calls in tests;
- `MRT_main` is split into `MRT_init()` and `MRT_loop_once()`, so a test can run the superloop.

### Tradeoffs, and how to work around them

| Inaccuracy | Why | Mitigation |
|---|---|---|
| **Firmware code takes zero time.** Only modelled costs (UART bytes, SD writes, stalls, delays) move time. | The sim can't know how long your C takes on a 168 MHz Cortex-M4. | Loop time and CPU margin (the ≥2× logging margin) must be measured on target with the DWT cycle counter. We can add estimated per-call costs to the sim (`sim_advance_ns` in a cost hook) once we have measurements. |
| **Interrupts can't land in the middle of main-loop code.** They fire only when the code under test spends sim time (a delay, a blocking HAL call, a UART write, an SD write). | Events are delivered when time advances, and pure computation doesn't advance it. | This hides races on data shared between ISRs and the main loop (missing `volatile`, torn reads, unprotected ring buffers). Keep shared data to single-producer/single-consumer ring buffers, review them, and test them on target. If needed, a sim build can advance time by a random few µs inside ring-buffer push/pop to force interleavings (fuzzing). |
| **No interrupt priorities or preemption.** A handler runs to completion; edges that arrive meanwhile wait (that *is* modelled, as latency). | Single-threaded event dispatch. | Keep ISRs short (capture and store only), which the design already requires. Priority-sensitive paths get an on-target test. |
| **No interrupt entry latency, no capture synchroniser delay.** A software timestamp read inside an ISR looks perfect in the sim but is later on the board; hardware capture has a fixed 2-3 tick delay (~25-35 ns at 84 MHz). | Not modelled yet. | Take timing only from hardware capture (the design already does). Hardware capture is exact in the sim *and* on the board, up to a constant offset we can calibrate. Adding a fixed or jittered ISR latency to the sim is a small change if we want it. |
| **Too deterministic.** The same seed gives the same run, which can hide bugs that need unlucky timing. | That's the point of reproducible tests. | Run the integration tests under several seeds (`sim_seed`) and with PPS jitter on. Any failure is then reproducible with its seed. |
| **Peripherals are modelled at the transfer level**: UART as a DMA ring, SPI and I2C as whole transfers. | Byte-level DMA and IRQ timing is not useful for drivers and costs a lot. | On-target bring-up covers DMA and IRQ configuration. |

## Clocks

Every clock is a `sim_osc_t`: a time error x(t) against true time, from the integrated frequency error (offset, aging, white FM set by the 1 s Allan deviation, random-walk FM, temperature coefficient driven by any temperature signal, g sensitivity driven by the IMU's acceleration signals, plus steering). Noise is keyed to a fixed 100 ms grid and a per-oscillator seed, so results don't depend on how often or in what order clocks are queried.

- **GPS PPS:** exactly on true seconds. With `qerr_enabled`, it lands on the receiver's own clock tick instead (a few ns sawtooth), reported in TIM-TP qErr like the real part.
- **CSAC / MAC:** a free-running TCXO until atomic lock, then atomic. Sync and JamSync re-phase the PPS to the RF cycle nearest the reference edge (within ±50 ns). Disciplining is a critically damped 2nd-order loop with the user's tau. Microchip's actual algorithm isn't published.
- **MCU timer:** counts at its nominal rate as seen by its clock source, so a crystal 20 ppm fast gives 84 MHz × (1 + 20e-6) ticks per GPS second.

### Where the numbers come from

Every number in a default config cites its source in a comment. `UNVERIFIED` marks the few that no document in `docs/` gives.

| Part | Verified (source) | Not in any document (placeholder or 0) |
|---|---|---|
| CSAC SA.45s | ADEV 3e-10 at 1 s → 1e-11 at 1000 s, aging <9e-10/month, ±5e-10 over −10…70 °C, shipment offset ±5e-11, retrace ±5e-10, digital tuning ±1e-6 at 1e-12, warm-up <180 s, sync ±100 ns, 1PPS 100 µs, 57600 baud (data sheet 900-00744). Unlocked TCXO <1e-8, ~30e-9/°C; 450 ps / 100 ns phase meters; protocol, limits, BITE timing (user guide). | Acceleration sensitivity (0), unlocked TCXO noise. The data sheet is the *Space* CSAC's: confirm it matches the variant we fly. |
| MAC SA53 | ADEV 3e-11 at 1 s → 1e-12 at 1000 s → 3e-12 at 10⁴ s, drift <1e-10/month, TempCo 1e-10 p-p, offset and retrace ±5e-11, tuning ±1e-6, lock <8 min, 1PPS 20 µs (data sheet DS00003348E). Unlocked ±2 ppm, 450 ps meter, +75 °C baseplate limit, protocol (user guide). | Acceleration sensitivity (0), unlocked TCXO noise and tempco, how long "eventually loses lock" takes above 75 °C (60 s). |
| NEO-M9N | 38400 8N1 and the default NMEA set, cold 24 s / hot 2 s, 4 g / 80 km / 500 m/s limits, timepulse 30 ns RMS, baud 4800–921600 (data sheet UBX-19014285). All 582 configuration keys with types and defaults (`sim_gps_keys.inc`, generated), the NAV-PVT / TIM-TP / MON-VER / ACK layouts, VALSET/VALGET/RST rules, default UTC time grid, 50 ns cable-delay compensation, boot-screen strings (interface description UBX-21022436). Dynamic-model limits, hot start within 4 h on backup power, warm = cold-start time, no UTC-grid pulse until the UTC parameters arrive (≤12.5 min), UTC fields rounded to 0.01 s, reset types, delay signs (integration manual UBX-19014286). | qErr sign and granularity, NAV-PVT latency, the g limit of non-airborne models (data sheet's 4 g used), when confirmedDate/Time are set. |
| LSM6DSR | Sensitivities, ODR / BDR tables, noise 60 µg/√Hz and 5 mdps/√Hz, zero-g ±10 mg (±0.1 mg/°C), zero-rate ±1 dps, gyro gain ±1 %, FIFO 3 kbytes = 512 words, tag codes, WTM / STOP_ON_WTM, FIFO and Continuous modes, status flag meanings, OVR_LATCHED cleared on read, INT routing, INT2_on_INT1, H_LACTIVE, data-ready latched or 75 µs pulses (data sheet DS11976 Rev 2). | When a FIFO word is popped, TAG_PARITY rule, order of words in one time slot, temperature-word layout, batching above the ODR. Compression, timestamps and trigger modes are not modelled. |
| ECS-327TXO | ±1.5 ppm at 25 °C, ±5 ppm over −40…85 °C, ±3 ppm first-year aging (`sim_osc_ecs327txo`). | Noise (0). |
| DSC60xxB MEMS | **PENDING: exact part number.** Placeholder in use: DSC6001CI2B-032K768 (±25 ppm, −40…85 °C), `sim_osc_mems_placeholder`. Stability grade ±20/25/50 ppm total, aging ±5 ppm first year, start-up 1.5 ms (data sheet DS20006133). | The real ordering code; noise at 32.768 kHz (jitter only specified at 27 MHz). |
| TDC7200 | Register map and reset values, SPI command byte, 55 ps LSB, 35 ps std at 1 µs in mode 2 (reproduced by a test), 1–16 MHz clock, mode 1 ≤500 ns, mode 2 ≤(2¹⁶−2) tCLOCK, minimum edge spacings, start-up 100 µs / 300 µs / 1.5 ms, the TOF equations and worked examples (data sheet SNAS647D). | Internal offset delay (0), calibration duration after the last STOP, exact LDO-settling error curve, how TIMEn combine when averaging, behaviour when edge spacing is violated. |
| MCP79511 | Instruction set, memory map and block wrap, POR values, EEPROM pages / BP / WEL / 5 ms write cycle, unlock sequence, EUI-48 at 02h–07h, BCD carry rules, hundredths algorithm, OSCRUN timing, trim of 2 × TRIMVAL cycles per minute or per second (data sheet DS20002300D). | Position and duty cycle of the 1 Hz edge, which second gets the minute trim, first tick 1 s after start, MFP level outside square-wave mode, ST with an external clock. Alarms, VBAT and timestamps are not modelled. |
| Board | — | ASSUMED, to confirm with hardware: TDC STOP channel order, TDC reference = CSAC 1 10 MHz, STOP delay line 1 µs on the flight board, MCU timer 84 MHz, crystal ±20 ppm, IMU thrust axis Z, which parts the ground-test board has. |

TDC consequences: mode 2 only measures from a START forward to the next STOP, up to (2¹⁶−2) clock periods (8.2 ms at 8 MHz, 6.6 ms at 10 MHz), and needs at least 2 clock periods (200 ns at 10 MHz) from START to STOP. The flight board has a known delay line on STOP (`tdc_stop_delay_s`), which the firmware subtracts; the ground-test board has none. **After `!S` the CSAC is within ±50 ns of GPS, so without a delay line the GPS edge is either before START (missed) or closer than 200 ns (timing violation)**: the ground-test TDC can only measure GPS vs CSAC before sync, or by phase-shifting one of them. `test_sim_board` shows both cases. The CSAC's own phase meter (phase measurement mode) agrees with the TDC to < 1 ns and is an alternative for the ground test.

MCP79511 consequence: a non-zero OSCTRIM makes one second per minute 2 × TRIMVAL cycles (61 µs each) shorter or longer, which shows up as a phase step on the TCXO / MEMS PPS. Leave trimming off and correct in post-processing, or account for it in the TDC data.

GPS consequence for the flight: the default **Portable** model invalidates the fix above 50 m/s vertical speed, so the climb kills the fix whatever the g-load. **Airborne <4g** only checks altitude (80 km) and acceleration (4 g). The default **UTC** timepulse grid can also delay the first PPS by up to 12.5 minutes after a cold start; the GPS grid, or backup power for hot starts, avoids that.

Conflicts between documents, and what the sim does:
- **MAC ALARM polarity:** the user guide says active low, the data sheet says "High = Alarm". `alarm_active_high` defaults to the guide. Check it on the real unit.
- **MAC steering resolution:** the guide says ~1e-14 realised, the data sheet 1e-15. The sim uses the guide's 1e-14.
- **CSAC disciplining speed:** the guide's Figure 5-2 numbers can't all hold with its own "1/e per tau". The servo matches the −9e-9 initial step and the ±5 ns phase, and reaches ~6e-13 rather than 5e-13 at 7 tau.

No acceleration sensitivity is published for either atomic clock. The only related figures are "no loss of lock" at 7.7 g RMS vibration, and ≤4e-8 momentary frequency change under a 30 g shock for the MAC. It defaults to 0. To study boost effects, set `g_sens` and point `accel_g` at the IMU signals.

## Faults

Every device has the same controls through its `base`:

```c
sim_inject(&dev.base, SIM_FAULT_OFF);                          /* from now on */
sim_inject_for(&dev.base, SIM_FAULT_BUS_ERROR, SIM_MS(200));   /* for 200 ms */
sim_inject_at(&dev.base, SIM_FAULT_OFF, SIM_S(60), SIM_S(5));  /* at t = 60 s, for 5 s */
sim_inject_next(&dev.base, SIM_FAULT_CORRUPT, 3);              /* next 3 transfers */
sim_inject_random(&dev.base, SIM_FAULT_STALL, 0.01)->stall_ns = SIM_MS(250);
sim_clear_faults(&dev.base);
```

| Mode | Meaning |
|---|---|
| `OFF` | Not answering: I2C NACK, UART silent. SPI has no acknowledge, so reads return the MISO idle level (`sim_miso_t`, 0xFF by default) and HAL reports success. |
| `BUS_ERROR` | The transfer returns an error (HAL error or timeout). |
| `CORRUPT` | The transfer "works", but one random bit is flipped. |
| `STUCK` | Answers normally, but the measurement freezes. |
| `STALL` | Works, but takes `stall_ns` longer. |

Device-level faults have their own calls: `sim_csac_power`, `sim_mac_power`, `sim_gps_power` (unplug and replug, with real warm-up and lock times), `sim_csac_lose_lock`, `sim_mac_lose_lock`, `sim_mac_fail_acquisition`, and `sim_sd_power_cut`. Environmental faults are signals: GPS `sky` (antenna blocked) and acceleration (boost above the dynamic model limit).

## Run log

```c
sim_log_open(".pio/sim/full_mission.csv");    /* folders are created */
/* ... run ... */
sim_log_close();                               /* also prints a per-source summary */
```

`t_s,source,kind,name,value,detail`, one row per event, with time to the picosecond:

| kind | Rows |
|---|---|
| `inject` / `fault` | A fault was injected, and every transfer it hit. |
| `power`, `state` | Power changes, CSAC status, atomic lock, BITE, sync, discipline state, GPS fix (with the reason it was lost), MAC alarms, baud changes. |
| `truth` | Each PPS edge's real offset from the true second. Compare it with what the firmware measured and logged. |
| `error` | Something lost or rejected: UART overrun, missed IMU samples, bad command or checksum, UBX NAK, sync timeout, event queue full. |
| `cmd` | Every command a device received. |
| `edge` | Every pin edge, only with `sim_log_edges(true)` (it is a lot of rows). |

`sim_log_count(source, kind)` gives the same counts in an assert, e.g. `TEST_ASSERT_EQUAL(0, sim_log_count("gps", "error"))`. The sim's own test suites write one CSV per test into `.pio/sim/`.

## SD card

```c
sim_sd_config_t cfg = sim_sd_default_config(".pio/sim/my_test");
cfg.write_bytes_per_s = 2e6;        /* optional: throughput in virtual time */
sim_sd_init(&cfg);

/* firmware code, unchanged: */
f_mount(&fs, "", 1);
f_open(&fil, "0:/log0.bin", FA_WRITE | FA_CREATE_ALWAYS);
f_write(&fil, rec, sizeof(rec), &bw);
f_sync(&fil);

sim_sd_power_cut();                 /* everything after the last f_sync is gone */
```

The mock sits at the FatFs API, so it covers SDIO and SPI alike. The real card uses SDIO with the CubeMX-generated FatFs disk layer.

## Tips

- Each chip model has a `hook` member that runs before the model and can answer any transfer itself.
- Set `dev.base.trace = true` (or call `sim_trace_all(true)`) to print every bus transfer.
- `sim_uart_t` and the devices that contain one are large (a 16 KB queue). Declare them `static`, not on the stack.

## Status

Every part on the DR2 block diagram is simulated: GPS, CSAC ×2, MAC ×2, TCXO ×2, MEMS ×2 (through MCP79511 ×4), TDC7200 ×8, LSM6DSR, TMP102, SD card and the MCU timebase, plus the board harness and the mission scenario runner. The MAX2771 / SDR is out of scope.

Open items (none block writing drivers):
- **PENDING:** the MEMS ordering code (placeholder DSC6001CI2B-032K768).
- **ASSUMED board wiring** (`sim_board.h`), to confirm with hardware.
- **Values no document gives**, marked UNVERIFIED in each header: atomic-clock g sensitivity, GPS qErr sign and granularity, TDC calibration timing, RTC edge placement, LSM6DSR FIFO pop timing.
- **MAC ALARM polarity:** the guide and data sheet disagree. Check on a real unit.
- **CSAC variant:** the data sheet in `docs/` is the Space CSAC's.

Note: device statistics (such as `imu.fifo_overruns`) are computed lazily and are only up to date after the device has been accessed. The run log and interrupt pins are always on time.
