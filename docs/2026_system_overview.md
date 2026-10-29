# payload-2026: how last year's system worked

Background for the 2027 driver debrief. Based on the latest `main` of
[payload-2026](https://github.com/McGillRocketTeam/payload-2026) (last commit Aug 11, 2026), mainly the `v8.1/` folder.

payload-2026 has **no GPS, TDC, PPS or clock code**. Last year's payload measured launch vibration. The GPS and TDC drivers are new for 2027.

## The big picture

There were two boards, linked by CAN:

```
 4× ADXL345 (digital, SPI)                  IEPE accelerometer (analog x/y/z)
          │                                              │
 ┌────────▼─────────┐   CAN (raw samples)    ┌───────────▼──────────┐   CAN    ┌────┐
 │  ADAPTER board   │ ─────────────────────► │      IEPE board      │ ◄──────► │ FC │
 │  STM32F303CB     │   IDs 0x100–0x103      │     STM32F446RE      │  CAN1    └────┘
 │  v8.1/adapter_v2 │                        │  v8.1/iepe_v8.1      │
 └────────┬─────────┘                        └───────────┬──────────┘
          │ SD card (SPI)                                │ SD card (SPI)
       dataN.pld                                     dataN.pld
```

- **The IEPE board was the main one.** It read an analog IEPE vibration sensor, ran an FFT, logged to SD, and was the only board that talked to FC.
- **The adapter board** read four ADXL345 digital accelerometers, logged them to its own SD card, and streamed raw samples to the IEPE board. The IEPE board forwarded a small fraction of them to FC.
- **Both were a superloop**, with no RTOS. Interrupts only set flags or fill buffers, and the `while(1)` loop in `main.c` does the real work.

## Board 1: IEPE (`v8.1/iepe_v8.1/Core/`)

| File | What it does |
|---|---|
| `Src/main.c` | Starts everything, then runs the main loop: write to SD, run the FFT, handle FC commands, send telemetry |
| `Src/ADC.c` | Starts the three ADCs in triple-synchronous mode (one per axis), with DMA filling a buffer. A timer (TIM8) triggers each conversion |
| `Src/accelerometer.c` | `Record()` splits the DMA buffer into x/y/z arrays. `Analyze()` runs an FFT per axis (ARM CMSIS-DSP). `PeakFrequency()` finds the strongest frequency |
| `Src/SD_card.c` | FatFs on the SD card. Opens the next free `dataN.pld` and writes tagged binary packets |
| `FATFS/Target/user_diskio_spi.c` | Low-level SD-over-SPI driver: kiwih's port of ChaN's FatFs sample code (third-party) |
| `Src/CAN_bus.c` | Two buses. CAN1 goes to FC (receive commands, send telemetry). CAN2 goes to the adapter (receive raw frames, forward some to FC) |
| `Src/serial_monitor.c` | `PL_Log(category, status, fmt, ...)`: coloured, filterable debug prints over UART4 |
| `Src/blink.c` | Heartbeat LED on TIM9 |
| `Inc/enabled.h` | Compile-time switches (`CAN_BUS_ENABLED`, `SD_CARD_ENABLED`, ...). A disabled feature's functions do nothing and report success |

How data flows:
1. TIM8 triggers the ADCs, and DMA fills `accelerometer_buffer` in the background.
2. **Ping-pong buffering.** At half full, `HAL_ADC_ConvHalfCpltCallback` copies the first half out; at full, `HAL_ADC_ConvCpltCallback` copies the second half. One half is always being filled while the other is processed. The copy sets `analysis_ready`.
3. When `analysis_ready` is set, the main loop:
   - writes an `ACC!` packet (raw samples) and a `TEL!` packet (ok and sampling flags) to SD;
   - runs the FFT and finds the peak frequency and amplitude per axis;
   - every second cycle, sends three CAN frames to FC.
4. A CAN receive interrupt sets `command_ready`, and the main loop acts on the command.
5. At boot it runs an **SD self-test**: write three known packets to `selftest.pld`, read them back, compare byte by byte. Setting `SELFTEST_INJECT_FAULT` to 1 corrupts a packet on purpose, to prove the check can fail.

## Board 2: adapter (`v8.1/adapter_v2/Core/`)

`v8.1/adapter/` is an older version; ignore it.

| File | What it does |
|---|---|
| `Src/main.c` | Most of the logic: ADXL345 register read/write, the sampling interrupts, the main loop |
| `Src/accelerometer.c` | Same FFT code as the IEPE board, trimmed to fit 128 KB of flash |
| `Src/SD_card.c` | Same design as IEPE, but each packet holds `sensor_id` plus int16 x/y/z samples |
| `Src/CAN_bus.c` | `PL_CANBus_SendRaw()` streams every sample to the IEPE board, one CAN frame each: x, y, z, sample index |

How it works:
1. At boot, `adxl345_init_all()` reads each sensor's **DEVID** register and expects `0xE5`, then sets 3200 Hz output rate and measure mode. A bad DEVID is only printed, never acted on.
2. A TIM1 interrupt fires at 12 kHz. Each time it starts a non-blocking 7-byte SPI read of one sensor, starting at `DATAX0`.
3. `HAL_SPI_TxRxCpltCallback` stores the sample and moves to the next sensor (chip selects BO1–BO4 in turn): 3 kHz per sensor, into a ping-pong buffer.
4. When a half is full, the main loop sends it over CAN, writes it to SD and runs the FFT.

## Connection to the rest of the rocket (CAN IDs)

| Direction | ID | Contents |
|---|---|---|
| FC → IEPE | `0x05` | `RESET_PAYLOAD` (calls `NVIC_SystemReset()`) |
| FC → IEPE | `0x11` | `TOGGLE_SAMPLING` (start or stop sampling) |
| IEPE → FC | `0x300` | ok and sampling flags |
| IEPE → FC | `0x301` | peak frequency x/y/z, amplitude x |
| IEPE → FC | `0x302` | amplitude y/z, time since boot |
| Adapter → IEPE | `0x100`–`0x103` | One raw x/y/z sample per frame, per sensor |
| IEPE → FC | `0x400`–`0x403` | One forwarded adapter sample per sensor, from roughly 1 in 11 bursts, so FC isn't flooded |

Messages are packed C structs inside unions with `uint8_t bytes[8]`, so the struct layout *is* the CAN payload. FC must decode it with the same layout.

## SD format and parsers (`utils/`)

- Each record is a 4-byte tag (`ACC!` or `TEL!`) followed by a packed struct, little-endian. Files are `data0.pld`, `data1.pld`, ... at the card root.
- `utils/pld_parser` (IEPE) and `utils/adapter_parser` (adapter) are small C programs that turn `.pld` files into CSV.
- **Weak point:** the sample count isn't stored in the file, so each parser hard-codes a copy that must match the firmware. Change one side only and the output is garbage.

## What to reuse in 2027

| | Item | Why |
|---|---|---|
| **Keep the design** | SD logging: FatFs + kiwih SPI diskio, tagged packed packets, auto-numbered files | Holdover and PPS data needs logging the same way. Port it to the I/O-struct pattern |
| **Keep the design** | Boot-time SD self-test with fault injection | Cheap, catches bad cards before flight |
| **Keep the design** | `.pld` → CSV parser approach | New parser for the 2027 format. **Store record sizes in the file this time** |
| **Keep the design** | Superloop: interrupts set flags or fill buffers, main loop does the work | Same structure fits the TDC interrupt and GPS UART (unless we switch to FreeRTOS) |
| **Keep the design** | Ping-pong buffers | For continuous streams: GPS UART bytes, possibly the SDR |
| **Keep the design** | `serial_monitor.c` (`PL_Log`) | Good debug tool. Only `__io_putchar` touches hardware, so it ports easily |
| **Keep the design** | `enabled.h` feature switches | Useful for bench testing one subsystem at a time |
| **Teaching example** | ADXL345 `read_reg` / `write_reg` and DEVID check (`adapter_v2/Core/Src/main.c` lines 739–786) | Closest thing to the TDC7200 work. "Read a register with a known reset value" is exactly the DEVID check |
| **Maybe** | CAN protocol with FC (`0x05` reset, `0x11` toggle sampling) | 2027 recording is started and stopped by command, which is what `TOGGLE_SAMPLING` did. Reuse if payload still talks to FC over CAN; ask FC |
| **Maybe** | Accelerometer and FFT | Only if 2027 also logs launch vibration (v8 ground-test boards log accelerometer data) |
| **Leave** | Triple-ADC IEPE sampling, adapter→IEPE relay | Specific to last year's sensors |
| **Leave** | `Drivers/` (BME280, peltier, PID, temperature) | From an earlier temperature-control payload, not v8.1 |

## Questions members might ask

- **"Why is everything in `main.c`?"** CubeMX's workflow puts user code in `main.c`, and the adapter grew there. The 2027 layout (`modules/` plus `MRT/`) avoids this.
- **"Why can't we just copy last year's drivers?"** They call the STM32 HAL directly (`HAL_SPI_*`, `GPIO_TypeDef`, `huart4`) and include family-specific headers (`stm32f3xx.h`), so they only build for that chip.
- **"How did FC start and stop the payload?"** CAN ID `0x11` (`TOGGLE_SAMPLING`).
- **"Why ping-pong buffers?"** So new samples never overwrite data still being processed.
- **"What's the risk with packed structs on SD and CAN?"** Both sides must agree on the exact layout and byte order. Change one side and the other silently misreads it.
- **"Was there GPS or TDC code last year?"** No. Those drivers start from scratch this year.

## Odd spots in the 2026 code (examples of what to improve)

- `PL_Accelerometer_PeakFrequency()` still has a leftover `printf` of the peak index on every call.
- The IEPE conversion macro divides by `65536`, while a comment in `main.c` calls the ADC 12-bit (0–4095). One is probably wrong.
- The adapter comment says 256 samples per half-buffer, but the real value is 32 (`ACCELEROMETER_SAMPLE_SIZE_SINGLE = 64`). The parser README confirms 32.
- CAN sends busy-wait. The adapter sends 128 frames per half-buffer, each waiting up to 5 ms.
- `SD_FLUSH_BYTES` is 0, so the file syncs after every write: safe, but slow.

## Not confirmed

- **Which version actually flew.** The README mentions a `legacy/` folder for flown code, but it doesn't exist. Ask whoever led payload-2026.
- **Exact CAN bitrate and timer clock rates.** They depend on clock settings not fully traced; the sample rates above come from the code's constants and comments.
