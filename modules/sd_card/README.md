# modules/sd_card

SD card logging over **SDIO** (4-bit, with DMA). We use SDIO rather than SPI because the SPI buses are needed for the TDCs and other chips, and SDIO is faster.

FatFs is a library we use, not one we write. With SDIO, CubeMX generates FatFs's disk I/O layer (`sd_diskio.c` on `HAL_SD`), so payload-2026's SPI layer (`user_diskio_spi.c`) is not needed. The payload-2026 `SD_card.c` is still a good reference for the logging logic: tagged packets, auto-numbered files and the boot-time self-test.

Write this module against the FatFs API (`f_open`, `f_write`, `f_sync`, ...). On the laptop it then runs unchanged against the simulated card in `sim/` (see [sim/README.md](../../sim/README.md)), including write errors, stalls and power cuts.

The log record format still needs to be decided.
