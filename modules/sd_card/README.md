# modules/sd_card

SD card logging. Starting point: payload-2026 `SD_card.c` (FatFs over SPI, `user_diskio_spi.c`).

FatFs itself is portable. Only its disk I/O layer touches hardware, so port that layer behind an I/O struct like the sensor modules (see [docs/driver_conventions.md](../../docs/driver_conventions.md)).

The log record format still needs to be decided.
