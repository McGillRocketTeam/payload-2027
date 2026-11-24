# modules/sd_card

SD card logging. Starting point: payload-2026 `SD_card.c` (FatFs over SPI, `user_diskio_spi.c`).

FatFs itself is portable. Only its disk I/O layer touches hardware, so port that layer behind an I/O struct like the sensor modules (see [docs/driver_conventions.md](../../docs/driver_conventions.md)).

Plan (DR2): fixed binary records with a CRC, in 2 x 32 KB ping-pong buffers, synced to the card every second, with a new file per boot. A ground tool converts them to CSV. The exact record layout is still to be decided.
