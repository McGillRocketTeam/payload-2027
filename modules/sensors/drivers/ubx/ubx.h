#ifndef SENSORS_DRIVERS_UBX_H_
#define SENSORS_DRIVERS_UBX_H_

/*
 * u-blox UBX binary protocol (NEO-M9N).
 *
 * Pure protocol code: no UART access. The GPS sensor module feeds received
 * bytes in and sends the frames built here out.
 *
 * Frame: 0xB5 0x62 | class | id | length (2 bytes, little endian) | payload | CK_A CK_B
 * The checksum covers class, id, length and payload (8-bit Fletcher).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62

/* TODO(workshop): compute CK_A and CK_B over data[0..len). */
void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b);

/*
 * TODO(workshop): byte-at-a-time frame parser.
 * Suggested shape: a ubx_parser_t state struct and
 *   bool ubx_parser_feed(ubx_parser_t *p, uint8_t byte);
 * that returns true once a complete frame with a valid checksum is received.
 */

/* TODO(workshop): UBX-NAV-PVT and UBX-TIM-TP decoders. */

#endif /* SENSORS_DRIVERS_UBX_H_ */
