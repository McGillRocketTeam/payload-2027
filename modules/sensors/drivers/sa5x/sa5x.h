#ifndef SENSORS_DRIVERS_SA5X_H_
#define SENSORS_DRIVERS_SA5X_H_

/*
 * Microchip SA5x (SA53) miniature atomic clock (MAC), serial protocol.
 *
 * Pure protocol code: no UART access. The mac sensor module sends the
 * commands built here and feeds received bytes in.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    /* TODO(workshop): telemetry fields from the SA5x user guide (lock status,
     * steer, internal temperature, ...). */
    uint8_t placeholder;
} sa5x_telemetry_t;

/* TODO(workshop): build the telemetry request command. Returns its length, or 0 if buf is too small. */
size_t sa5x_build_telemetry_request(uint8_t *buf, size_t max);

/* TODO(workshop): build the command that turns 1PPS disciplining on or off. */
size_t sa5x_build_discipline_command(bool enable, uint8_t *buf, size_t max);

/* TODO(workshop): parse one telemetry response line. Returns true on success. */
bool sa5x_parse_telemetry(const char *line, size_t len, sa5x_telemetry_t *out);

#endif /* SENSORS_DRIVERS_SA5X_H_ */
