#ifndef SENSORS_DRIVERS_SA45S_H_
#define SENSORS_DRIVERS_SA45S_H_

/*
 * Microchip SA.45s chip-scale atomic clock (CSAC), serial protocol.
 *
 * Pure protocol code: no UART access. The csac sensor module sends the
 * commands built here and feeds received bytes in.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    /* TODO(workshop): telemetry fields from the SA.45s user guide (BITE / lock
     * status, steer, contrast, heater power, internal temperature, ...). */
    uint8_t placeholder;
} sa45s_telemetry_t;

/* TODO(workshop): build the telemetry request command. Returns its length, or 0 if buf is too small. */
size_t sa45s_build_telemetry_request(uint8_t *buf, size_t max);

/* TODO(workshop): build the command that turns 1PPS disciplining on or off. */
size_t sa45s_build_discipline_command(bool enable, uint8_t *buf, size_t max);

/* TODO(workshop): parse one telemetry response line. Returns true on success. */
bool sa45s_parse_telemetry(const char *line, size_t len, sa45s_telemetry_t *out);

#endif /* SENSORS_DRIVERS_SA45S_H_ */
