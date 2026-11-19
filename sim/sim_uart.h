#ifndef SIM_SIM_UART_H_
#define SIM_SIM_UART_H_

/*
 * A simulated UART link between the MCU and a serial device (GPS, CSAC, MAC).
 * Its read/write functions match the uart_read / uart_write members of
 * gps_io_t, csac_io_t and mac_io_t; pass the sim_uart_t* as the handle.
 *
 * Device models sit on the other end:
 *   - produce(): called before every MCU read with the current time, so the
 *     device can emit whatever it would have sent by now (periodic messages)
 *   - on_rx():   called with every chunk the MCU writes (commands)
 *
 * Timing is real: at `baud`, each byte takes 10 bit times (8N1) on the wire,
 * so a burst arrives gradually and an MCU that reads too rarely overflows the
 * receive buffer (rx_buffer bytes, like a DMA ring). Overrun drops the oldest
 * bytes, as a circular DMA does, and is counted.
 *
 * Faults: OFF -> device side silent, MCU writes go nowhere;
 * BUS_ERROR -> MCU write returns -1, device chunk lost (framing error);
 * CORRUPT -> a bit flipped in a device chunk; STALL -> chunk delayed.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sim_device.h"

#define SIM_UART_QUEUE 16384

typedef struct sim_uart sim_uart_t;

struct sim_uart {
    sim_device_t base;
    uint32_t baud;
    size_t rx_buffer;    /* MCU side buffer, bytes (default 512) */

    void (*produce)(sim_uart_t *u, sim_time_ns_t now, void *user);
    void (*on_rx)(sim_uart_t *u, const uint8_t *buf, size_t len, void *user);
    void *user;

    /* Bytes on their way to the MCU, each with its arrival time. */
    uint8_t q[SIM_UART_QUEUE];
    sim_time_ns_t q_t[SIM_UART_QUEUE];
    size_t q_head, q_count;
    sim_time_ns_t line_free_ns;

    /* Statistics. */
    uint64_t bytes_to_mcu;
    uint64_t bytes_from_mcu;
    uint64_t overrun_bytes;
    uint64_t dropped_bytes; /* faults or queue full */
};

void sim_uart_init(sim_uart_t *u, const char *name, uint32_t baud);

/* Device side: send bytes starting now (or at time t), queued behind any
 * bytes still on the wire. */
void sim_uart_send(sim_uart_t *u, const uint8_t *buf, size_t len);
void sim_uart_send_at(sim_uart_t *u, sim_time_ns_t t, const uint8_t *buf, size_t len);
void sim_uart_send_str(sim_uart_t *u, const char *s);

/* MCU side, matching the io structs. handle = sim_uart_t*. */
size_t sim_uart_read(void *handle, uint8_t *buf, size_t max);
int sim_uart_write(void *handle, const uint8_t *buf, size_t len);

/* Bytes that have arrived and not been read yet. */
size_t sim_uart_available(sim_uart_t *u);

#endif /* SIM_SIM_UART_H_ */
