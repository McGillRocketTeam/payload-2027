#include "sim_uart.h"

#include <string.h>

void sim_uart_init(sim_uart_t *u, const char *name, uint32_t baud)
{
    memset(u, 0, sizeof(*u));
    sim_device_init(&u->base, name);
    u->baud = baud;
    u->rx_buffer = 512;
}

static sim_time_ns_t byte_time(const sim_uart_t *u)
{
    return u->baud ? (sim_time_ns_t)(10.0 * 1e9 / (double)u->baud) : 0;
}

void sim_uart_send_at(sim_uart_t *u, sim_time_ns_t t, const uint8_t *buf, size_t len)
{
    sim_fault_mode_t f = sim_device_begin(&u->base);
    if (f == SIM_FAULT_OFF || f == SIM_FAULT_BUS_ERROR) {
        u->dropped_bytes += len;
        return;
    }
    if (f == SIM_FAULT_STALL) {
        /* sim_device_begin already moved time; push the chunk out too. */
        t = sim_now_ns() > t ? sim_now_ns() : t;
    }

    size_t flip = SIZE_MAX;
    uint8_t flip_mask = 0;
    if (f == SIM_FAULT_CORRUPT && len > 0) {
        uint32_t bit = sim_rand_u32() % (uint32_t)(len * 8);
        flip = bit / 8;
        flip_mask = (uint8_t)(1u << (bit % 8));
    }

    sim_time_ns_t bt = byte_time(u);
    sim_time_ns_t at = t > u->line_free_ns ? t : u->line_free_ns;
    for (size_t i = 0; i < len; i++) {
        if (u->q_count == SIM_UART_QUEUE) {
            u->dropped_bytes += len - i;
            break;
        }
        at += bt;
        size_t idx = (u->q_head + u->q_count) % SIM_UART_QUEUE;
        u->q[idx] = (i == flip) ? (uint8_t)(buf[i] ^ flip_mask) : buf[i];
        u->q_t[idx] = at;
        u->q_count++;
    }
    u->line_free_ns = at;
    sim_device_trace(&u->base, "device sends %zu bytes", len);
}

void sim_uart_send(sim_uart_t *u, const uint8_t *buf, size_t len)
{
    sim_uart_send_at(u, sim_now_ns(), buf, len);
}

void sim_uart_send_str(sim_uart_t *u, const char *s)
{
    sim_uart_send(u, (const uint8_t *)s, strlen(s));
}

/* Count arrived bytes and apply overrun (drop oldest beyond rx_buffer). */
static size_t settle(sim_uart_t *u)
{
    if (u->produce) {
        u->produce(u, sim_now_ns(), u->user);
    }
    sim_time_ns_t now = sim_now_ns();
    size_t arrived = 0;
    while (arrived < u->q_count && u->q_t[(u->q_head + arrived) % SIM_UART_QUEUE] <= now) {
        arrived++;
    }
    if (u->rx_buffer && arrived > u->rx_buffer) {
        size_t drop = arrived - u->rx_buffer;
        u->q_head = (u->q_head + drop) % SIM_UART_QUEUE;
        u->q_count -= drop;
        u->overrun_bytes += drop;
        arrived -= drop;
        sim_device_trace(&u->base, "overrun: %zu bytes lost", drop);
    }
    return arrived;
}

size_t sim_uart_available(sim_uart_t *u)
{
    return settle(u);
}

size_t sim_uart_read(void *handle, uint8_t *buf, size_t max)
{
    sim_uart_t *u = handle;
    size_t n = settle(u);
    if (n > max) {
        n = max;
    }
    for (size_t i = 0; i < n; i++) {
        buf[i] = u->q[u->q_head];
        u->q_head = (u->q_head + 1) % SIM_UART_QUEUE;
    }
    u->q_count -= n;
    u->bytes_to_mcu += n;
    return n;
}

int sim_uart_write(void *handle, const uint8_t *buf, size_t len)
{
    sim_uart_t *u = handle;
    sim_fault_mode_t f = sim_device_begin(&u->base);
    if (f == SIM_FAULT_BUS_ERROR) {
        return -1;
    }
    u->bytes_from_mcu += len;
    /* Blocking HAL_UART_Transmit: the MCU waits for the bytes to go out. */
    sim_advance_ns(byte_time(u) * len);
    if (f == SIM_FAULT_OFF) {
        return 0;
    }
    sim_device_trace(&u->base, "mcu sends %zu bytes", len);
    if (u->on_rx) {
        u->on_rx(u, buf, len, u->user);
    }
    return 0;
}
