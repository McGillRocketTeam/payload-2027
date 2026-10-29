#include "sensors/drivers/ubx/ubx.h"

void ubx_checksum(const uint8_t *data, size_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    /* TODO(workshop) */
    (void)data;
    (void)len;
    *ck_a = 0;
    *ck_b = 0;
}
