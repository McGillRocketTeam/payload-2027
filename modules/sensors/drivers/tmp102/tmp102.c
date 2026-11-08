#include "sensors/drivers/tmp102/tmp102.h"

int tmp102_read_raw(const tmp102_ctx_t *ctx, int16_t *raw)
{
    /* TODO(workshop) */
    (void)ctx;
    (void)raw;
    return -1;
}

float tmp102_raw_to_celsius(int16_t raw)
{
    /* TODO(workshop) */
    (void)raw;
    return 0.0f;
}
