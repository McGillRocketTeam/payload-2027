#include "sensors/drivers/tmp102/tmp102.h"

int tmp102_read_raw(const tmp102_ctx_t *ctx, int16_t *raw)
{
    uint8_t buf[2];
    int result = ctx->i2c_read(ctx->handle, ctx->addr, 0x00, buf, 2);
    if (result != 0) {
        return -1;
    }
    *raw = (int16_t)((buf[0] << 8) | buf[1]);
    return 0;
}

float tmp102_raw_to_celsius(int16_t raw)
{
    int16_t chopped = raw>>4; // chop off the 4 least significant bits
    return chopped * 0.0625f;
}
