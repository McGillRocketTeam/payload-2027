#include "sensors/drivers/tmp102/tmp102.h"

// set up shift size based on EM mode bit in config register (TODO)
// set up 12/13 bit count based on EM mode bit in config register (TODO)

int tmp102_read_raw(const tmp102_ctx_t *ctx, int16_t *raw)
{
    // TODO(workshop) 

    // read into raw using i2c_read function inside ctx
    int ret = ctx->i2c_read(ctx->handle, ctx->addr, 
                            TMP102_TEMP_REG,
                            (uint8_t *)raw, sizeof(*raw));
    
    // ERROR HANDLING: check if the read was successful, return if not
    if (ret != 0) {
          return ret;
    }

    * raw = (*raw) >> 4; // EM mode = 0

    return 0;
}

float tmp102_raw_to_celsius(int16_t raw)
{
     //TODO(workshop)
    // check leftmost bit 0/1   
    // if 1, convert to 2's complement (invert bits, (12 bits)
    // add 1, 
    // make negative
   
    // multiply by resolution  
    
    // return value
}
