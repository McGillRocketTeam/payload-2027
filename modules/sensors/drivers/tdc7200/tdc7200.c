#include "sensors/drivers/tdc7200/tdc7200.h"

int tdc7200_read_reg(const tdc7200_ctx_t *ctx, uint8_t addr, uint8_t *value)
{
    /* TODO(workshop): build the command byte from the datasheet SPI format. */
    (void)ctx;
    (void)addr;
    (void)value;
    return -1;
}

int tdc7200_write_reg(const tdc7200_ctx_t *ctx, uint8_t addr, uint8_t value)
{
    /* TODO(workshop): build the command byte from the datasheet SPI format. */
    (void)ctx;
    (void)addr;
    (void)value;
    return -1;
}
