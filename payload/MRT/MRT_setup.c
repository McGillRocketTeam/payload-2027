#include "MRT_setup.h"

/*
 * Board glue: the only place that knows which SPI/UART handles and pins the
 * modules use. Fill in once the MCU and .ioc exist:
 *
 *   1. Write small wrappers around HAL calls, e.g. a tdc7200_ctx_t
 *      spi_transfer that drives TDC_CS and calls HAL_SPI_TransmitReceive.
 *   2. Pass them to tdc_create() / gps_create().
 *   3. Call tdc_init() / gps_init().
 */
void MRT_setup(void)
{
}
