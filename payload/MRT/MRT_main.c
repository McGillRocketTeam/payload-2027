#include "MRT_main.h"

/*
 * Superloop (same structure as payload-2026): interrupts only set flags or
 * fill buffers, and this loop does the polling, logging and command handling.
 * Never returns.
 */
void MRT_main(void)
{
    while (1) {
        /* TODO: poll modules, write logs, handle commands. */
    }
}
