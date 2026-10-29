/*
 * Entry point for `pio run -e native` (the VS Code Build button).
 *
 * Lets the build link so every file in modules/ gets compiled on the host.
 * Unit test builds define PIO_UNIT_TESTING and use the test's own main().
 */
#ifndef PIO_UNIT_TESTING

#include <stdio.h>

int main(void)
{
    printf("modules/ built for the host. Run the unit tests with: pio test -e native\n");
    return 0;
}

#endif
