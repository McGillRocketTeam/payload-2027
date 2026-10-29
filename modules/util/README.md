# modules/util

Small shared helpers that every module can use (ring buffers, byte packing, and so on). Keep them board independent.

FC's `util/HAL.h` (EXTI interrupt to object callback) is C++ and also depends on the MCU. Write a C version once the board exists; it belongs with the board code, not here.
