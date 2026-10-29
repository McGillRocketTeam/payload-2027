# boards/

Custom PlatformIO board definitions, one JSON per board, same as flightComputer-2027.

Add `payload.json` once the MCU is chosen. Use FC's `boards/flightComputer.json` as the template and change `mcu`, `product_line`, `extra_flags`, `f_cpu`, the FPU fields, the RAM and flash sizes, and the debug target.
