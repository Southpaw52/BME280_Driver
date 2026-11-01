# BME280 Driver for STM32 HAL

**Author: southpaw**

A C implementation for reading temperature, pressure and relative humidity
from a Bosch BME280 using STM32 HAL I2C.

| Interface | Measurements | Platform |
|---|---|---|
| I2C | Temperature (°C), pressure (Pa), humidity (%RH) | STM32F4 HAL |

## Project scope

The repository contains sensor register definitions, calibration structures,
initialization, forced-mode control, raw register decoding and floating-point
compensation functions. The implementation uses STM32 HAL directly.

## Source map

| Path | Purpose |
|---|---|
| [Inc/bme280.h](Inc/bme280.h) | Public declarations and register definitions |
| [Src/bme280.c](Src/bme280.c) | Sensor initialization, reads and compensation |
| [Examples/main.c](Examples/main.c) | STM32 integration example |

## Integration notes

The application must supply STM32 HAL, the I2C handle, clocks, GPIO alternate
functions, pull-ups and board-specific startup configuration. The example is
a starting point for integration, rather than a complete firmware build.

Compensate temperature before pressure and humidity because those calculations
depend on the temperature-derived fine-resolution value. A forced conversion
must be triggered for each new sample; callers should check HAL return values
and impose bounded waits.

## Current status

This is an educational driver project. The published source still needs
integration cleanup: the implementation's include/state definitions and the
example's header naming must be reconciled before claiming a standalone build.
The repository does not currently include an automated test runner.
No new tests or hardware checks were run for this documentation update.

## Reference

[Bosch BME280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)

## Authorship and licensing

Application code and sensor integration: **southpaw**.
Third-party HAL and reference materials retain their own attribution.
A project-wide license has not yet been selected.
