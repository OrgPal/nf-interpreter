## Configuration of ChibiOS, HAL and MCU

For a successful build the following changes are required:

## ADC configurations

The following ADC channels (and respective GPIO pins) are available to the managed API, in the respective index:
- PA6, ADC1 IN6
- PA4  ADC1 IN4
- PC2  ADC1 IN12
- PF10 ADC1 IN8
- PF8  ADC3 IN6
- PB8  ADC3 IN7
- Temp Sensor ADC1
- VrefInt ADC1
- Vbatt ADC1

## Floating point

The current build is set to add support for single-precision floating point.
Meaning that `System.Math` API supports only the `float` overloads. The `double` ones will throw a `NotImplementedException`.
