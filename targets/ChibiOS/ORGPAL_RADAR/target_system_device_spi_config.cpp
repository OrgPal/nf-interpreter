//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <sys_dev_spi_native_target.h>

//////////
// SPI1 //
//////////

// pin configuration for SPI1
// port for SCK pin is: GPIOA_SPI1_SCK
// port for MISO pin is: GPIOA_SPI1_MISO
// port for MOSI pin is: GPIOA_SPI1_MOSI

// GPIO alternate pin function is 5 (see alternate function mapping table in device datasheet)
SPI_CONFIG_PINS(1, GPIOA, 5, GPIOA, 6, GPIOA, 7, 5)

//////////
// SPI3 //
//////////

// pin configuration for SPI3
// port for SCK pin is: GPIOC_SPI3_SCK
// port for MISO pin is: GPIOC_SPI3_MISO
// port for MOSI pin is: GPIOB_SPI3_MOSI

// GPIO alternate pin function is 6 for SCK and MISO and 7 for MOSI
// (see alternate function mapping table in device datasheet)
SPI_CONFIG_PINS_ALT(3, GPIOC, 10, 6, GPIOC, 11, 6, GPIOB, 2, 7)
