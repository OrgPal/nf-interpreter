//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "target_system_io_ports_config.h"
#include <sys_io_ser_native_target.h>

///////////
// UART2 //
///////////

UART_CONFIG_PINS(2, GPIOD, GPIOD, 5, 6, 7)

// initialization for UART2
UART_INIT(2)

// un-initialization for UART2
UART_UNINIT(2)

///////////
// UART4 //
///////////

// pin configuration for UART4
UART_CONFIG_PINS(4, GPIOA, GPIOA, 0, 1, 7)

// initialization for UART3
UART_INIT(4)

// un-initialization for UART3
UART_UNINIT(4)
