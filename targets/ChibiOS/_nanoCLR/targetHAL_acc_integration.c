//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>

#include <hal_system_device_acconeer.h>

#include <acc_definitions_common.h>
#include <acc_hal_definitions_a121.h>
#include <acc_hal_integration_a121.h>
#include <acc_integration.h>
#include <acc_integration_log.h>

const acc_hal_a121_t *acc_hal_rss_integration_get_implementation(void)
{
    static const acc_hal_a121_t val = {
        // set to max uint32_t value
        .max_spi_transfer_size = 65535,

        .mem_alloc = platform_malloc,
        .mem_free = platform_free,

        .transfer = NULL,
        .log = acc_nano_hal_integration_log,

        .optimization.transfer16 = acc_nano_hal_sensor_transfer16,
    };

    return &val;
}
