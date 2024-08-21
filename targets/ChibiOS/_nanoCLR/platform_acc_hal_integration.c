//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>

#include <acc_definitions_common.h>
#include <acc_hal_definitions_a121.h>
#include <acc_hal_integration_a121.h>
#include <acc_integration.h>
#include <acc_integration_log.h>

static void acc_hal_integration_sensor_transfer16(acc_sensor_id_t sensor_id, uint16_t *buffer, size_t buffer_length)
{
    (void)sensor_id;
    (void)buffer;
    (void)buffer_length;

    // 	const uint32_t SPI_TRANSMIT_RECEIVE_TIMEOUT = 5000;

    // #ifdef A121_USE_SPI_DMA
    // 	spi_transfer_complete = false;
    // 	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(&A121_SPI_HANDLE, (uint8_t *)buffer, (uint8_t *)buffer,
    // buffer_length);

    // 	if (status != HAL_OK)
    // 	{
    // 		return;
    // 	}

    // 	uint32_t start = HAL_GetTick();

    // 	while (!spi_transfer_complete && (HAL_GetTick() - start) < SPI_TRANSMIT_RECEIVE_TIMEOUT)
    // 	{
    // 		// Turn off interrupts
    // 		disable_interrupts();
    // 		// Check once more so that the interrupt have not occurred
    // 		if (!spi_transfer_complete)
    // 		{
    // 			__WFI();
    // 		}

    // 		// Enable interrupt again, the ISR will execute directly after this
    // 		enable_interrupts();
    // 	}
    // #else
    // 	HAL_SPI_TransmitReceive(&A121_SPI_HANDLE, (uint8_t *)buffer, (uint8_t *)buffer, buffer_length,
    // SPI_TRANSMIT_RECEIVE_TIMEOUT);

    // #endif
}

const acc_hal_a121_t *acc_hal_rss_integration_get_implementation(void)
{
    static const acc_hal_a121_t val = {
        .max_spi_transfer_size = 16,

        .mem_alloc = platform_malloc,
        .mem_free = platform_free,

        .transfer = NULL,
        .log = NULL,

        .optimization.transfer16 = acc_hal_integration_sensor_transfer16,
    };

    return &val;
}
