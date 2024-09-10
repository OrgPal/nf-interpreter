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

// Define a heap area
#define HEAP_SIZE 2048
uint8_t acc_heap_area[HEAP_SIZE] __attribute__((section(".accheap")));

// Declare a heap object
static memory_heap_t accHeap;
static bool heapInitialized = false;

void *acc_hal_mem_alloc(size_t size)
{
    if (!heapInitialized)
    {
        chHeapObjectInit(&accHeap, acc_heap_area, HEAP_SIZE);
        heapInitialized = true;
    }

    // round up to nearest 32 bytes
    size = (size + 31) & ~31;

    return chHeapAllocAligned(&accHeap, size, 32);
}

void acc_hal_mem_free(void *ptr)
{
    chHeapFree(ptr);
}

const acc_hal_a121_t *acc_hal_rss_integration_get_implementation(void)
{
    static const acc_hal_a121_t val = {
        // set to max uint32_t value
        .max_spi_transfer_size = 65535,

        .mem_alloc = acc_hal_mem_alloc,
        .mem_free = acc_hal_mem_free,

        .transfer = NULL,
        .log = acc_nano_hal_integration_log,

        .optimization.transfer16 = acc_nano_hal_sensor_transfer16,
    };

    return &val;
}
