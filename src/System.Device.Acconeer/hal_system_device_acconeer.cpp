//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "sys_dev_acconeer.h"
#include "hal_system_device_acconeer.h"
#include <nanoWeak.h>
#include <nanoHAL_v2.h>

#define LOG_BUFFER_MAX_SIZE 150

#define LOG_FORMAT "(%c) (%s) %s\r\n"

#define INT_WAIT_POOL_TIMESLOT 1U

typedef Library_sys_dev_acconeer_System_Device_Acconeer_Sensor Sensor;

static bool outputDebugMessages = false;

//////////////////////////////////////////////////////////////////////////////
// all functions are weak so they can be replaced at target level if needed //
//////////////////////////////////////////////////////////////////////////////

// this is the equivalent implementation of Acconeer's HAL function acc_hal_integration_sensor_supply_on
void __nfweak acc_nano_hal_sensor_supply_on(GPIO_PIN powerPin)
{
#ifdef ACC_SENSOR_ALWAYS_POWERED
    (void)powerPin;

#else

#error "Powering the sensor is not supported"

#endif
}

// this is the equivalent implementation of Acconeer's HAL function acc_hal_integration_sensor_supply_off
void __nfweak acc_nano_hal_sensor_supply_off(GPIO_PIN powerPin)
{
#ifdef ACC_SENSOR_ALWAYS_POWERED
    (void)powerPin;

#else

#error "Powering off the sensor is not supported"

#endif
}

// this is the equivalent implementation of Acconeer's HAL function acc_hal_integration_sensor_enable
void __nfweak acc_nano_hal_sensor_enable(GPIO_PIN enablePin)
{
    CPU_GPIO_SetPinState(enablePin, GpioPinValue_High);

    // wait 2 ms to make sure that the sensor crystal have time to stabilize
    OS_DELAY(2);
}

// this is the equivalent implementation of Acconeer's HAL function acc_hal_integration_sensor_disable
void __nfweak acc_nano_hal_sensor_disable(GPIO_PIN enablePin)
{
    CPU_GPIO_SetPinState(enablePin, GpioPinValue_Low);

    // wait 2 ms to make sure that the sensor crystal have time to stabilize
    OS_DELAY(2);
}

// this is the equivalent implementation of Acconeer's HAL function acc_hal_integration_wait_for_sensor_interrupt
bool __nfweak acc_nano_hal_integration_wait_for_sensor_interrupt(GPIO_PIN interruptPin, uint32_t timeoutMilliseconds)
{
    // read current ticks
    int32_t passes = timeoutMilliseconds / INT_WAIT_POOL_TIMESLOT;

    // read interrupt pin state
    GpioPinValue pinState = CPU_GPIO_GetPinState(interruptPin);

    while (pinState == GpioPinValue_Low && passes > 0)
    {
        // read interrupt pin state
        pinState = CPU_GPIO_GetPinState(interruptPin);

        // decress counter
        passes--;

        OS_DELAY(INT_WAIT_POOL_TIMESLOT);
    }

    // check if interrupt was triggered
    return pinState == GpioPinValue_High;
}

void __nfweak acc_nano_hal_integration_log(acc_log_level_t level, const char *module, const char *format, ...)
{
    char log_buffer[LOG_BUFFER_MAX_SIZE];
    va_list ap;
    char level_ch;
    va_start(ap, format);

    if (!outputDebugMessages)
    {
        // output is disabled
        return;
    }
    else
    {
        // output is enabled
        // start composing the log entry

        int ret = vsnprintf(log_buffer, LOG_BUFFER_MAX_SIZE, format, ap);

        if (ret >= LOG_BUFFER_MAX_SIZE)
        {
            log_buffer[LOG_BUFFER_MAX_SIZE - 4] = '.';
            log_buffer[LOG_BUFFER_MAX_SIZE - 3] = '.';
            log_buffer[LOG_BUFFER_MAX_SIZE - 2] = '.';
            log_buffer[LOG_BUFFER_MAX_SIZE - 1] = 0;
        }

        level_ch = (level <= ACC_LOG_LEVEL_DEBUG) ? "EWIVD"[level] : '?';

        CLR_Debug::Printf(LOG_FORMAT, level_ch, module, log_buffer);
    }

    va_end(ap);
}

void acc_nano_hal_integration_set_debug_output(bool enable)
{
    outputDebugMessages = enable;
}

bool acc_nano_hal_integration_get_debug_output()
{
    return outputDebugMessages;
}

void acc_nano_hal_sensor_transfer(acc_sensor_id_t sensor_id, uint8_t *buffer, size_t buffer_length)
{
    uint32_t spiHandle;

    SPI_WRITE_READ_SETTINGS swrs = {
        .fullDuplex = true,
        .readOffset = 0,
        .Bits16ReadWrite = false,
        .callback = NULL,
        .DeviceChipSelect = Sensor::GetTargetSpiCSLine((uint32_t)sensor_id),
        .ChipSelectActiveState = Sensor::GetTargetSpiCSActiveState((uint32_t)sensor_id)};

    // get the ACC sensor
    spiHandle = Sensor::GetSpiHandleForAccSensor(sensor_id);

    nanoSPI_Write_Read(spiHandle, swrs, buffer, buffer_length, buffer, buffer_length);
}

void acc_nano_hal_sensor_transfer16(acc_sensor_id_t sensor_id, uint16_t *buffer, size_t buffer_length)
{
    uint32_t spiHandle;

    SPI_WRITE_READ_SETTINGS swrs = {
        .fullDuplex = true,
        .readOffset = 0,
        .Bits16ReadWrite = true,
        .callback = NULL,
        .DeviceChipSelect = Sensor::GetTargetSpiCSLine((uint32_t)sensor_id),
        .ChipSelectActiveState = Sensor::GetTargetSpiCSActiveState((uint32_t)sensor_id)};

    // get the ACC sensor
    spiHandle = Sensor::GetSpiHandleForAccSensor(sensor_id);

    nanoSPI_Write_Read(spiHandle, swrs, (uint8_t *)buffer, buffer_length, (uint8_t *)buffer, buffer_length);
}
