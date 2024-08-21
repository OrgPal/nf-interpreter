//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "sys_dev_acconeer.h"
#include "hal_system_device_acconeer.h"
#include <nanoWeak.h>

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

    // TBD: need to wait 2 ms to make sure that the sensor crystal have time to stabilize
}

// this is the equivalent implementation of Acconeer's HAL function acc_hal_integration_sensor_disable
void __nfweak acc_nano_hal_sensor_disable(GPIO_PIN enablePin)
{
    CPU_GPIO_SetPinState(enablePin, GpioPinValue_Low);

    // TBD: need to wait 2 ms to make sure that the sensor crystal have time to stabilize
}
