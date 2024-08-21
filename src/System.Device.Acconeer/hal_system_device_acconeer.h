//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#ifndef HAL_SYS_DEV_ACCONEER_H
#define HAL_SYS_DEV_ACCONEER_H

/**
 * @brief Power on sensor supply
 *
 * @param[in] powerPin The GPIO number to power on the sensor
 */
void __nfweak acc_nano_hal_sensor_supply_on(GPIO_PIN powerPin);

/**
 * @brief Power off sensor supply
 *
 * @param[in] powerPin The GPIO number to power off the sensor
 */
void __nfweak acc_nano_hal_sensor_supply_off(GPIO_PIN powerPin);

/**
 * @brief Enable sensor
 *
 * Any pending sensor interrupts should be cleared before returning from function.
 * The sensor supply needs to be enabled by invoking @ref acc_nano_hal_sensor_supply_on
 * before calling this function.
 *
 * @param[in] enablePin The GPIO number to enable the sensor
 */
void __nfweak acc_nano_hal_sensor_enable(GPIO_PIN enablePin);

/**
 * @brief Disable sensor
 *
 * @param[in] enablePin The GPIO number to disable the sensor
 */
void __nfweak acc_nano_hal_sensor_disable(GPIO_PIN enablePin);

#endif // HAL_SYS_DEV_ACCONEER_H
