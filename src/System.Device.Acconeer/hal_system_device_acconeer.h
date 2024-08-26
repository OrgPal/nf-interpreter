//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#ifndef HAL_SYS_DEV_ACCONEER_H
#define HAL_SYS_DEV_ACCONEER_H

#ifdef __cplusplus
extern "C"
{
#endif

    #include <acc_definitions_common.h>

    /**
     * @brief Power on sensor supply
     *
     * @param[in] powerPin The GPIO number to power on the sensor
     */
    void acc_nano_hal_sensor_supply_on(GPIO_PIN powerPin);

    /**
     * @brief Power off sensor supply
     *
     * @param[in] powerPin The GPIO number to power off the sensor
     */
    void acc_nano_hal_sensor_supply_off(GPIO_PIN powerPin);

    /**
     * @brief Enable sensor
     *
     * Any pending sensor interrupts should be cleared before returning from function.
     * The sensor supply needs to be enabled by invoking @ref acc_nano_hal_sensor_supply_on
     * before calling this function.
     *
     * @param[in] enablePin The GPIO number to enable the sensor
     */
    void acc_nano_hal_sensor_enable(GPIO_PIN enablePin);

    /**
     * @brief Disable sensor
     *
     * @param[in] enablePin The GPIO number to disable the sensor
     */
    void acc_nano_hal_sensor_disable(GPIO_PIN enablePin);

    void nanoACC_HAL_Initialize(void);
    void nanoACC_HAL_Uninitialize(void);
    void acc_nano_hal_integration_log(acc_log_level_t level, const char *module, const char *format, ...);
    void acc_nano_hal_sensor_transfer(acc_sensor_id_t sensor_id, uint8_t *buffer, size_t buffer_length);
    void acc_nano_hal_sensor_transfer16(acc_sensor_id_t sensor_id, uint16_t *buffer, size_t buffer_length);
    void acc_nano_hal_integration_set_debug_output(bool enable);
    bool acc_nano_hal_integration_get_debug_output();

#ifdef __cplusplus
}
#endif

#endif // HAL_SYS_DEV_ACCONEER_H
