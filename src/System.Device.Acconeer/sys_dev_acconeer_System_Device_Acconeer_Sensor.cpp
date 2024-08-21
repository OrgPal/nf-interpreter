//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "sys_dev_acconeer.h"

typedef Library_sys_dev_spi_native_System_Device_Spi_SpiConnectionSettings SpiConnectionSettings;
typedef Library_sys_dev_spi_native_System_Device_Spi_SpiDevice SpiDevice;
typedef Library_sys_dev_acconeer_System_Device_Acconeer_Sensor Sensor;

// sensor reservation: 1 bit per sensor, 0 indexed, 0 = not reserved, 1 = reserved
static uint8_t sensorReserved;

static acc_sensor_t *accSensors[ACC_SENSOR_MAX_COUNT];
static uint32_t spiHandles[ACC_SENSOR_MAX_COUNT];

// helper methods

void nanoACC_HAL_Initialize()
{
    // initialize sensor reservation
    sensorReserved = 0;

    // initialize sensor instances
    for (int i = 0; i < ACC_SENSOR_MAX_COUNT; i++)
    {
        accSensors[i] = NULL;
    }
}

void nanoACC_HAL_Uninitialize()
{
    // destroy sensor instances
    for (int i = 0; i < ACC_SENSOR_MAX_COUNT; i++)
    {
        if (accSensors[i] != NULL)
        {
            acc_sensor_destroy(accSensors[i]);
        }
    }
}

static uint32_t GetTargetSpiBusId(uint32_t sensorId)
{
    switch (sensorId)
    {
#ifdef ACCONEER_SENSOR_0_BUSID
        case 0:
            return ACCONEER_SENSOR_0_BUSID;
#endif
#ifdef ACCONEER_SENSOR_1_BUSID
        case 1:
            return ACCONEER_SENSOR_1_BUSID;
#endif

        default:
            return 99;
    }
}

static uint32_t GetTargetSpiClock(uint32_t sensorId)
{
    switch (sensorId)
    {
#ifdef ACCONEER_SENSOR_0_SPI_CLOCK
        case 0:
            return ACCONEER_SENSOR_0_SPI_CLOCK;
#endif
#ifdef ACCONEER_SENSOR_1_SPI_CLOCK
        case 1:
            return ACCONEER_SENSOR_1_SPI_CLOCK;
#endif

        // default to 1MHz
        default:
            return 1000000;
    }
}

static uint8_t GetTargetEnablePin(uint32_t sensorId)
{
    switch (sensorId)
    {
#ifdef ACCONEER_SENSOR_0_ENABLE_GPIO
        case 0:
            return ACCONEER_SENSOR_0_ENABLE_GPIO;
#endif
#ifdef ACCONEER_SENSOR_1_ENABLE_GPIO
        case 1:
            return ACCONEER_SENSOR_1_ENABLE_GPIO;
#endif

        default:
            // this needs to be a valid GPIO pin
            _ASSERTE(FALSE);
            return 99;
    }
}

static uint8_t GetTargetInterruptPin(uint32_t sensorId)
{
    switch (sensorId)
    {
#ifdef ACCONEER_SENSOR_0_INTERRUPT_GPIO
        case 0:
            return ACCONEER_SENSOR_0_INTERRUPT_GPIO;
#endif
#ifdef ACCONEER_SENSOR_1_INTERRUPT_GPIO
        case 1:
            return ACCONEER_SENSOR_1_INTERRUPT_GPIO;
#endif

        default:
            // this needs to be a valid GPIO pin
            _ASSERTE(FALSE);
            return 99;
    }
}

static SPI_DEVICE_CONFIGURATION GetDefaultSpiConfig(uint32_t sensorId)
{
    SPI_DEVICE_CONFIGURATION spiConfig;

    // internally SPI bus ID is zero based, so better take care of that here
    spiConfig.Spi_Bus = GetTargetSpiBusId(sensorId) - 1;
    spiConfig.DeviceChipSelect = Sensor::GetTargetSpiCSLine(sensorId);
    spiConfig.ChipSelectActiveState = Sensor::GetTargetSpiCSActiveState(sensorId);
    spiConfig.Clock_RateHz = GetTargetSpiClock(sensorId);
    spiConfig.DataOrder16 = DataBitOrder_MSB;
    spiConfig.BusMode = SpiBusMode_master;
    spiConfig.Spi_Mode = SpiMode_Mode0;

    return spiConfig;
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::PerformCalibration___BOOLEAN__I4(
    CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    int32_t sensorId = -1;
    int32_t calibrationRetries;
    uint32_t bufferSize = 0;
    bool status = false;
    bool cal_complete = false;
    acc_cal_result_t *calibrationResult = NULL;
    uint8_t *buffer = NULL;
    GPIO_PIN interruptPin;

    CLR_RT_HeapBlock_Array *calibrationBuffer;

    // get a pointer to the managed object instance and check that it's not NULL
    CLR_RT_HeapBlock *pThis = stack.This();
    FAULT_ON_NULL(pThis);

    // grab the sensor ID
    sensorId = pThis[FIELD___sensorId].NumericByRef().u4;

    // get the calibration buffer
    calibrationBuffer = pThis[FIELD___calibration].DereferenceArray();
    calibrationResult = (acc_cal_result_t *)calibrationBuffer->GetFirstElement();

    bufferSize = pThis[FIELD___workBufferLength].NumericByRef().u4;

    interruptPin = (GPIO_PIN)pThis[FIELD___interruptPinNumber].NumericByRef().s4;

    // number of calibration retries
    calibrationRetries = stack.Arg1().NumericByRef().s4;

    // create buffer for calibration
    buffer = (uint8_t *)platform_malloc(bufferSize);

    if (buffer == NULL)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_OUT_OF_MEMORY);
    }

    for (int i = 0; i < calibrationRetries; i++)
    {
        // Reset sensor before calibration by disabling/enabling it
        acc_nano_hal_sensor_disable(pThis[FIELD___enablePinNumber].NumericByRef().u4);
        acc_nano_hal_sensor_enable(pThis[FIELD___enablePinNumber].NumericByRef().u4);

        do
        {
            status = acc_sensor_calibrate(accSensors[sensorId], &cal_complete, calibrationResult, buffer, bufferSize);

            // arbitrary delay to allow sensor to complete calibration
            PLATFORM_DELAY(10);

            if (status && !cal_complete)
            {
                status = CPU_GPIO_GetPinState(interruptPin) == GpioPinValue_High;
            }

        } while (status && !cal_complete);

        if (status)
        {
            // calibration successful

            // reset sensor after calibration by disabling/enabling it
            acc_nano_hal_sensor_disable(sensorId);
            acc_nano_hal_sensor_enable(sensorId);

            // exit the loop now
            break;
        }
    }

    stack.SetResult_Boolean(status);

    NANOCLR_CLEANUP();

    if (buffer != NULL)
    {
        platform_free(buffer);
    }

    NANOCLR_CLEANUP_END();
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeInit___VOID__U4__BOOLEAN(CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    const acc_hal_a121_t *accHal;
    int32_t sensorId = -1;
    uint32_t handle = -1;
    SPI_DEVICE_CONFIGURATION spiConfig;
    GPIO_PIN enablePin;
    GPIO_PIN interruptPin;

    CLR_RT_HeapBlock *spiDevice;
    CLR_RT_HeapBlock *connectionSettings;

    // get a pointer to the managed object instance and check that it's not NULL
    CLR_RT_HeapBlock *pThis = stack.This();
    FAULT_ON_NULL(pThis);

    // get the sensor ID from the argument
    sensorId = stack.Arg1().NumericByRef().u4;

    // check if this sensor ID has already been reserved
    if (sensorReserved & (1 << sensorId))
    {
        // this sensor ID has already been reserved
        NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_OPERATION);
    }

    // get a pointer to the SPI device
    spiDevice = pThis[FIELD___spiDevice].Dereference();

    // check if we need to grab the default confguration
    if (stack.Arg2().NumericByRef().u1)
    {
        // get the default configuration
        spiConfig = GetDefaultSpiConfig(sensorId);

        // store the remaining configurations in the managed object
        pThis[FIELD___enablePinNumber].NumericByRef().s4 = GetTargetEnablePin(sensorId);
        pThis[FIELD___interruptPinNumber].NumericByRef().s4 = GetTargetInterruptPin(sensorId);
    }
    else
    {
        // compose the SPI configuration from the managed object

        // grab the SPI device
        spiDevice = pThis[FIELD___spiDevice].Dereference();

        // get a pointer to the SPI config
        connectionSettings = spiDevice[SpiDevice::FIELD___connectionSettings].Dereference();

        // populate the SPI_DEVICE_CONFIGURATION struct

        // internally SPI bus ID is zero based, so better take care of that here
        spiConfig.Spi_Bus = connectionSettings[SpiConnectionSettings::FIELD___busId].NumericByRef().s4 - 1;
        spiConfig.BusMode = SpiBusMode_master;
        spiConfig.DeviceChipSelect = connectionSettings[SpiConnectionSettings::FIELD___csLine].NumericByRef().s4;

        // sanity check chip select line
        if (spiConfig.DeviceChipSelect < -1)
        {
            NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_PARAMETER);
        }

        // load CS active state from config (which is always PinValue.Low or PinValue.High
        spiConfig.ChipSelectActiveState =
            (bool)connectionSettings[SpiConnectionSettings::FIELD___chipSelectLineActiveState].NumericByRef().s4;
        spiConfig.Spi_Mode = (SpiMode)connectionSettings[SpiConnectionSettings::FIELD___spiMode].NumericByRef().s4;
        spiConfig.DataOrder16 =
            (DataBitOrder)connectionSettings[SpiConnectionSettings::FIELD___dataFlow].NumericByRef().s4;
        spiConfig.Clock_RateHz = connectionSettings[SpiConnectionSettings::FIELD___clockFrequency].NumericByRef().s4;
        spiConfig.BusConfiguration =
            (SpiBusConfiguration)connectionSettings[SpiConnectionSettings::FIELD___busConfiguration].NumericByRef().s4;
    }

    accHal = acc_hal_rss_integration_get_implementation();

    // set the data length accroding to platform implementation
    spiConfig.DataIs16bits = accHal->max_spi_transfer_size == 16;

    // open device
    hr = nanoSPI_OpenDevice(spiConfig, handle);
    NANOCLR_CHECK_HRESULT(hr);

    // enable pin
    enablePin = (GPIO_PIN)pThis[FIELD___enablePinNumber].NumericByRef().s4;
    if (!CPU_GPIO_ReservePin(enablePin, true))
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_PARAMETER);
    }

    CPU_GPIO_EnableOutputPin(enablePin, GpioPinValue_High, PinMode_Output);

    // interrupt pin
    interruptPin = (GPIO_PIN)pThis[FIELD___interruptPinNumber].NumericByRef().s4;
    if (!CPU_GPIO_ReservePin(interruptPin, true))
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_PARAMETER);
    }

    CPU_GPIO_EnableInputPin(interruptPin, 0, NULL, NULL, GPIO_INT_LEVEL_LOW, PinMode_Input);

    // store the SPI handle
    spiHandles[sensorId] = handle;

    // register HALL driver, if not already done
    if (sensorReserved == 0)
    {
        if (!acc_rss_hal_register(accHal))
        {
            NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
        }
    }

    // power on and enable sensor
    acc_nano_hal_sensor_supply_on(sensorId);
    acc_nano_hal_sensor_enable(sensorId);

    // create sensor instance
    accSensors[sensorId] = NULL;

    accSensors[sensorId] = acc_sensor_create(sensorId);

    // sanity check
    if (accSensors[sensorId] == NULL)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
    }

    // create calibration buffer
    NANOCLR_CHECK_HRESULT(CLR_RT_HeapBlock_Array::CreateInstance(
        pThis[FIELD___calibration],
        sizeof(acc_cal_result_t),
        g_CLR_RT_WellKnownTypes.m_UInt8));

    // all good, reserve this sensor ID...
    sensorReserved |= (1 << sensorId);

    // ... and store it in the instance field
    pThis[FIELD___sensorId].NumericByRef().u4 = sensorId;

    // set sensor enable state to true
    pThis[FIELD___enabled].NumericByRef().u1 = true;

    NANOCLR_CLEANUP();

    // destroy sensor in case of failure, only possible if we already have a sensor ID
    if (hr != S_OK && sensorId != -1 && accSensors[sensorId] != NULL)
    {
        acc_sensor_destroy(accSensors[sensorId]);
    }

    NANOCLR_CLEANUP_END();
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeDeInit___VOID(CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    uint32_t sensorId;

    // get a pointer to the managed object instance and check that it's not NULL
    CLR_RT_HeapBlock *pThis = stack.This();
    FAULT_ON_NULL(pThis);

    // get the sensor ID
    sensorId = pThis[FIELD___sensorId].NumericByRef().u4;

    // destroy the sensor instance
    acc_sensor_destroy(accSensors[sensorId]);

    // clear handle
    spiHandles[sensorId] = 0;

    // clear the reservation for this sensor ID
    sensorReserved &= ~(1 << sensorId);

    // nothing else to do here, SPI device has already been closed when SPI object was disposed

    NANOCLR_NOCLEANUP();
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeSetEnable___VOID__BOOLEAN(
    CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    bool enable;
    uint32_t sensorId;

    // access the managed object instance
    CLR_RT_HeapBlock *pThis = stack.This();
    FAULT_ON_NULL(pThis);

    // check if object has been disposed
    if (pThis[FIELD___disposedValue].NumericByRef().u1)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_OBJECT_DISPOSED);
    }

    // get the enable state from the argument
    enable = stack.Arg1().NumericByRef().u1;

    // get the sensor ID
    sensorId = pThis[FIELD___sensorId].NumericByRef().u4;

    if (enable)
    {
        acc_nano_hal_sensor_supply_on(sensorId);
        acc_nano_hal_sensor_enable(sensorId);
    }
    else
    {
        acc_nano_hal_sensor_disable(sensorId);
        acc_nano_hal_sensor_supply_off(sensorId);
    }

    NANOCLR_NOCLEANUP();
}

acc_sensor_t *Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::GetAccSensor(uint32_t sensorId)
{
    // sanity check for sensor ID
    if (sensorId >= ACC_SENSOR_MAX_COUNT)
    {
        return NULL;
    }

    return accSensors[sensorId];
}

uint32_t Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::GetSpiHandleForAccSensor(uint32_t sensorId)
{
    // sanity check for sensor ID
    if (sensorId >= ACC_SENSOR_MAX_COUNT)
    {
        return 0;
    }

    return spiHandles[sensorId];
}

int32_t Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::GetTargetSpiCSLine(uint32_t sensorId)
{
    switch (sensorId)
    {
#ifdef ACCONEER_SENSOR_0_CS
        case 0:
            return ACCONEER_SENSOR_0_CS;
#endif
#ifdef ACCONEER_SENSOR_1_CS
        case 1:
            return ACCONEER_SENSOR_1_CS;
#endif

        default:
            // this needs to be a valid GPIO pin
            _ASSERTE(FALSE);
            return 99;
    }
}

bool Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::GetTargetSpiCSActiveState(uint32_t sensorId)
{
    switch (sensorId)
    {
#ifdef ACCONEER_SENSOR_0_CS_ACTIVE_STATE
        case 0:
            return ACCONEER_SENSOR_0_CS_ACTIVE_STATE;
#endif
#ifdef ACCONEER_SENSOR_1_CS_ACTIVE_STATE
        case 1:
            return ACCONEER_SENSOR_1_CS_ACTIVE_STATE;
#endif
        // default to CS active low
        default:
            return false;
    }
}
