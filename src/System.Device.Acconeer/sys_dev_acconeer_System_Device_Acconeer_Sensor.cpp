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

void Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::Initialize()
{
    // initialize sensor reservation
    sensorReserved = 0;

    // initialize sensor instances
    for (int i = 0; i < ACC_SENSOR_MAX_COUNT; i++)
    {
        accSensors[i] = NULL;
    }
}

void Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::Uninitialize()
{
    // destroy sensor instances
    for (int i = 0; i < ACC_SENSOR_MAX_COUNT; i++)
    {
        if (accSensors[i] != NULL)
        {
            acc_sensor_destroy(accSensors[i]);
        }
    }

    // reset sensor reservation
    sensorReserved = 0;
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

        // default to 10MHz
        default:
            return 10000000;
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
    GpioPinValue interruptState;

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

    memset(buffer, 0, bufferSize);

    for (int i = 0; i < calibrationRetries; i++)
    {
        // Reset sensor before calibration by disabling/enabling it
        acc_nano_hal_sensor_disable(pThis[FIELD___enablePinNumber].NumericByRef().u4);
        acc_nano_hal_sensor_enable(pThis[FIELD___enablePinNumber].NumericByRef().u4);

        do
        {
            interruptState = CPU_GPIO_GetPinState(interruptPin);

            status = (interruptState == GpioPinValue_High);

            status = acc_sensor_calibrate(accSensors[sensorId], &cal_complete, calibrationResult, buffer, bufferSize);

            // arbitrary delay to allow sensor to complete calibration
            PLATFORM_DELAY(10);

            if (status && !cal_complete)
            {
                interruptState = CPU_GPIO_GetPinState(interruptPin);

                status = (interruptState == GpioPinValue_High);
            }

        } while (status && !cal_complete);

        if (status)
        {
            // calibration successful

            // reset sensor after calibration by disabling/enabling it
            acc_nano_hal_sensor_disable(pThis[FIELD___enablePinNumber].NumericByRef().u4);
            acc_nano_hal_sensor_enable(pThis[FIELD___enablePinNumber].NumericByRef().u4);

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

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::PerformAssemblyTest___VOID(CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    int32_t sensorId = -1;
    uint8_t *buffer = NULL;

    acc_rss_assembly_test_t *assembly_test = NULL;
    acc_config_t *config = NULL;

    // get a pointer to the managed object instance and check that it's not NULL
    CLR_RT_HeapBlock *pThis = stack.This();
    FAULT_ON_NULL(pThis);

    // grab the sensor ID
    sensorId = pThis[FIELD___sensorId].NumericByRef().u4;

    CLR_Debug::Printf("Acconeer software version %s\n", acc_version_get());

    // allocate work buffer
    buffer = (uint8_t *)platform_malloc(ACC_RSS_ASSEMBLY_TEST_MIN_BUFFER_SIZE);

    // sanity check
    if (buffer == NULL)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_OUT_OF_MEMORY);
    }

    // Create test
    assembly_test = acc_rss_assembly_test_create(sensorId, buffer, ACC_RSS_ASSEMBLY_TEST_MIN_BUFFER_SIZE);
    if (assembly_test == NULL)
    {
        CLR_Debug::Printf("Could not create assembly test");
        NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
    }

    // Basic read test
    acc_rss_assembly_test_disable_all_tests(assembly_test);
    acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_BASIC_READ);
    if (!RunTest(assembly_test, sensorId))
    {
        CLR_Debug::Printf("Basic read test failed");
        NANOCLR_SET_AND_LEAVE(S_OK);
    }

    // Communication test
    acc_rss_assembly_test_disable_all_tests(assembly_test);
    acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_COMMUNICATION);
    if (!RunTest(assembly_test, sensorId))
    {
        CLR_Debug::Printf("Communication test failed");
        NANOCLR_SET_AND_LEAVE(S_OK);
    }

    // Enable test
    acc_rss_assembly_test_disable_all_tests(assembly_test);
    acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_ENABLE_PIN);
    if (!RunTest(assembly_test, sensorId))
    {
        CLR_Debug::Printf("Enable test failed");
        NANOCLR_SET_AND_LEAVE(S_OK);
    }

    // Interrupt test
    acc_rss_assembly_test_disable_all_tests(assembly_test);
    acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_INTERRUPT);
    if (!RunTest(assembly_test, sensorId))
    {
        CLR_Debug::Printf("Interrupt test failed");
        NANOCLR_SET_AND_LEAVE(S_OK);
    }

    // Clock and Supply test
    acc_rss_assembly_test_disable_all_tests(assembly_test);
    acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_CLOCK_AND_SUPPLY);
    if (!RunTest(assembly_test, sensorId))
    {
        CLR_Debug::Printf("Clock and Supply test failed");
        NANOCLR_SET_AND_LEAVE(S_OK);
    }

    // Sensor calibration test
    acc_rss_assembly_test_disable_all_tests(assembly_test);
    acc_rss_assembly_test_enable(assembly_test, ACC_RSS_ASSEMBLY_TEST_ID_SENSOR_CALIBRATION);
    if (!RunTest(assembly_test, sensorId))
    {
        CLR_Debug::Printf("Sensor calibration test failed");
        NANOCLR_SET_AND_LEAVE(S_OK);
    }

    CLR_Debug::Printf("All tests passed\n");

    NANOCLR_CLEANUP();

    if (assembly_test != NULL)
    {
        acc_rss_assembly_test_destroy(assembly_test);
    }

    if (config != NULL)
    {
        acc_config_destroy(config);
    }

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
    spiConfig.DataIs16bits = accHal->transfer == NULL ? true : false;

    // open device
    hr = nanoSPI_OpenDevice(spiConfig, handle);
    NANOCLR_CHECK_HRESULT(hr);

    // enable pin
    enablePin = (GPIO_PIN)pThis[FIELD___enablePinNumber].NumericByRef().s4;
    if (!CPU_GPIO_ReservePin(enablePin, true))
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_PARAMETER);
    }

    CPU_GPIO_EnableOutputPin(enablePin, GpioPinValue_Low, PinMode_Output);

    // interrupt pin
    interruptPin = (GPIO_PIN)pThis[FIELD___interruptPinNumber].NumericByRef().s4;
    if (!CPU_GPIO_ReservePin(interruptPin, true))
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_INVALID_PARAMETER);
    }

    CPU_GPIO_EnableInputPin(interruptPin, 0, NULL, NULL, GPIO_INT_LEVEL_LOW, PinMode_InputPullDown);

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
    acc_nano_hal_sensor_supply_on(enablePin);
    acc_nano_hal_sensor_enable(enablePin);

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
    GPIO_PIN enablePin;

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

    // get the enable pin
    enablePin = pThis[FIELD___enablePinNumber].NumericByRef().s4;

    if (enable)
    {
        acc_nano_hal_sensor_supply_on(enablePin);
        acc_nano_hal_sensor_enable(enablePin);
    }
    else
    {
        acc_nano_hal_sensor_disable(enablePin);
        acc_nano_hal_sensor_supply_off(enablePin);
    }

    NANOCLR_NOCLEANUP();
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::get_EnableDebugMessages___STATIC__BOOLEAN(
    CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    stack.SetResult_Boolean(acc_nano_hal_integration_get_debug_output());

    NANOCLR_NOCLEANUP_NOLABEL();
}

HRESULT Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::set_EnableDebugMessages___STATIC__VOID__BOOLEAN(
    CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    bool enable = stack.Arg0().NumericByRef().u1;
    acc_nano_hal_integration_set_debug_output(enable);

    NANOCLR_NOCLEANUP_NOLABEL();
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

bool Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::RunTest(
    acc_rss_assembly_test_t *assembly_test,
    acc_sensor_id_t sensor_id)
{
    bool all_passed = true;
    GPIO_PIN interruptPin = (GPIO_PIN)GetTargetInterruptPin(sensor_id);
    GPIO_PIN enablePin = (GPIO_PIN)GetTargetEnablePin(sensor_id);
    GpioPinValue interruptState;

    // power on and enable sensor
    acc_nano_hal_sensor_supply_on(enablePin);
    acc_nano_hal_sensor_enable(enablePin);

    acc_rss_test_state_t assembly_test_state = ACC_RSS_TEST_STATE_ONGOING;
    acc_rss_test_integration_status_t integration_status = ACC_RSS_TEST_INTEGRATION_STATUS_OK;

    do
    {
        assembly_test_state = acc_rss_assembly_test_execute(assembly_test, integration_status);

        switch (assembly_test_state)
        {
            case ACC_RSS_TEST_STATE_TOGGLE_ENABLE_PIN:
                acc_nano_hal_sensor_disable(enablePin);
                acc_nano_hal_sensor_enable(enablePin);
                integration_status = ACC_RSS_TEST_INTEGRATION_STATUS_OK;
                break;

            case ACC_RSS_TEST_STATE_WAIT_FOR_INTERRUPT:

                interruptState = CPU_GPIO_GetPinState(interruptPin);

                // if (!acc_hal_integration_wait_for_sensor_interrupt(sensor_id, SENSOR_TIMEOUT_MS))
                if (interruptState != GpioPinValue_High)
                {
                    /* Wait for interrupt failed */
                    integration_status = ACC_RSS_TEST_INTEGRATION_STATUS_TIMEOUT;
                }
                else
                {
                    integration_status = ACC_RSS_TEST_INTEGRATION_STATUS_OK;
                }

                break;

            default:
                integration_status = ACC_RSS_TEST_INTEGRATION_STATUS_OK;
                break;
        }

    } while (assembly_test_state != ACC_RSS_TEST_STATE_COMPLETE);

    acc_nano_hal_sensor_disable(enablePin);
    acc_nano_hal_sensor_supply_off(enablePin);

    uint16_t nbr_of_test_results = 0U;

    const acc_rss_assembly_test_result_t *test_results =
        acc_rss_assembly_test_get_results(assembly_test, &nbr_of_test_results);

    for (uint16_t idx = 0; idx < nbr_of_test_results; idx++)
    {
        CLR_Debug::Printf(
            "'%s' [%s]\r\n",
            test_results[idx].test_name,
            test_results[idx].test_result ? "PASS" : "FAIL");
        if (!test_results[idx].test_result)
        {
            all_passed = false;
        }
    }

    return all_passed;
}
