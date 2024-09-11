//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "sys_dev_acconeer_distance.h"

typedef Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_DistanceConfiguration DistanceConfiguration;
typedef Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_DistanceResult DistanceResult;
typedef Library_sys_dev_acconeer_System_Device_Acconeer_Sensor Sensor;

static acc_detector_distance_handle_t *accDetectors[ACC_SENSOR_MAX_COUNT];
static acc_detector_distance_config_t *accDistanceConfig[ACC_SENSOR_MAX_COUNT];

static void SetDistanceConfig(acc_detector_distance_config_t *config, CLR_RT_HeapBlock *distanceConfig)
{
    acc_detector_distance_config_start_set(
        config,
        distanceConfig[DistanceConfiguration::FIELD___startOfMeasuredInterval].NumericByRef().r4);
    acc_detector_distance_config_end_set(
        config,
        distanceConfig[DistanceConfiguration::FIELD___endOfMeasuredInterval].NumericByRef().r4);
    acc_detector_distance_config_max_step_length_set(
        config,
        distanceConfig[DistanceConfiguration::FIELD___maxStepLength].NumericByRef().u2);
    acc_detector_distance_config_max_profile_set(
        config,
        (acc_config_profile_t)distanceConfig[DistanceConfiguration::FIELD___maxProfile].NumericByRef().u1);
    acc_detector_distance_config_num_frames_recorded_threshold_set(
        config,
        distanceConfig[DistanceConfiguration::FIELD___numberOfFrames].NumericByRef().u2);
    acc_detector_distance_config_peak_sorting_set(
        config,
        (acc_detector_distance_peak_sorting_t)distanceConfig[DistanceConfiguration::FIELD___peakSorting]
            .NumericByRef()
            .u1);
    acc_detector_distance_config_reflector_shape_set(
        config,
        (acc_detector_distance_reflector_shape_t)distanceConfig[DistanceConfiguration::FIELD___reflectorShape]
            .NumericByRef()
            .u1);
    acc_detector_distance_config_threshold_sensitivity_set(
        config,
        distanceConfig[DistanceConfiguration::FIELD___thresholdSensivity].NumericByRef().r4);
    acc_detector_distance_config_signal_quality_set(
        config,
        distanceConfig[DistanceConfiguration::FIELD___signalQuality].NumericByRef().r4);
    acc_detector_distance_config_close_range_leakage_cancellation_set(
        config,
        (bool)distanceConfig[DistanceConfiguration::FIELD___closeRangeLeakageCancellation].NumericByRef().u1);
}

HRESULT Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_Detector::
    GetMeasurement___SystemDeviceAcconeerDistanceDistanceResult(CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    uint32_t sensorId;
    uint32_t bufferSize = 0;
    bool resultAvailable = false;
    acc_sensor_t *accSensor;
    acc_cal_result_t *calibrationResult = NULL;
    acc_detector_cal_result_dynamic_t *dynamicCalibrationResult = NULL;
    acc_detector_distance_result_t *detectorDistanceResult = NULL;
    uint8_t *buffer = NULL;
    GPIO_PIN interruptPin;

    CLR_RT_HeapBlock_Array *calibrationResultBuffer;
    CLR_RT_HeapBlock_Array *staticCalibrationBuffer;
    CLR_RT_HeapBlock_Array *dynamicCalibrationBuffer;
    CLR_RT_HeapBlock *sensor = NULL;

    // get a pointer to the managed object instance
    CLR_RT_HeapBlock *pThis = stack.This();
    FAULT_ON_NULL(pThis);

    // instantiate the return object
    stack.PushValueAndClear();

    // get the sensor ID for this detector
    sensor = pThis[FIELD___sensor].Dereference();
    sensorId = sensor[Sensor::FIELD___sensorId].NumericByRef().u4;

    // get the ACC sensor
    accSensor = Sensor::GetAccSensor(sensorId);

    // get the calibration buffers
    calibrationResultBuffer = sensor[Sensor::FIELD___calibration].DereferenceArray();
    calibrationResult = (acc_cal_result_t *)calibrationResultBuffer->GetFirstElement();

    staticCalibrationBuffer = pThis[FIELD___staticCalibration].DereferenceArray();
    dynamicCalibrationBuffer = pThis[FIELD___dynamicCalibration].DereferenceArray();
    dynamicCalibrationResult = (acc_detector_cal_result_dynamic_t *)dynamicCalibrationBuffer->GetFirstElement();

    interruptPin = (GPIO_PIN)sensor[Sensor::FIELD___interruptPinNumber].NumericByRef().s4;

    // create buffer for operation
    bufferSize = sensor[Sensor::FIELD___workBufferLength].NumericByRef().u4;

    buffer = (uint8_t *)platform_malloc(bufferSize);

    if (buffer == NULL)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_OUT_OF_MEMORY);
    }

    do
    {
        if (!acc_detector_distance_prepare(
                accDetectors[sensorId],
                accDistanceConfig[sensorId],
                accSensor,
                calibrationResult,
                buffer,
                bufferSize))
        {
            // failure, need to exit now, returning null already on the stack
            NANOCLR_SET_AND_LEAVE(S_OK);
        }

        if (!acc_sensor_measure(accSensor))
        {
            // failure, need to exit now, returning null already on the stack
            NANOCLR_SET_AND_LEAVE(S_OK);
        }

        if (!acc_nano_hal_integration_wait_for_sensor_interrupt(interruptPin, ACC_SENSOR_TIMEOUT_MS))
        {
            // failure, need to exit now, returning null already on the stack
            NANOCLR_SET_AND_LEAVE(S_OK);
        }

        if (!acc_sensor_read(accSensor, buffer, bufferSize))
        {
            // failure, need to exit now, returning null already on the stack
            NANOCLR_SET_AND_LEAVE(S_OK);
        }

        if (!acc_detector_distance_process(
                accDetectors[sensorId],
                buffer,
                staticCalibrationBuffer->GetFirstElement(),
                dynamicCalibrationResult,
                &resultAvailable,
                detectorDistanceResult))
        {
            // failure, need to exit now, returning null already on the stack
            NANOCLR_SET_AND_LEAVE(S_OK);
        }

    } while (!resultAvailable);

    // got here, so we have a valid distance result
    NANOCLR_CHECK_HRESULT(
        ComposeDistanceResult(detectorDistanceResult, stack.TopValue(), pThis[FIELD___configuration]));

    NANOCLR_CLEANUP();

    if (buffer != NULL)
    {
        platform_free(buffer);
    }

    NANOCLR_CLEANUP_END();
}

HRESULT Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_Detector::NativeInitDetector___VOID(
    CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    uint32_t bufferSize;
    uint32_t calibrationSize;
    int32_t sensorId = -1;

    CLR_RT_HeapBlock *managedDistanceConfig = NULL;
    CLR_RT_HeapBlock *sensor = NULL;

    // get a pointer to the managed object instance
    CLR_RT_HeapBlock *pThis = stack.This();
    FAULT_ON_NULL(pThis);

    // get the sensor ID for this detector
    sensor = pThis[FIELD___sensor].Dereference();
    sensorId = sensor[Sensor::FIELD___sensorId].NumericByRef().u4;

    accDistanceConfig[sensorId] = acc_detector_distance_config_create();

    // sanity check
    if (accDistanceConfig[sensorId] == NULL)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
    }

    // grab a pointer to the distance configuration
    managedDistanceConfig = pThis[FIELD___configuration].Dereference();

    SetDistanceConfig(accDistanceConfig[sensorId], managedDistanceConfig);

    // store the detector handle
    accDetectors[sensorId] = acc_detector_distance_create(accDistanceConfig[sensorId]);

    // sanity check
    if (accDetectors[sensorId] == NULL)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
    }

    if (!acc_detector_distance_get_sizes(accDetectors[sensorId], &bufferSize, &calibrationSize))
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_FAIL);
    }

    // store the sizes
    sensor[Sensor::FIELD___workBufferLength].NumericByRef().u4 = bufferSize;

    NANOCLR_CHECK_HRESULT(CLR_RT_HeapBlock_Array::CreateInstance(
        pThis[FIELD___staticCalibration],
        calibrationSize,
        g_CLR_RT_WellKnownTypes.m_UInt8));

    // create calibration buffers
    NANOCLR_CHECK_HRESULT(CLR_RT_HeapBlock_Array::CreateInstance(
        pThis[FIELD___dynamicCalibration],
        sizeof(acc_detector_cal_result_dynamic_t),
        g_CLR_RT_WellKnownTypes.m_UInt8));

    NANOCLR_CLEANUP();

    // destroy driver instances in case of failure, only possible if we already have a sensor ID
    if (hr != S_OK && sensorId != -1)
    {
        if (accDistanceConfig[sensorId] != NULL)
        {
            acc_detector_distance_config_destroy(accDistanceConfig[sensorId]);
        }

        if (accDetectors[sensorId] != NULL)
        {
            acc_detector_distance_destroy(accDetectors[sensorId]);
        }
    }

    NANOCLR_CLEANUP_END();
}

HRESULT Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_Detector::NativeDispose___VOID(
    CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    uint32_t sensorId;

    CLR_RT_HeapBlock *sensor = NULL;

    // get a pointer to the managed object instance
    CLR_RT_HeapBlock *pThis = stack.This();
    FAULT_ON_NULL(pThis);

    // get the sensor ID for this detector
    sensor = pThis[FIELD___sensor].Dereference();
    sensorId = sensor[Sensor::FIELD___sensorId].NumericByRef().u4;

    // destroy the distance configuration
    acc_detector_distance_config_destroy(accDistanceConfig[sensorId]);
    acc_detector_distance_destroy(accDetectors[sensorId]);

    NANOCLR_NOCLEANUP();
}

HRESULT Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_Detector::NativeUpdateCalibration___BOOLEAN(
    CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    uint32_t sensorId;
    bool initialCalibrationDone;
    bool done = false;
    uint32_t bufferSize = 0;
    bool status;
    acc_sensor_t *accSensor;
    acc_cal_result_t *calibrationResult = NULL;
    acc_detector_cal_result_dynamic_t *dynamicCalibrationResult = NULL;
    uint8_t *buffer = NULL;
    GPIO_PIN interruptPin;

    CLR_RT_HeapBlock_Array *calibrationResultBuffer;
    CLR_RT_HeapBlock_Array *staticCalibrationBuffer;
    CLR_RT_HeapBlock_Array *dynamicCalibrationBuffer;
    CLR_RT_HeapBlock *sensor = NULL;

    // get a pointer to the managed object instance
    CLR_RT_HeapBlock *pThis = stack.This();
    FAULT_ON_NULL(pThis);

    // get the sensor ID for this detector
    sensor = pThis[FIELD___sensor].Dereference();
    sensorId = sensor[Sensor::FIELD___sensorId].NumericByRef().u4;

    // get the ACC sensor
    accSensor = Sensor::GetAccSensor(sensorId);

    // get the calibration buffers
    calibrationResultBuffer = sensor[Sensor::FIELD___calibration].DereferenceArray();
    calibrationResult = (acc_cal_result_t *)calibrationResultBuffer->GetFirstElement();

    staticCalibrationBuffer = pThis[FIELD___staticCalibration].DereferenceArray();
    dynamicCalibrationBuffer = pThis[FIELD___dynamicCalibration].DereferenceArray();
    dynamicCalibrationResult = (acc_detector_cal_result_dynamic_t *)dynamicCalibrationBuffer->GetFirstElement();

    interruptPin = (GPIO_PIN)sensor[Sensor::FIELD___interruptPinNumber].NumericByRef().s4;

    // get the initial calibration done flag
    initialCalibrationDone = pThis[FIELD___initialCalibrationDone].NumericByRef().u1;

    // create buffer for calibration
    bufferSize = sensor[Sensor::FIELD___workBufferLength].NumericByRef().u4;

    buffer = (uint8_t *)platform_malloc(bufferSize);

    if (buffer == NULL)
    {
        NANOCLR_SET_AND_LEAVE(CLR_E_OUT_OF_MEMORY);
    }

    if (initialCalibrationDone)
    {
        // update calibration
        do
        {
            status = acc_detector_distance_update_calibration(
                accSensor,
                accDetectors[sensorId],
                calibrationResult,
                buffer,
                bufferSize,
                dynamicCalibrationResult,
                &done);

            if (status && !done)
            {
                status = acc_nano_hal_integration_wait_for_sensor_interrupt(interruptPin, ACC_SENSOR_TIMEOUT_MS);
            }

        } while (status && !done);
    }
    else
    {
        // perform 1st time calibration
        do
        {
            status = acc_detector_distance_calibrate(
                accSensor,
                accDetectors[sensorId],
                calibrationResult,
                buffer,
                bufferSize,
                staticCalibrationBuffer->GetFirstElement(),
                staticCalibrationBuffer->m_numOfElements,
                dynamicCalibrationResult,
                &done);

            if (done)
            {
                // set the initial calibration done flag
                pThis[FIELD___initialCalibrationDone].NumericByRef().u1 = 1;

                break;
            }

            if (status && !done)
            {
                status = acc_nano_hal_integration_wait_for_sensor_interrupt(interruptPin, ACC_SENSOR_TIMEOUT_MS);
            }

        } while (status && !done);
    }

    stack.SetResult_Boolean(done);

    NANOCLR_CLEANUP();

    if (buffer != NULL)
    {
        platform_free(buffer);
    }

    NANOCLR_CLEANUP_END();
}

HRESULT Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_Detector::
    get_EnableDebugMessages___STATIC__BOOLEAN(CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    stack.SetResult_Boolean(acc_nano_hal_integration_get_debug_output());

    NANOCLR_NOCLEANUP_NOLABEL();
}

HRESULT Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_Detector::
    set_EnableDebugMessages___STATIC__VOID__BOOLEAN(CLR_RT_StackFrame &stack)
{
    NANOCLR_HEADER();

    bool enable = stack.Arg0().NumericByRef().u1;
    acc_nano_hal_integration_set_debug_output(enable);

    NANOCLR_NOCLEANUP_NOLABEL();
}

HRESULT Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_Detector::ComposeDistanceResult(
    acc_detector_distance_result_t *result,
    CLR_RT_HeapBlock &distanceResultRef,
    CLR_RT_HeapBlock &distanceConfigRef)
{
    NANOCLR_HEADER();

    CLR_RT_TypeDef_Index distanceResultTypeDef;
    CLR_RT_HeapBlock *distanceResult = NULL;
    CLR_RT_HeapBlock_Array *arrayRef = NULL;

    // create a new DistanceResult object
    // find <DistanceResult> type definition, don't bother checking the result as it exists for sure
    g_CLR_RT_TypeSystem.FindTypeDef("DistanceResult", "System.Device.Acconeer.Distance", distanceResultTypeDef);

    // create an instance of <NativeFindFile>
    NANOCLR_CHECK_HRESULT(g_CLR_RT_ExecutionEngine.NewObjectFromIndex(distanceResultRef, distanceResultTypeDef));

    // get a handle to the distance result object
    distanceResult = distanceResultRef.Dereference();

    // set the fields

    // set the distances array
    NANOCLR_CHECK_HRESULT(CLR_RT_HeapBlock_Array::CreateInstance(
        distanceResult[DistanceResult::FIELD___distances],
        ACC_DETECTOR_DISTANCE_RESULT_MAX_NUM_DISTANCES,
        g_CLR_RT_WellKnownTypes.m_Single));

    // copy over the distances array
    arrayRef = distanceResult[DistanceResult::FIELD___distances].DereferenceArray();
    memcpy(
        arrayRef->GetFirstElement(),
        result->distances,
        ACC_DETECTOR_DISTANCE_RESULT_MAX_NUM_DISTANCES * sizeof(float));

    // set the strengths array
    NANOCLR_CHECK_HRESULT(CLR_RT_HeapBlock_Array::CreateInstance(
        distanceResult[DistanceResult::FIELD___strengths],
        ACC_DETECTOR_DISTANCE_RESULT_MAX_NUM_DISTANCES,
        g_CLR_RT_WellKnownTypes.m_Single));

    // copy over the strengths array
    arrayRef = distanceResult[DistanceResult::FIELD___strengths].DereferenceArray();
    memcpy(
        arrayRef->GetFirstElement(),
        result->strengths,
        ACC_DETECTOR_DISTANCE_RESULT_MAX_NUM_DISTANCES * sizeof(float));

    distanceResult[DistanceResult::FIELD___nearStartEdge].NumericByRef().u1 = result->near_start_edge_status;
    distanceResult[DistanceResult::FIELD___calibrationNeeded].NumericByRef().u1 = result->calibration_needed;
    distanceResult[DistanceResult::FIELD___temperature].NumericByRef().s4 = result->temperature;
    distanceResult[DistanceResult::FIELD___distanceConfiguration].SetObjectReference(&distanceConfigRef);

    NANOCLR_NOCLEANUP();
}
