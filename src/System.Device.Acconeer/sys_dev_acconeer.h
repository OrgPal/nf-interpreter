//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#ifndef SYS_DEV_ACCONEER_H
#define SYS_DEV_ACCONEER_H

#include <nanoCLR_Interop.h>
#include <nanoCLR_Runtime.h>
#include <nanoPackStruct.h>
#include <corlib_native.h>
#include <sys_dev_spi_native.h>

#include <target_system_device_acconeer_config.h>

extern "C"
{
#include <acc_definitions_a121.h>
#include <acc_detector_distance.h>
#include <acc_hal_definitions_a121.h>
#include <acc_hal_integration_a121.h>
#include <acc_integration.h>
#include <acc_integration_log.h>
#include <acc_rss_a121.h>
#include <acc_sensor.h>
#include <acc_version.h>
}

// set max number of sensors, if not defined at target level
#ifndef ACC_SENSOR_MAX_COUNT
#define ACC_SENSOR_MAX_COUNT 2
#endif

// set sensor timeout, if not defined at target level
#ifndef ACC_SENSOR_TIMEOUT_MS
#define ACC_SENSOR_TIMEOUT_MS 1000
#endif

struct Library_sys_dev_acconeer_System_Device_Acconeer_Sensor
{
    static const int FIELD___spiHandle = 1;
    static const int FIELD___calibration = 2;
    static const int FIELD___disposedValue = 3;
    static const int FIELD___enabled = 4;
    static const int FIELD___workBufferLength = 5;
    static const int FIELD___sensorId = 6;
    static const int FIELD___spiDevice = 7;
    static const int FIELD___enablePinNumber = 8;
    static const int FIELD___interruptPinNumber = 9;

    NANOCLR_NATIVE_DECLARE(PerformCalibration___BOOLEAN__I4);
    NANOCLR_NATIVE_DECLARE(NativeInit___I4__U4__BOOLEAN);
    NANOCLR_NATIVE_DECLARE(NativeDeInit___VOID);
    NANOCLR_NATIVE_DECLARE(NativeSetEnable___VOID__BOOLEAN);

    //--//

    static acc_sensor_t *GetAccSensor(uint32_t sensorId);
    static uint32_t GetSpiHandleForAccSensor(uint32_t sensorId);
    static int32_t GetTargetSpiCSLine(uint32_t sensorId);
    static bool GetTargetSpiCSActiveState(uint32_t sensorId);
};

extern const CLR_RT_NativeAssemblyData g_CLR_AssemblyNative_System_Device_Acconeer;

#endif // SYS_DEV_ACCONEER_H
