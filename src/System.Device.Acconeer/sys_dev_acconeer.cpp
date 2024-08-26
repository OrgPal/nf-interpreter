//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include "sys_dev_acconeer.h"

// clang-format off

static const CLR_RT_MethodHandler method_lookup[] =
{
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::PerformCalibration___BOOLEAN__I4,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::PerformAssemblyTest___VOID,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeInit___VOID__U4__BOOLEAN,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeDeInit___VOID,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeSetEnable___VOID__BOOLEAN,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::get_EnableDebugMessages___STATIC__BOOLEAN,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::set_EnableDebugMessages___STATIC__VOID__BOOLEAN,
};

const CLR_RT_NativeAssemblyData g_CLR_AssemblyNative_System_Device_Acconeer =
{
    "System.Device.Acconeer",
    0xF9907C62,
    method_lookup,
    { 100, 0, 0, 1 }
};

// clang-format on
