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
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::_ctor___VOID__U4,
    NULL,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::PerformCalibration___BOOLEAN__I4,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeInit___VOID,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeDeInit___VOID,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeSetEnable___VOID__BOOLEAN,
};

const CLR_RT_NativeAssemblyData g_CLR_AssemblyNative_System_Device_Acconeer =
{
    "System.Device.Acconeer",
    0xC5C5B0E4,
    method_lookup,
    { 100, 0, 0, 1 }
};

// clang-format on
