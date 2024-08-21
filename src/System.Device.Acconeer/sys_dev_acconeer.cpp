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
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeInit___I4__U4__BOOLEAN,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeDeInit___VOID,
    Library_sys_dev_acconeer_System_Device_Acconeer_Sensor::NativeSetEnable___VOID__BOOLEAN,
};

const CLR_RT_NativeAssemblyData g_CLR_AssemblyNative_System_Device_Acconeer =
{
    "System.Device.Acconeer",
    0x3B4B464E,
    method_lookup,
    { 100, 0, 0, 1 }
};

// clang-format on
