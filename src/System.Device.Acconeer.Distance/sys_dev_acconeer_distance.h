//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#ifndef SYS_DEV_ACCONEER_DISTANCE_H
#define SYS_DEV_ACCONEER_DISTANCE_H

#include <nanoCLR_Interop.h>
#include <nanoCLR_Runtime.h>
#include <nanoPackStruct.h>
#include <corlib_native.h>

#include <acc_definitions_a121.h>
#include <acc_detector_distance.h>
#include <acc_hal_definitions_a121.h>
#include <acc_hal_integration_a121.h>
#include <acc_integration.h>
#include <acc_integration_log.h>
#include <acc_rss_a121.h>
#include <acc_sensor.h>
#include <acc_version.h>

// matches declaration of acc_detector_distance_peak_sorting_t
// typedef enum __nfpack PeakSorting
// {
//     PeakSorting_Closest = 0,
//     PeakSorting_Strongest = 1,
// } PeakSorting;

// matched declaration of acc_config_profile_t
// typedef enum __nfpack Profile
// {
//     Profile_None = 0,
//     Profile_Profile1 = 1,
//     Profile_Profile2 = 2,
//     Profile_Profile3 = 3,
//     Profile_Profile4 = 4,
//     Profile_Profile5 = 5,
// } Profile;

// matches declaration of acc_detector_distance_reflector_shape_t
// typedef enum __nfpack ReflectorShape
// {
//     ReflectorShape_Generic = 0,
//     ReflectorShape_Planar = 1,
// } ReflectorShape;

struct Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_Detector
{
    static const int FIELD___sensor = 1;
    static const int FIELD___configuration = 2;

    NANOCLR_NATIVE_DECLARE(Calibrate___BOOLEAN);
    NANOCLR_NATIVE_DECLARE(GetMeasurement___SystemDeviceAcconeerDistanceDistanceResult);

    //--//
};

struct Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_DistanceConfiguration
{
    static const int FIELD___startOfMeasuredInterval = 1;
    static const int FIELD___endOfMeasuredInterval = 2;
    static const int FIELD___maxStepLength = 3;
    static const int FIELD___maxProfile = 4;
    static const int FIELD___numberOfFrames = 5;
    static const int FIELD___peakSorting = 6;
    static const int FIELD___reflectorShape = 7;
    static const int FIELD___thresholdSensivity = 8;
    static const int FIELD___signalQuality = 9;
    static const int FIELD___closeRangeLeakageCancellation = 10;

    //--//
};

struct Library_sys_dev_acconeer_distance_System_Device_Acconeer_Distance_DistanceResult
{
    static const int FIELD___distances = 1;
    static const int FIELD___strengths = 2;
    static const int FIELD___nearStartEdge = 3;
    static const int FIELD___calibrationNeeded = 4;
    static const int FIELD___temperature = 5;
    static const int FIELD___distanceConfiguration = 6;

    //--//
};

extern const CLR_RT_NativeAssemblyData g_CLR_AssemblyNative_System_Device_Acconeer_Distance;

#endif // SYS_DEV_ACCONEER_DISTANCE_H
