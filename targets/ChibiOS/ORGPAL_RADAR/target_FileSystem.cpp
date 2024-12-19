//
// Copyright (c) .NET Foundation and Contributors
// Portions Copyright (c) Microsoft Corporation.  All rights reserved.
// See LICENSE file in the project root for full license information.
//

#include <nanoHAL_v2.h>
#include <nanoHAL_Types.h>
#include <nanoPAL_FileSystem.h>
#include <littlefs_FS_Driver.h>
#include "Target_System_IO_FileSystem.h"

extern FILESYSTEM_DRIVER_INTERFACE g_LITTLEFS_FILE_SYSTEM_DriverInterface;
extern STREAM_DRIVER_INTERFACE g_LITTLEFS_STREAM_DriverInterface;

FILESYSTEM_INTERFACES g_AvailableFSInterfaces[] = {
    {&g_LITTLEFS_FILE_SYSTEM_DriverInterface, &g_LITTLEFS_STREAM_DriverInterface},
};

const size_t g_InstalledFSCount = ARRAYSIZE(g_AvailableFSInterfaces);

uint32_t g_FS_NumVolumes;
STREAM_DRIVER_DETAILS *g_FS_DriverDetails;
FileSystemVolume *g_FS_Volumes;

void FS_AddVolumes()
{
    // 1 SPI flash device
    g_FS_NumVolumes = 1;

    g_FS_Volumes = new FileSystemVolume[g_FS_NumVolumes];
    g_FS_DriverDetails = new STREAM_DRIVER_DETAILS[g_FS_NumVolumes];

    // W25Q64, drive F:, volume 1
    FileSystemVolumeList::AddVolume(
        &g_FS_Volumes[0],
        "I:",
        0,
        g_AvailableFSInterfaces[0].streamDriver,
        g_AvailableFSInterfaces[0].fsDriver,
        1,
        FALSE);
}

void FS_MountRemovableVolumes()
{

}
