//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#ifndef TARGET_LITTLEFS_H
#define TARGET_LITTLEFS_H

#include <hal.h>
#include <cache.h>
#include <nanoHAL_v2.h>
#include <lfs.h>

#define LFS_SPI1

// need to define how many litllefs instances we'll be running
#define LITTLEFS_INSTANCES_COUNT 1

//////////////////////////////////
// defines specific for SPI1 drive
#ifdef LFS_SPI1

// timeout for flash operation (4s)
// max timing for write & erase operations (except chip erase)
#define HAL_SPI_TIMEOUT_DEFAULT_VALUE ((uint32_t)4000)

// 64 Mbits => 8 MByte
#define W25Q64_FLASH_SIZE 0x800000
// 128 sectors of 64kBytes
#define W25Q64_SECTOR_SIZE 0x10000
// 2048 subsectors of 4kBytes (!!minimum erase size!!)
#define W25Q64_SUBSECTOR_SIZE 0x1000
// 32768 pages of 256 bytes
#define W25Q64_PAGE_SIZE 0x100

// W25Q64 Commands
#define READ_CMD        0x03
#define BLOCK_ERASE_CMD 0x20

// Program Operations
#define PAGE_PROG_CMD     0x02
#define WRITE_ENABLE_CMD  0x06
#define WRITE_DISABLE_CMD 0x04

// Register Operations
#define READ_STATUS_REG1_CMD  0x05
#define WRITE_STATUS_REG1_CMD 0x01
#define READ_STATUS_REG2_CMD  0x35
#define WRITE_STATUS_REG2_CMD 0x31

// Erase Operations
#define SECTOR_ERASE_CMD 0x20
#define READ_ID_CMD      0x90
#define READ_ID_CMD2     0x9F

// power commands
#define RESUME_DEEP_PD_CMD  0xAB
#define DEEP_POWER_DOWN_CMD 0xB9

// IDs
#define W25Q64_MANUFACTURER_ID ((uint8_t)0xEF)
#define W25Q64_DEVICE_ID1      ((uint8_t)0x40)
#define W25Q64_DEVICE_ID2      ((uint8_t)0x17)

// Status Register 1
#define W25Q64_SR_BUSY ((uint8_t)0x01)

////////////////////////////////
// remapping into littlefs defines
#define LFS0_READ_SIZE      1
#define LFS0_PROG_SIZE      1
#define LFS0_BLOCK_SIZE     W25Q64_SUBSECTOR_SIZE
#define LFS0_BLOCK_COUNT    W25Q64_FLASH_SIZE / W25Q64_SUBSECTOR_SIZE
#define LFS0_BLOCK_CYCLES   100
#define LFS0_CACHE_SIZE     W25Q64_PAGE_SIZE
#define LFS0_LOOKAHEAD_SIZE LFS0_BLOCK_COUNT / 8

#define LFS0_READ_HANDLER  hal_lfs_read_0
#define LFS0_PROG_HANDLER  hal_lfs_prog_0
#define LFS0_ERASE_HANDLER hal_lfs_erase_0
#define LFS0_SYNC_HANDLER  hal_lfs_sync_0

#endif // LFS_SPI1

#ifdef __cplusplus
extern "C"
{
#endif

    int32_t hal_lfs_sync_0(const struct lfs_config *c);
    int32_t hal_lfs_sync_1(const struct lfs_config *c);

    bool hal_lfs_erase_chip_0();
    int32_t hal_lfs_erase_0(const struct lfs_config *c, lfs_block_t block);
    int32_t hal_lfs_read_0(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
    int32_t hal_lfs_prog_0(
        const struct lfs_config *c,
        lfs_block_t block,
        lfs_off_t off,
        const void *buffer,
        lfs_size_t size);

#ifdef __cplusplus
}
#endif

#endif // TARGET_LITTLEFS_H
