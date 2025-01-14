//
// Copyright (c) .NET Foundation and Contributors
// See LICENSE file in the project root for full license information.
//

#include <ch.h>
#include <hal.h>
#include <target_littlefs.h>
#include <hal_littlefs.h>

#if defined(LFS_SPI1)
#define LFS_CACHE_SIZE W25Q64_PAGE_SIZE
#else
#error "There is no define for LFS_SPI1"
#endif

#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
uint8_t lfs_inputBuffer[CACHE_SIZE_ALIGN(uint8_t, LFS_CACHE_SIZE)] __attribute__((section(".nocache")));
CC_ALIGN_DATA(CACHE_LINE_SIZE)
uint8_t lfs_outputBuffer[CACHE_SIZE_ALIGN(uint8_t, LFS_CACHE_SIZE)] __attribute__((section(".nocache")));
#else
uint8_t lfs_inputBuffer[];
uint8_t lfs_outputBuffer[];
#endif

int32_t lfs_inputBufferSize = LFS_CACHE_SIZE;
int32_t lfs_outputBufferSize = LFS_CACHE_SIZE;

#ifdef LFS_SPI1

static const SPIConfig spiConfig = {
    .circular = false,
    .slave = false,
    .data_cb = NULL,
    .error_cb = NULL,
    // CPHA=0, CPOL=0, MSb first
    .cr1 = 0U + 32, // SPI_CR1_CPOL | SPI_CR1_BR_0,
    // transfer length to 8bit
    .cr2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0};

#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
uint8_t dataBuffer_0[CACHE_SIZE_ALIGN(uint8_t, W25Q64_PAGE_SIZE)] __attribute__((section(".nocache")));
#else
uint8_t dataBuffer_0[W25Q64_PAGE_SIZE];
#endif

#ifdef DEBUG
#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
uint8_t tempBuffer[CACHE_SIZE_ALIGN(uint8_t, W25Q64_PAGE_SIZE)] __attribute__((section(".nocache")));
#else
uint8_t tempBuffer[W25Q64_PAGE_SIZE];
#endif
#endif

///////////////
// Definitions
#define CS_SELECT   palClearPad(PAL_PORT(LINE_FLASH_CS), PAL_PAD(LINE_FLASH_CS))
#define CS_UNSELECT palSetPad(PAL_PORT(LINE_FLASH_CS), PAL_PAD(LINE_FLASH_CS))

///////////////
// declarations
static bool SPI_Erase_Block(uint32_t addr, bool largeBlock);
static bool SPI_Read(uint8_t *pData, uint32_t readAddr, uint32_t size);
static bool SPI_Write(const uint8_t *pData, uint32_t writeAddr, uint32_t size);
static bool SPI_WaitOnBusy();

extern uint32_t HAL_GetTick(void);

// target specific implementation of hal_lfs_erase
int32_t hal_lfs_erase_0(const struct lfs_config *c, lfs_block_t block)
{
    Watchdog_Reset();

    uint32_t addr = block * c->block_size;

    if (!SPI_Erase_Block(addr, false))
    {
        return LFS_ERR_IO;
    }

    return LFS_ERR_OK;
}

// target specific implementation of hal_lfs_read
int32_t hal_lfs_read_0(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t addr = block * c->block_size + off;

    if (!SPI_Read(buffer, addr, size))
    {
        return LFS_ERR_IO;
    }

    return LFS_ERR_OK;
}

// target specific implementation of hal_lfs_prog
int32_t hal_lfs_prog_0(
    const struct lfs_config *c,
    lfs_block_t block,
    lfs_off_t off,
    const void *buffer,
    lfs_size_t size)
{
    uint32_t addr = block * c->block_size + off;

    if (!SPI_Write(buffer, addr, size))
    {
        return LFS_ERR_IO;
    }

#ifdef DEBUG

    memset(tempBuffer, 0xBB, size);

    // read back and compare
    SPI_Read(tempBuffer, addr, size);
    ASSERT(memcmp(buffer, tempBuffer, size) == 0);

#endif

    return LFS_ERR_OK;
}

// target specific implementation of chip erase
bool hal_lfs_erase_chip_0()
{
    // need to do this one one block at a time to avoid watchdog reset
    for (uint32_t i = 0; i < W25Q64_FLASH_SIZE / W25Q64_SECTOR_SIZE; i++)
    {
        if (!SPI_Erase_Block(i * W25Q64_SECTOR_SIZE, true))
        {
            return false;
        }

        // reset watchdog
        Watchdog_Reset();
    }

    return true;
}

static bool SPI_WaitOnBusy()
{
    uint32_t tickstart = HAL_GetTick();

    // clear read buffer
    memset(dataBuffer_0, 0xFF, 1);

    dataBuffer_0[0] = READ_STATUS_REG1_CMD;
    cacheBufferFlush(dataBuffer_0, sizeof(dataBuffer_0));

    CS_SELECT;

    // send read status register 1
    spiSend(&SPID3, 1, dataBuffer_0);

    while (true)
    {
        // read register value
        spiReceive(&SPID3, 1, dataBuffer_0);
        cacheBufferInvalidate(dataBuffer_0, sizeof(dataBuffer_0));

        if (!(dataBuffer_0[0] & W25Q64_SR_BUSY))
        {
            // BuSY bit is cleared
            break;
        }

        if ((HAL_GetTick() - tickstart) > HAL_SPI_TIMEOUT_DEFAULT_VALUE)
        {
            // operation timeout

            // unselect SPI
            CS_UNSELECT;

            return false;
        }
    }

    CS_UNSELECT;

    return true;
}

static bool SPI_Erase_Block(uint32_t addr, bool largeBlock)
{
    // send write enable
    dataBuffer_0[0] = WRITE_ENABLE_CMD;
    cacheBufferFlush(dataBuffer_0, sizeof(dataBuffer_0));

    CS_SELECT;
    spiSend(&SPID3, 1, dataBuffer_0);
    CS_UNSELECT;

    // send block erase
    dataBuffer_0[0] = largeBlock ? BLOCK_ERASE_CMD : SECTOR_ERASE_CMD;
    dataBuffer_0[1] = (uint8_t)(addr >> 16);
    dataBuffer_0[2] = (uint8_t)(addr >> 8);
    dataBuffer_0[3] = (uint8_t)addr;

    CS_SELECT;
    spiSend(&SPID3, 4, dataBuffer_0);
    CS_UNSELECT;

    // wait for erase operation to complete
    return SPI_WaitOnBusy();
}

static bool SPI_Read(uint8_t *pData, uint32_t readAddr, uint32_t size)
{
    // send read page command
    dataBuffer_0[0] = READ_CMD;
    dataBuffer_0[1] = (uint8_t)(readAddr >> 16);
    dataBuffer_0[2] = (uint8_t)(readAddr >> 8);
    dataBuffer_0[3] = (uint8_t)readAddr;

    CS_SELECT;
    spiSend(&SPID3, 4, dataBuffer_0);

    // clear read buffer
    memset(dataBuffer_0, 0xDD, size);

    spiReceive(&SPID3, size, dataBuffer_0);
    CS_UNSELECT;

    // invalidate cache
    // (only required for Cortex-M7)
    cacheBufferInvalidate(dataBuffer_0, sizeof(dataBuffer_0));

    // copy to pointer
    memcpy(pData, dataBuffer_0, size);

    return true;
}

static bool SPI_Write(const uint8_t *pData, uint32_t writeAddr, uint32_t size)
{
    uint32_t writeSize;
    uint32_t address = writeAddr;

    // perform paged program
    while (size > 0)
    {
        // send write enable
        dataBuffer_0[0] = WRITE_ENABLE_CMD;
        cacheBufferFlush(dataBuffer_0, sizeof(dataBuffer_0));

        CS_SELECT;
        spiSend(&SPID3, 1, dataBuffer_0);
        CS_UNSELECT;

        // calculate write size
        writeSize = __builtin_fmin(W25Q64_PAGE_SIZE - (address % W25Q64_PAGE_SIZE), size);

        // send write page
        dataBuffer_0[0] = PAGE_PROG_CMD;
        dataBuffer_0[1] = (uint8_t)(address >> 16);
        dataBuffer_0[2] = (uint8_t)(address >> 8);
        // adjust address if writeSize is the full page
        dataBuffer_0[3] = (uint8_t)(writeSize == W25Q64_PAGE_SIZE ? address & 0xFFFFFFF0 : address);

        CS_SELECT;
        spiSend(&SPID3, 4, dataBuffer_0);

        // copy from buffer
        memcpy(dataBuffer_0, pData, writeSize);

        spiSend(&SPID3, writeSize, dataBuffer_0);
        CS_UNSELECT;

        // wait for operation to complete
        SPI_WaitOnBusy();

        address += writeSize;
        pData += writeSize;
        size -= writeSize;
    }

    return true;
}

#endif // LFS_SPI1

int8_t target_lfs_init()
{
#ifdef LFS_SPI1

    // init driver
    spiAcquireBus(&SPID3);
    spiStart(&SPID3, &spiConfig);

    ////////////////////////////////////////////////////////////////////////
    // no need to worry with cache issues at this early stage of the boot //
    ////////////////////////////////////////////////////////////////////////

    // resume from deep power down
    // have to send this to make sure device is functional after sleep
    dataBuffer_0[0] = RESUME_DEEP_PD_CMD;

    CS_SELECT;
    spiSend(&SPID3, 1, dataBuffer_0);
    CS_UNSELECT;

    // sanity check: read device ID and unique ID
    dataBuffer_0[0] = READ_ID_CMD2;

    // from W25Q64 datasheet: need time for chip to become fully responsive
    chThdSleepMilliseconds(10);

    CS_SELECT;
    spiSend(&SPID3, 1, dataBuffer_0);
    spiReceive(&SPID3, 3, dataBuffer_0);
    CS_UNSELECT;

    // constants from ID Definitions table in W25Q64 datasheet
    ASSERT(dataBuffer_0[0] == W25Q64_MANUFACTURER_ID);
    ASSERT(dataBuffer_0[1] == W25Q64_DEVICE_ID1);
    ASSERT(dataBuffer_0[2] == W25Q64_DEVICE_ID2);

#endif // LFS_SPI1

    return LFS_ERR_OK;
}

// target specific implementation of hal_lfs_sync
int32_t hal_lfs_sync_0(const struct lfs_config *c)
{
    (void)c;

    __DSB();

    return 0;
}

// target specific implementation of hal_lfs_sync
int32_t hal_lfs_sync_1(const struct lfs_config *c)
{
    (void)c;

    __DSB();

    return 0;
}
