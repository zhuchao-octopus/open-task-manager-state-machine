/**
 * @file	halPeripheral.c
 * @author	chipsea
 * @brief	
 * @version	0.1
 * @date	2020-11-30
 * @copyright Copyright (c) 2020, CHIPSEA Co., Ltd.
 * @note
 */

/*********************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"
#include "octopus_flash_hal.h"
#include "octopus_log.h"

void hal_flash_init(uint8_t task_id)
{	
#ifdef PLATFORM_CST_OSAL_RTOS
	///HalDMAInit();
	///HalDMAInitChannel(dma_cfg);
 #endif
	LOG_LEVEL("hal flash init\r\n");
}

void hal_flash_read_to_buff(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
     #ifdef PLATFORM_CST_OSAL_RTOS
	 HalFlashRead(addr, buf, len);
     #endif
}

/**
 * @brief  Read a data buffer from Flash.
 * @param  addr: Start address in Flash memory
 * @param  buf: Destination buffer to read into
 * @param  len: Length in bytes
 */
void hal_flash_read_buff(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
    memcpy(buffer, (const void *)startaddr, length);
}
/**
 * @brief  Write a data buffer to Flash with auto-erasure.
 * @param  addr: Start address in Flash memory
 * @param  buf: Pointer to data buffer
 * @param  len: Length in bytes
 */
void hal_flash_write_buff(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
    uint32_t i, page_start, page_end, page;

    // Unlock Flash memory for write/erase
    FLASH_Unlock();

    // --- Step 1: Calculate how many pages to erase ---
    page_start = startaddr / FLASH_PAGE_SIZE;
    page_end   = (startaddr + length - 1) / FLASH_PAGE_SIZE;

    for (page = page_start; page <= page_end; ++page)
    {
        FLASH_ErasePage(page * FLASH_PAGE_SIZE);  // Erase each needed page
    }

    // --- Step 2: Write data in 32-bit word format ---
    for (i = 0; i < length; i += 4)
    {
        uint32_t word = 0xFFFFFFFF;

        // Handle partial word at the end
        uint8_t b0 = (i < length)     ? buffer[i]     : 0xFF;
        uint8_t b1 = (i+1 < length)   ? buffer[i+1]   : 0xFF;
        uint8_t b2 = (i+2 < length)   ? buffer[i+2]   : 0xFF;
        uint8_t b3 = (i+3 < length)   ? buffer[i+3]   : 0xFF;

        word = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);

        if (FLASH_ProgramWord(startaddr + i, word) != FLASH_COMPLETE)
        {
            // Writing error occurred, you can log or handle it here
            while (1);  // Stay here for debug
        }
    }

    // Lock Flash again to protect it
    FLASH_Lock();
}

void hal_flash_write_buff_dma(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
     #ifdef PLATFORM_CST_OSAL_RTOS
	 HalFlashWriteByDma(addr, buf, len);
     #endif
}

void hal_flash_save(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
	I2C_EepromPageWrite(startaddr,buffer,length);
}

void hal_flash_read(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
	I2C_EepromPageWrite(startaddr,buffer,length);
}
