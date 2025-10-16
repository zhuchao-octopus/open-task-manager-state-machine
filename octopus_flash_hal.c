/**
 ******************************************************************************
 * @file    octopus_flash_hal.c
 * @author  Octopus Hardware Team
 * @brief   Hardware Abstraction Layer for Flash and EEPROM operations.
 *
 * This file provides unified Flash and EEPROM read/write/erase functions
 * for different RTOS or bare-metal platforms (CST OSAL, Nation RTOS, STM32 RTOS).
 *
 * Supported Features:
 *   - Flash read/write/erase with 4-byte alignment validation
 *   - Page-based Flash erase with automatic locking/unlocking
 *   - EEPROM read/write interface through I2C abstraction
 *   - Platform-adaptive implementation via conditional compilation
 *
 * @version 0.2
 * @date    2020-08-13
 *
 * @note    Copyright (c) 2020-2025 Octopus Co., Ltd.
 * @note    All rights reserved.
 ******************************************************************************
 */

/*********************************************************************
 * INCLUDES
 */
#include "octopus_flash_hal.h"
#include "octopus_platform.h"

/* -------------------------------------------------------------------------- */
/*                              Macro Definitions                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Flash memory page size definition.
 *
 * For STM32-series MCUs, each Flash page is 1KB.
 * Adjust this value according to your MCU’s memory organization.
 */
#define FLASH_PAGE_SIZE ((uint32_t)0x00000400) /* 1KB per page */
/* -------------------------------------------------------------------------- */
/*                              Public Functions                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize Flash HAL driver.
 * @param  task_id: ID of the task requesting initialization (reserved)
 * @retval None
 */
void hal_flash_init(uint8_t task_id)
{
    LOG_LEVEL("hal flash init\r\n");
}

/* -------------------------------------------------------------------------- */
/*                Platform-specific Implementations (RTOS Variants)           */
/* -------------------------------------------------------------------------- */
#ifdef PLATFORM_CST_OSAL_RTOS

uint32_t hal_flash_read_(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
    HalFlashRead(startaddr, buffer, length);
    return 0;
}
uint32_t hal_flash_erase_page_(uint32_t startaddr, uint8_t page_count)
{
    return 0;
}
uint32_t hal_flash_erase_area_(uint32_t startaddr, uint32_t endaddr)
{
    return 0;
}

uint32_t hal_flash_write_(uint32_t startaddr, uint8_t *buffer, uint32_t length)
{
    return 0;
}
#elif defined(PLATFORM_NATION_RTOS)

uint32_t hal_flash_read_(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
    return 0;
}
uint32_t hal_flash_erase_page_(uint32_t startaddr, uint8_t page_count)
{
    return 0;
}
uint32_t hal_flash_erase_area_(uint32_t startaddr, uint32_t endaddr)
{
    return 0;
}

uint32_t hal_flash_write_(uint32_t startaddr, uint8_t *buffer, uint32_t length)
{
    return 0;
}
/* -------------------------------------------------------------------------- */
/*                       STM32 Platform Flash Driver                          */
/* -------------------------------------------------------------------------- */
#elif defined(PLATFORM_STM32_RTOS)
/**
 * @brief  Erase one or more Flash pages starting from a specific address.
 * @param  startaddr: Start address of the first page to erase
 * @param  page_count: Number of pages to erase
 * @retval Number of successfully erased pages
 */
uint32_t hal_flash_erase_page_(uint32_t startaddr, uint8_t page_count)
{
    uint32_t i = 0;
    /* Unlock the Flash to enable the flash control register access *************/
    FLASH_Unlock();

    /* Erase the user Flash area
      (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

    /* Clear pending flags (if any) */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_SIZE_ERR);

    /* Define the number of page to be erased */
    // NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;

    /* Erase the FLASH pages */
    for (i = 0; i < page_count; i++)
    {
        if (FLASH_ErasePage(startaddr + (FLASH_PAGE_SIZE * i)) != FLASH_COMPLETE)
        {
            /* Error occurred while sector erase.
                User can add here some code to deal with this error  */
            while (1)
            {
            }
        }
    }
    /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
    FLASH_Lock();
    return i;
}
/**
 * @brief  Erase a specific area of Flash memory.
 * @param  startaddr: Start address of the erase area
 * @param  endaddr:   End address of the erase area (exclusive)
 * @retval Number of successfully erased pages
 */
uint32_t hal_flash_erase_area_(uint32_t startaddr, uint32_t endaddr)
{
    uint32_t i = 0;
    /* Unlock the Flash to enable the flash control register access *************/
    FLASH_Unlock();

    /* Erase the user Flash area
      (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

    /* Clear pending flags (if any) */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_SIZE_ERR);

    /* Define the number of page to be erased */
    uint32_t NbrOfPage = (endaddr - startaddr) / FLASH_PAGE_SIZE;

    /* Erase the FLASH pages */
    for (i = 0; i < NbrOfPage; i++)
    {
        if (FLASH_ErasePage(startaddr + (FLASH_PAGE_SIZE * i)) != FLASH_COMPLETE)
        {
            /* Error occurred while sector erase.
                User can add here some code to deal with this error  */
            while (1)
            {
            }
        }
    }
    /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
    FLASH_Lock();
    return i;
}
/**
 * @brief  Read data from Flash memory.
 * @param  startaddr: Start address in Flash memory
 * @param  buffer:    Destination buffer pointer
 * @param  length:    Length in bytes to read
 * @retval Number of bytes successfully read
 */
uint32_t hal_flash_read_(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
// Directly read from the flash memory
#ifdef PLATFORM_CST_OSAL_RTOS
    HalFlashRead(startaddr, buffer, length);
#else
    memcpy(buffer, (const void *)startaddr, length);
#endif
    return length; // Return the number of bytes read
}
/**
 * @brief  Write data to Flash memory word-by-word (4-byte aligned).
 * @param  startaddr: Start address in Flash memory
 * @param  buffer:    Pointer to data buffer
 * @param  length:    Length in bytes (must be 4-byte aligned)
 * @retval Number of bytes successfully written (0 if failed)
 */
uint32_t hal_flash_writ_(uint32_t startaddr, uint8_t *buffer, uint32_t length)
{
    uint32_t Address = startaddr;
    uint32_t *data = (uint32_t *)buffer;
    uint32_t written_bytes = 0;
    /* Unlock the Flash for write access */
    if ((length & (sizeof(uint32_t) - 1U)) || ((uint32_t)buffer & (sizeof(uint32_t) - 1U)))
        return 0;
    /* Unlock the Flash for write access */
    FLASH_Unlock();
    /* Clear pending flags (if any) */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_SIZE_ERR);
    /* Write the data word by word */
    for (uint32_t i = 0; i < length / sizeof(uint32_t); i++)
    {
        if (FLASH_ProgramWord(Address, data[i]) == FLASH_COMPLETE)
        {
            Address += sizeof(uint32_t);
            written_bytes += sizeof(uint32_t);
        }
        else
        {
            FLASH_Lock();
            return 0; // Writing failed, return 0 bytes
        }
    }

    /* Lock the Flash after writing */
    FLASH_Lock();
    return written_bytes; // Return the number of bytes written
}
#else
/* Dummy implementations for unsupported or test environments */
uint32_t hal_flash_read_(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
    return 0;
}
uint32_t hal_flash_erase_page_(uint32_t startaddr, uint8_t page_count)
{
    return 0;
}
uint32_t hal_flash_erase_area_(uint32_t startaddr, uint32_t endaddr)
{
    return 0;
}

uint32_t hal_flash_write_(uint32_t startaddr, uint8_t *buffer, uint32_t length)
{
    return 0;
}
#endif
/* -------------------------------------------------------------------------- */
/*                         EEPROM Access Layer Functions                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Write data to external EEPROM via I2C interface.
 * @param  startaddr: EEPROM memory start address
 * @param  buffer:    Pointer to data buffer
 * @param  length:    Length in bytes to write
 * @retval None
 */
void hal_eeprom_writ_(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
    // #ifdef FLASH_USE_EEROM_FOR_DATA_SAVING
    uint8_t ret = I2C_EepromBufferWrite(startaddr, buffer, length);
    // if (ret == ERROR)
    LOG_LEVEL("Save data to eeprom status:%d\r\n", ret);
    // #endif
}
/**
 * @brief  Read data from external EEPROM via I2C interface.
 * @param  startaddr: EEPROM memory start address
 * @param  buffer:    Pointer to destination buffer
 * @param  length:    Length in bytes to read
 * @retval None
 */
void hal_eeprom_read_(uint32_t startaddr, uint8_t *buffer, uint8_t length)
{
    // #ifdef FLASH_USE_EEROM_FOR_DATA_SAVING
    uint8_t ret = EEPROM_Read(startaddr, buffer, length);
    LOG_LEVEL("read data from eeprom status:%d\r\n", ret);
    // #endif
}
/* -------------------------------------------------------------------------- */
/*                                End of File                                 */
/* -------------------------------------------------------------------------- */
