
/*******************************************************************************
 * @file     octopus_task_manager_flash_hal.h
 * @brief    Header file for the Flash Hardware Abstraction Layer (HAL).
 *           This file provides the interface for initializing and accessing
 *           the flash memory in the Octopus project.
 * @details  The functions declared in this file are used to interact with
 *           flash memory, including initialization and reading data into a
 *           buffer. The implementation of these functions is dependent on
 *           the platform and hardware being used.
 * @version  1.0
 * @date     2024-12-12
 * @author   Octopus Team
 ******************************************************************************/

#ifndef __OCTOPUS_TASK_MANAGER_FLASH_HAL_H__
#define __OCTOPUS_TASK_MANAGER_FLASH_HAL_H__

#include "octopus_base.h" //  Base include file for the Octopus project.

// Macro for user flash testing.
// This could be used for testing purposes or debugging flash operations.
#define USR_FLASH_TEST 0x0001

#ifdef __cplusplus
extern "C"
{
#endif
    /******************************************************************************/
    /**
     * @brief   Initializes the flash memory module.
     * @param   task_id The ID of the task that will use the flash.
     *                  This can be used to associate the flash initialization
     *                  with a specific task or operation.
     * @retval  None
     * @details This function initializes the flash memory for use. It configures
     *          the necessary hardware settings and prepares the flash module for
     *          read/write operations.
     */
    void hal_flash_init(uint8_t task_id);

    /* -------------------------------------------------------------------------- */
    /*                        Flash Memory Access Interfaces                      */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief  Erase one or more Flash pages starting from a specific address.
     *
     * @param  startaddr   Start address of the first page to erase.
     * @param  page_count  Number of pages to erase.
     *
     * @retval Number of successfully erased pages.
     * @note   This function automatically unlocks and locks the Flash.
     */
    uint32_t hal_flash_erase_page_(uint32_t startaddr, uint8_t page_count);

    /**
     * @brief  Erase a continuous area in Flash memory.
     *
     * @param  startaddr   Start address of the erase area.
     * @param  endaddr     End address of the erase area (exclusive).
     *
     * @retval Number of successfully erased pages.
     * @note   The area size should be aligned with Flash page boundaries.
     */
    uint32_t hal_flash_erase_area_(uint32_t startaddr, uint32_t endaddr);

    /**
     * @brief  Read a data buffer directly from Flash memory.
     *
     * @param  startaddr   Start address in Flash memory.
     * @param  buffer      Pointer to the destination buffer.
     * @param  length      Number of bytes to read.
     *
     * @retval Number of bytes successfully read.
     * @note   The read operation does not require Flash unlocking.
     */
    uint32_t hal_flash_read_(uint32_t startaddr, uint8_t *buffer, uint8_t length);

    /**
     * @brief  Write a data buffer to Flash memory (4-byte aligned).
     *
     * @param  startaddr   Start address in Flash memory.
     * @param  buffer      Pointer to source data buffer.
     * @param  length      Number of bytes to write. Must be 4-byte aligned.
     *
     * @retval Number of bytes successfully written (0 if failed).
     * @note   This function automatically handles Flash unlocking and locking.
     *         Both buffer address and length must be 4-byte aligned.
     */
    uint32_t hal_flash_writ_(uint32_t startaddr, uint8_t *buffer, uint32_t length);

    /* -------------------------------------------------------------------------- */
    /*                        EEPROM Access Layer Interfaces                      */
    /* -------------------------------------------------------------------------- */

    /**
     * @brief  Write data to external EEPROM via I2C interface.
     *
     * @param  startaddr   EEPROM memory start address.
     * @param  buffer      Pointer to source data buffer.
     * @param  length      Number of bytes to write.
     *
     * @retval None
     * @note   Function internally calls `I2C_EepromBufferWrite()`.
     *         Log messages are printed upon completion.
     */
    void hal_eeprom_writ_(uint32_t startaddr, uint8_t *buffer, uint8_t length);

    /**
     * @brief  Read data from external EEPROM via I2C interface.
     *
     * @param  startaddr   EEPROM memory start address.
     * @param  buffer      Pointer to destination buffer.
     * @param  length      Number of bytes to read.
     *
     * @retval None
     * @note   Function internally calls `EEPROM_Read()`.
     *         Log messages are printed upon completion.
     */
    void hal_eeprom_read_(uint32_t startaddr, uint8_t *buffer, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_TASK_MANAGER_FLASH_HAL_H__ */
