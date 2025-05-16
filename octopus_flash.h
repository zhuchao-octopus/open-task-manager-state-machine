
/*******************************************************************************
 * @file     octopus_task_manager_flash.h
 * @brief    Header file for managing Flash operations in the Octopus project.
 * @details  This file provides function declarations for reading and writing
 *           data to and from Flash memory. It also includes functions for
 *           printing data in a hexadecimal format for debugging purposes.
 * @version  1.0
 * @date     2024-12-12
 * @author   Octopus Team
 ******************************************************************************/
#ifndef __OCTOPUS_TASK_MANAGER_FLASH_H__
#define __OCTOPUS_TASK_MANAGER_FLASH_H__
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
#include "octopus_platform.h"  ///< Include platform-specific configurations
#include "octopus_flash_hal.h" ///< Include Flash Hardware Abstraction Layer (HAL) for low-level operations
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************
 * Struct Definition
 *******************************************************/
typedef struct
{
    uint32_t user_APP_flag;
    uint32_t user_data_flag;
    uint32_t crc; // CRC32 value for data integrity

    uint32_t flag; // Status flag for application state
} app_meta_data_t;

extern app_meta_data_t app_meta_data;

#define USE_EEROM_FOR_DATA_SAVING

#define EEROM_START_ADDRESS (0x00000000)

// #define EEROM_APPPP_VALID_ADDRESS       (EEROM_START_ADDRESS)
// #define EEROM_APPPP_CRC_ADDRESS        	(EEROM_START_ADDRESS + 4)
// #define EEROM_DATAS_VALID_ADDRESS       (EEROM_START_ADDRESS + 8)

#define EEROM_DATAS_ADDRESS (EEROM_START_ADDRESS + 1024)
#define CARINFOR_METER_EE_READ_ADDRESS (EEROM_DATAS_ADDRESS + 0)

#define EEROM_DATAS_VALID_FLAG (0xAA55)
#define EEROM_APPPP_VALID_FLAG (0x55AA)

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************
 * MCU Flash Memory Layout Configuration
 *******************************************************/
#define FLASH_BLOCK_SIZE (1024)                                 /* FLASH Page Size 1KB(1024)*/
#define FLASH_TOTAL_BLOCK (128)                                 // Total Flash size: 128KB
#define FLASH_TOTAL_SIZE (FLASH_TOTAL_BLOCK * FLASH_BLOCK_SIZE) // Total Flash size: 128KB

/*************** Bootloader Configuration ***************/
#define BOOTLOADER_BLOCK_COUNT (20)
#define BOOTLOADER_START_ADDR (0x08000000)                          // Bootloader start address
#define BOOTLOADER_SIZE (BOOTLOADER_BLOCK_COUNT * FLASH_BLOCK_SIZE) // Bootloader size: 20KB
#define BOOTLOADER_END_ADDR (BOOTLOADER_START_ADDR + BOOTLOADER_SIZE)

/*************** Main Application Configuration ***************/
#define MAIN_APP_START_ADDR (BOOTLOADER_END_ADDR) // Main Application start address
#define MAIN_APP_BLOCK_COUNT (FLASH_TOTAL_BLOCK - BOOTLOADER_BLOCK_COUNT)
#define MAIN_APP_SIZE (MAIN_APP_BLOCK_COUNT * FLASH_BLOCK_SIZE) // Main Application size: 100KB
#define MAIN_APP_END_ADDR (MAIN_APP_START_ADDR + MAIN_APP_SIZE)

/*************** Reserved or Free Space ***************/
#define RESERVED_START_ADDR (MAIN_APP_END_ADDR)
#define RESERVED_SIZE (FLASH_TOTAL_SIZE - (BOOTLOADER_SIZE + MAIN_APP_SIZE))
#define RESERVED_END_ADDR (RESERVED_START_ADDR + RESERVED_SIZE)

/* Debug Information */
#define DEBUG_BOOTLOADER_ADDR_INFO() \
    LOG_LEVEL("bootloader address: 0x%08X,0x%08X\n", BOOTLOADER_START_ADDR, BOOTLOADER_END_ADDR)

#define DEBUG_MAIN_APP_ADDR_INFO() \
    LOG_LEVEL("user apppp address: 0x%08X,0x%08X\n", MAIN_APP_START_ADDR, MAIN_APP_END_ADDR)

#ifdef __cplusplus
extern "C"
{
#endif

    /******************************************************************************/
    /**
     * @brief    Declarations of local functions for Flash operations.
     * @details  These functions include Flash read/write operations and debugging
     *           utilities for printing Flash data in hexadecimal format.
     */
    extern void PrintfBuffHex(const char *fun, int line, char *str, uint8_t *dat, int len);
    /**< Prints a buffer in hexadecimal format for debugging. */
    extern void FlashReadToBuff(uint32_t addr, uint8_t *buf, uint32_t len);
    /**< Reads data from Flash memory into a buffer. */
    extern uint32_t FlashWriteBuffTo(uint32_t addr, uint8_t *buf, uint32_t len);
    /**< Writes data from a buffer to Flash memory. */
    extern void E2ROMReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length);
    extern void E2ROMWriteBuffTo(uint32_t addr, uint8_t *buf, uint32_t length);
    extern void BootloaderMainLoopEvent(void);

    void flash_print_user_data_infor(void);
    void flash_init(void);
    uint32_t Flash_erase_user_app_arear(void);
#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_TASK_MANAGER_FLASH_H__ */
