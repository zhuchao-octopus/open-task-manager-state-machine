
/*******************************************************************************
 * @file     octopus_task_manager_flash.h
 * @brief    Header file for managing Flash operations in the Octopus project.
 * @details  This file provides function declarations for reading and writing
 *           data to and from Flash memory. It also includes functions for
 *           printing data in a hexadecimal format for debugging purposes.
 * @version  1.0
 * @date     2024-12-12
 * @author   Octopus Team
 ***************************************************************************************/
#ifndef __OCTOPUS_TASK_MANAGER_FLASH_H__
#define __OCTOPUS_TASK_MANAGER_FLASH_H__
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
#include "octopus_platform.h"  ///< Include platform-specific configurations
#include "octopus_flash_hal.h" ///< Include Flash Hardware Abstraction Layer (HAL) for low-level operations

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************
 * User Data EEPROM Struct Definition
 ***************************************************************************************/

#define EEPROM_APP_META_SIZE (128) // 128byte app meta struct total 2kb

#define EEROM_START_ADDRESS (0x00000000)
#define EEROM_APP_MATA_ADDRESS (EEROM_START_ADDRESS)
#define EEROM_DATAS_ADDRESS (EEROM_APP_MATA_ADDRESS + EEPROM_APP_META_SIZE) // user data area from app meta struct

#define EEROM_CARINFOR_METER_ADDRESS (EEROM_DATAS_ADDRESS + 0)

#define EEROM_DATAS_VALID_FLAG (0xAA55)
#define EEROM_APPPP_VALID_FLAG (0x55AA)

#define BOOTLOADER_ACTIVE_SLOT_A (0)
#define BOOTLOADER_ACTIVE_SLOT_B (1)

#define APP_FLAG_SLOT_A_NEED_UPGRADE (1 << 1)
#define APP_FLAG_SLOT_B_NEED_UPGRADE (1 << 2)
#define APP_FLAG_SLOT_A_NEED_REBOOT (1 << 3)
#define APP_FLAG_SLOT_B_NEED_REBOOT (1 << 4)
#define APP_FLAG_VALID_A (1 << 5)
#define APP_FLAG_VALID_B (1 << 6)
#define APP_FLAG_FORCE_SLOT_A (1 << 7)
#define APP_FLAG_FORCE_SLOT_B (1 << 8)

typedef enum
{
    BOOT_MODE_SINGLE_BANK_NONE = 0,
    BOOT_MODE_SINGLE_BANK_NO_LOADER,   // Bank,Bootloader
    BOOT_MODE_DUAL_BANK_NO_LOADER,     // Bank,Bootloader
    BOOT_MODE_SINGLE_BANK_WITH_LOADER, // Bank,Bootloader
    BOOT_MODE_DUAL_BANK_WITH_LOADER    // Bank,Bootloader
} boot_mode_t;

typedef struct
{
    uint32_t bootloader_addr; // Bootloader entry address

    uint32_t slot_a_addr; // Flash base address of application Slot A
    uint32_t slot_b_addr; // Flash base address of application Slot B

    uint32_t app_crc_slot_a; // CRC32 checksum for Slot A application
    uint32_t app_crc_slot_b; // CRC32 checksum for Slot B application

    uint32_t app_state_flags;   // Bit flags for application upgrade status
    uint32_t meter_data_flags;  // Flags for meter/user data integrity
    uint32_t config_data_flags; // Flags for config data state

    uint8_t active_slot;  // Active slot indicator: 0 = A, 1 = B
    uint8_t last_boot_ok; // Last boot result: 0 = fail, 1 = success
    uint8_t boot_mode;    // Boot mode: defined by enum (e.g., normal, upgrade, recovery)
    uint8_t reserved;     // Reserved for alignment or future fields
} app_meta_data_t;

extern app_meta_data_t app_meta_data;
/////////////////////////////////////////////////////////////////////////////////////////

#define BOOTLOADER_CONFIG_MODE_TYPE BOOT_MODE_DUAL_BANK_NO_LOADER
#define BOOTLOADER_CONFIG_MODE_BANK BOOTLOADER_ACTIVE_SLOT_A
/////////////////////////////////////////////////////////////////////////////////////////
/*******************************************************
 * MCU Flash Memory Layout Configuration
 *******************************************************/
#define FLASH_BLOCK_SIZE (1024)                                 // 0x00000400                    /* FLASH Page Size 1KB(1024)*/
#define FLASH_TOTAL_BLOCK (128)                                 // Total Flash size: 128KB
#define FLASH_TOTAL_SIZE (FLASH_TOTAL_BLOCK * FLASH_BLOCK_SIZE) // Total Flash size: 128KB

/*************** Bootloader Configuration ***************/
#if (BOOTLOADER_CONFIG_MODE <= BOOT_MODE_DUAL_BANK_NO_LOADER)
#define BOOTLOADER_BLOCK_COUNT (0)
#else
#define BOOTLOADER_BLOCK_COUNT (10)
#endif

#define BOOTLOADER_START_ADDR (0x08000000)                          // Bootloader start address
#define BOOTLOADER_SIZE (BOOTLOADER_BLOCK_COUNT * FLASH_BLOCK_SIZE) // Bootloader size: 20KB
#define BOOTLOADER_END_ADDR (BOOTLOADER_START_ADDR + BOOTLOADER_SIZE)

/*************** Main Application Configuration ***************/

#define MAIN_APP_BLOCK_COUNT ((FLASH_TOTAL_BLOCK - BOOTLOADER_BLOCK_COUNT) / 2)
#define MAIN_APP_SIZE (MAIN_APP_BLOCK_COUNT * FLASH_BLOCK_SIZE) // Main Application size: 100KB

// #define MAIN_APP_END_ADDR (MAIN_APP_START_ADDR + MAIN_APP_SIZE)

#define MAIN_APP_SLOT_A_START_ADDR (BOOTLOADER_END_ADDR)                        // Main Application start address
#define MAIN_APP_SLOT_B_START_ADDR (MAIN_APP_SLOT_A_START_ADDR + MAIN_APP_SIZE) // Main Application start address

/*************** Reserved or Free Space ***************/
// #define RESERVED_START_ADDR (MAIN_APP_END_ADDR)
// #define RESERVED_SIZE (FLASH_TOTAL_SIZE - (BOOTLOADER_SIZE + MAIN_APP_SIZE))
// #define RESERVED_END_ADDR (RESERVED_START_ADDR + RESERVED_SIZE)

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

    void flash_load_user_data_infor(void);
    void flash_save_carinfor_meter(void);
    void flash_init(void);
    uint32_t Flash_erase_user_app_arear(void);
    uint32_t CalculateCRC32(uint8_t *data, uint32_t length);
#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_TASK_MANAGER_FLASH_H__ */
