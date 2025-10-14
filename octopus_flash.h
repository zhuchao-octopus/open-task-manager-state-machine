
/*******************************************************************************
 * @file     octopus_flash.h
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
#include "octopus_base.h"      //  Base include file for the Octopus project.
#include "octopus_flash_hal.h" ///< Include Flash Hardware Abstraction Layer (HAL) for low-level operations

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/***************************************************************************************
 * User Data EEPROM Struct Definition
 ***************************************************************************************/
#define EEPROM_START_ADDRESS (0x00000000)
#define EEPROM_FLASH_META_SIZE (128) // 128byte app meta struct total 2kb
#define EEROM_SYSTEM_METER_SIZE (128)

#define EEROM_FLASH_MATA_ADDRESS (EEPROM_START_ADDRESS)
#define EEROM_SYSTEM_METER_ADDRESS (EEROM_FLASH_MATA_ADDRESS + EEPROM_FLASH_META_SIZE)

#define EEROM_DATAS_ADDRESS (EEROM_SYSTEM_METER_ADDRESS + EEROM_SYSTEM_METER_SIZE) // user data area from app meta struct
#define EEROM_CARINFOR_METER_ADDRESS (EEROM_DATAS_ADDRESS + 0)
#define EEROM_CARINFOR_BATTERY_ADDRESS (EEROM_CARINFOR_METER_ADDRESS + 128)

// #define EEROM_DATAS_VALID_FLAG (0xAA55)
// #define EEROM_APPPP_VALID_FLAG (0x55AA)

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
// Application Slot A requires an upgrade
#define APP_FLAG_SLOT_A_NEED_UPGRADE (1U << 1)

// Application Slot B requires an upgrade
#define APP_FLAG_SLOT_B_NEED_UPGRADE (1U << 2)

// Slot A application requires a system reboot to take effect
#define APP_FLAG_SLOT_A_NEED_REBOOT (1U << 3)

// Slot B application requires a system reboot to take effect
#define APP_FLAG_SLOT_B_NEED_REBOOT (1U << 4)

// Slot A application is verified and valid
#define APP_FLAG_VALID_A (1U << 5)

// Slot B application is verified and valid
#define APP_FLAG_VALID_B (1U << 6)

// Force booting from Slot A regardless of current active slot
#define APP_FLAG_FORCE_SLOT_A (1U << 7)

// Force booting from Slot B regardless of current active slot
#define APP_FLAG_FORCE_SLOT_B (1U << 8)

// Check if upgrade is needed for Slot A
#define IS_SLOT_A_NEED_UPGRADE(flags) ((flags) & APP_FLAG_SLOT_A_NEED_UPGRADE)

// Check if upgrade is needed for Slot B
#define IS_SLOT_B_NEED_UPGRADE(flags) ((flags) & APP_FLAG_SLOT_B_NEED_UPGRADE)

// Check if reboot is needed for Slot A
#define IS_SLOT_A_NEED_REBOOT(flags) ((flags) & APP_FLAG_SLOT_A_NEED_REBOOT)

// Check if reboot is needed for Slot B
#define IS_SLOT_B_NEED_REBOOT(flags) ((flags) & APP_FLAG_SLOT_B_NEED_REBOOT)

// Check if Slot A is marked as valid
#define IS_SLOT_A_VALID(flags) ((flags) & APP_FLAG_VALID_A)

// Check if Slot B is marked as valid
#define IS_SLOT_B_VALID(flags) ((flags) & APP_FLAG_VALID_B)

// Check if Slot A is forced to boot
#define IS_FORCE_BOOT_SLOT_A(flags) ((flags) & APP_FLAG_FORCE_SLOT_A)

// Check if Slot B is forced to boot
#define IS_FORCE_BOOT_SLOT_B(flags) ((flags) & APP_FLAG_FORCE_SLOT_B)

// Check if a specific flag is set
#define IS_FLAG_SET(flags, flag) (((flags) & (flag)) != 0)
// Set a specific flag
#define SET_FLAG(flags, flag) ((flags) |= (flag))
// Clear a specific flag
#define CLEAR_FLAG(flags, flag) ((flags) &= ~(flag))
// Toggle a specific flag
#define TOGGLE_FLAG(flags, flag) ((flags) ^= (flag))

/////////////////////////////////////////////////////////////////////////////////////////
typedef enum
{
    BANK_SLOT_LOADER = 0, // Bootloader bank (reserved for the loader)
    BANK_SLOT_A,          // Application Slot A
    BANK_SLOT_B,          // Application Slot B
    BANK_SLOT_INVALID,    // Invalid bank or No bank mode
} boot_bank_t;

typedef enum
{
    BOOT_MODE_SINGLE_BANK_NONE = 0,    // No bootloader or second bank present
    BOOT_MODE_SINGLE_BANK_NO_LOADER,   // Single bank without a bootloader
    BOOT_MODE_SINGLE_BANK_WITH_LOADER, // Single bank with bootloader present
    BOOT_MODE_DUAL_BANK_NO_LOADER,     // Two banks, but no dedicated bootloader
    BOOT_MODE_DUAL_BANK_WITH_LOADER,   // Two banks and a bootloader present
    BOOT_MODE_MAX
} boot_mode_t;

#pragma pack(push, 1)
typedef struct
{
    uint32_t loader_addr; // Bootloader entry address (if present)

    uint32_t slot_a_addr; // Start address of application Slot A in flash
    uint32_t slot_b_addr; // Start address of application Slot B in flash

    uint32_t slot_a_size; // Total size (bytes) of Slot A application
    uint32_t slot_b_size; // Total size (bytes) of Slot B application

    uint32_t slot_a_crc; // CRC32 checksum for Slot A application image
    uint32_t slot_b_crc; // CRC32 checksum for Slot B application image

    uint32_t loader_version; // Encoded version of the bootloader
    uint32_t slot_a_version; // Encoded version of the application in Slot A
    uint32_t slot_b_version; // Encoded version of the application in Slot B

    uint32_t slot_stat_flags; // Bit flags representing upgrade, validity, reboot needs, etc.
    uint32_t mete_data_flags; // Flags related to runtime/user/meter data validity
    uint32_t user_data_flags; // Flags related to configuration data state (e.g., checksum pass/fail)

    uint8_t bank_slot_activated; // Indicates the current active slot (1 = A, 2 = B)
    uint8_t bank_slot_mode;      // Current boot mode, corresponds to boot_mode_t
    uint8_t reserved1;
    uint8_t reserved2;

    uint32_t reserved3; // Reserved for future use or 4-byte alignment
    uint32_t reserved4; // Reserved for future use or 4-byte alignment

} flash_meta_infor_t;

typedef struct
{
    uint32_t trip_odo;      // Total distance traveled (unit: 1 meters), also known as trip odometer
    uint32_t trip_time;     // Total ride time (unit: seconds)
    uint32_t trip_distance; // Trip distance   (unit: 1 meters), resettable

    uint16_t speed_average; // Displayed vehicle speed (unit: 0.1 km/h)
    uint16_t speed_actual;  // Actual wheel speed (unit: 0.1 km/h)
    uint16_t speed_max;
    uint16_t speed_limit; // Speed limit setting; 0 = OFF, range: 10�C90 km/h

    uint8_t gear;           // Current gear level (0 = Neutral, 1�CN)
    uint8_t gear_level_max; // Maximum selectable gear level
    uint8_t wheel_diameter; // Wheel diameter (unit: inch)
    uint8_t reserve1;

    uint16_t rpm; // Motor RPM (raw value, offset by -20000)
    uint16_t reserve2;
} system_meter_infor_t;
#pragma pack(pop)

// Global instance holding metadata for application and bootloader
extern flash_meta_infor_t flash_meta_infor;
extern system_meter_infor_t system_meter_infor;
/////////////////////////////////////////////////////////////////////////////////////////
// * MCU Flash Memory Layout Configuration
/////////////////////////////////////////////////////////////////////////////////////////

#define FLASH_BLOCK_SIZE  (1024)                                 // 0x00000400  /* FLASH Page Size 1KB(1024)*/
#define FLASH_TOTAL_BLOCK (128)                                  // 128K 0x20000 Total Flash size: 128KB
#define FLASH_TOTAL_SIZE  (FLASH_TOTAL_BLOCK * FLASH_BLOCK_SIZE) // Total Flash size: 128KB

#define FLASH_BASE_START_ADDR (0x08000000)
#define FLASH_BASE_END_ADDR (FLASH_BASE_START_ADDR + FLASH_TOTAL_SIZE)

#define FLASH_BANK_MASK (0xFFFF0000)
#define FLASH_BANK_UNMASK (0x00FFFFFF)

#define FLASH_DATA_BLOCK (2)
#define FLASH_USER_DATA_BLOCK (FLASH_DATA_BLOCK - 1)
/////////////////////////////////////////////////////////////////////////////////////////
// Bootloader Configuration
// #define BOOTLOADER_CONFIG_MODE_TYPE BOOT_MODE_DUAL_BANK_NO_LOADER

#define FLASH_BOOTLOADER_START_ADDR (0x08000000)                                  // 0x08000000 + 0x20000 - 0x5000 // Bootloader start address
#define FLASH_BOOTLOADER_BLOCK_COUNT (40)                                         // 40K(0xA000)
#define FLASH_BOOTLOADER_SIZE ((FLASH_BOOTLOADER_BLOCK_COUNT) * FLASH_BLOCK_SIZE) // Bootloader size: 20KB
#define FLASH_BOOTLOADER_END_ADDR (FLASH_BOOTLOADER_START_ADDR + FLASH_BOOTLOADER_SIZE)

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
#define FLASH_USER_META_DATA_START_ADDRESS (FLASH_BASE_END_ADDR - FLASH_USER_DATA_BLOCK * FLASH_BLOCK_SIZE)
#define FLASH_META_DATA_START_ADDRESS (FLASH_USER_META_DATA_START_ADDRESS)
#define FLASH_SYSTEM_DATA_START_ADDRESS (FLASH_META_DATA_START_ADDRESS + 128)

#define FLASH_METER_DATA_START_ADDRESS (FLASH_SYSTEM_DATA_START_ADDRESS + 128)
//#define FLASH_USER_DATA_START_ADDRESS (FLASH_METER_DATA_START_ADDRESS + 128)

#define FLASH_META_DATAS_VALID_FLAG (0XA5A5)
/////////////////////////////////////////////////////////////////////////////////////////
// Main Application Configuration

// #define MAIN_APP_BLOCK_COUNT (FLASH_TOTAL_BLOCK - BOOTLOADER_BLOCK_COUNT - 2) //((FLASH_TOTAL_BLOCK - BOOTLOADER_BLOCK_COUNT) / 2)
// #define MAIN_APP_SIZE (MAIN_APP_BLOCK_COUNT * FLASH_BLOCK_SIZE) 							// Main Application size: 100KB

// #define MAIN_APP_END_ADDR (MAIN_APP_START_ADDR + MAIN_APP_SIZE)

// #define MAIN_APP_SLOT_A_START_ADDR (BOOTLOADER_END_ADDR + FLASH_BLOCK_SIZE)              // Main Application start address

// #define MAIN_APP_SLOT_B_START_ADDR (((FLASH_TOTAL_BLOCK - 2)/2 +1) * FLASH_BLOCK_SIZE)   // Main Application start address

/////////////////////////////////////////////////////////////////////////////////////////
#define IS_FLASH_ADDR(addr) ((addr) >= 0x08000000 && (addr) < 0x08100000)
/* Debug Information */
//#define DEBUG_BOOTLOADER_ADDR_INFO() \
//LOG_LEVEL("bootloader address: 0x%08X,0x%08X\n", BOOTLOADER_START_ADDR, BOOTLOADER_END_ADDR)

//#define DEBUG_MAIN_APP_ADDR_INFO() \
//LOG_LEVEL("user apppp address: 0x%08X,0x%08X\n", MAIN_APP_START_ADDR, MAIN_APP_END_ADDR)

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

	/**< Prints a buffer in hexadecimal format for debugging. */

	void otsm_flash_init(void);

	void flash_print_mcu_meta_infor(void);
	void flash_init_version(const char *date_str, const char *time_str);
	void flash_delay_ms(uint32_t ms);

	void flash_vector_table_config(boot_mode_t boot_mode, uint8_t bank_slot, uint32_t slot_address, bool mapping_vector);
	void flash_loader_active_user_app(uint8_t bank_slot, const char *date_str, const char *time_str);
	void flash_JumpToApplication(uint32_t app_address);
	void flash_load_sync_data_infor(void);

	flash_meta_infor_t *flash_get_meta_infor(void);
	uint32_t flash_get_app_max_size(void);
	uint32_t flash_erase_bank(uint8_t bank_slot);

	uint32_t flash_get_bank_slot_mode(void);
	uint32_t flash_get_current_bank(void);
	uint32_t flash_get_bank_address(uint8_t bank_slot);
	uint32_t flash_get_bank_offset_address(uint8_t bank_slot);


	const char *flash_get_current_bank_name(void);
	const char *flash_get_bank_name(uint8_t bank);

	bool flash_check_enter_upgrade_mode(void);
	bool flash_decode_active_version(uint8_t bank_slot, char *out_str, size_t max_len, const char *date_str, const char *time_str);
	bool flash_is_bank_address_valid(uint32_t b_address, uint32_t address);
	bool flash_is_meta_infor_valid(void);
	bool flash_is_allow_update_bank(uint8_t bank_type);
	bool flash_is_allow_update_address(uint32_t address);

	uint32_t FlashWritBuffTo(uint32_t addr, uint8_t *buf, uint32_t len);
	uint32_t FlashReadToBuff(uint32_t addr, uint8_t *buf, uint32_t len);

	void E2ROMReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length);
	void E2ROMWritBuffTo(uint32_t addr, uint8_t *buf, uint32_t length);

	void E2ROM_writ_metas_infor(void);
	void E2ROM_read_metas_infor(void);

	void E2ROM_writ_meter_infor(void);
	void E2ROM_read_meter_infor(void);

	void flash_writ_all_infor(void);
	void flash_read_all_infor(void);
	void flash_save_carinfor_meter(void);

#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_TASK_MANAGER_FLASH_H__ */
