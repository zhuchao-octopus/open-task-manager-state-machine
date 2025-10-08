/*************************************************************************************************
 * @file    bootloader_flash_jump_crc.c
 * @brief   Bootloader implementation for STM32, supporting flash jump and CRC verification.
 * @version 1.0
 * @date    2025-05-15
 * @author  Octopus Team
 *
 * This file contains the implementation of the bootloader for STM32 microcontrollers.
 * It includes functionalities for:
 *   - Jumping to the main application
 *   - Verifying application integrity using CRC32
 *   - Reading and writing to Flash memory
 *
 ************************************************************************************************/
#include "octopus_flash.h"
#include "octopus_utils.h"
#include "octopus_vehicle.h"
#include "octopus_platform.h"

// #ifdef FLASH_MAPPING_VECT_TABLE_TO_SRAM
#if (defined(__CC_ARM))
__IO uint32_t IRAM_Vector_Table[48] __attribute__((at(0x20000000)));
#elif defined(__GNUC__)
//__IO uint32_t IRAM_Vector_Table[48] __attribute__((section(".SRAM_VECTOR_TABLE")));
uint32_t IRAM_Vector_Table[48];
#elif defined(__TASKING__)
__IO uint32_t IRAM_Vector_Table[48] __at(0x20000000);
#else
uint32_t IRAM_Vector_Table[48] __at(0x20000000);
#endif
// #endif

void Print_VectorTable(void);
void Print_Flash_VectorTable(void);
void Print_SRAM_VectorTable(void);

bool flash_verify_bank_slot_crc(uint32_t slot_addr, uint32_t slot_size, uint32_t expected_crc);
void flash_goto_terget_bank(uint32_t active_app_addr, uint32_t expected_crc, uint32_t slot_length);

bool flash_check_vector_table(uint8_t bank_slot, uint32_t vector_address);
bool flash_is_first_boot(uint8_t bank_slot);
//////////////////////////////////////////////////////////////////////////////////////////////////

flash_meta_infor_t flash_meta_infor = {0};
system_meter_infor_t system_meter_infor = {0};
uint8_t flash_bank_config_mode_slot = BANK_SLOT_INVALID;
uint8_t flash_bank_config_mode_boot = BOOT_MODE_SINGLE_BANK_NONE;
uint32_t flash_bank_config_rom_address = FLASH_BOOTLOADER_END_ADDR + FLASH_BLOCK_SIZE;

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

void otsm_flash_init(void)
{
	hal_flash_init(0);
	// flash_meta_infor.bank_slot_mode = BOOT_MODE_SINGLE_BANK_NONE;
	// flash_meta_infor.loader_addr = FLASH_BOOTLOADER_START_ADDR;
	// flash_meta_infor.slot_a_addr = 0;
	// flash_meta_infor.slot_b_addr = 0;
}

void flash_init_version(const char *date_str, const char *time_str)
{
	switch (flash_bank_config_mode_slot)
	{
	case BANK_SLOT_LOADER:
		flash_meta_infor.loader_version = build_version_code(date_str, time_str);
		break;
	case BANK_SLOT_A:
		flash_meta_infor.slot_a_version = build_version_code(date_str, time_str);
		break;
	case BANK_SLOT_B:
		flash_meta_infor.slot_b_version = build_version_code(date_str, time_str);
		break;
	}
}

void flash_print_logo(void)
{
	LOG_NONE("\r\n");
	LOG_NONE("-----------------------------------------------------------------------------\r\n");
	LOG_NONE("               _____                                 \r\n");
	LOG_NONE(" ______ _________  /_______ ________ ____  __________\r\n");
	LOG_NONE(" _  __ \\_  ___/_  __/_  __ \\___  __ \\_  / / /__  ___/\r\n");
	LOG_NONE(" / /_/ // /__  / /_  / /_/ /__  /_/ // /_/ / _(__  ) \r\n");
	LOG_NONE(" \\____/ \\___/  \\__/  \\____/ _  .___/ \\__,_/  /____/  \r\n");
	LOG_NONE("                            /_/                       \r\n");
	LOG_NONE(" Embedded Real-Time Task Scheduler + FSM Engine\r\n");

	LOG_NONE(" Firmware  : v%s\r\n", OTMS_VERSION_NAME);
	LOG_NONE(" Compiled  : %s %s\r\n", __DATE__, __TIME__);
	LOG_NONE(" Module    : %s\r\n", flash_get_current_bank_name());
	LOG_NONE(" Author    : Octopus Dev Team\r\n");
	LOG_NONE("-----------------------------------------------------------------------------\r\n");
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
// LOCAL FUNCTIONS DECLEAR

void Print_VectorTable(void)
{
	// LOG_LEVEL("Interrupt Vector Table\r\n");
	for (int i = 0; i < 48; i++)
	{
		LOG_LEVEL("Vector_Table[%02d] = 0x%08X\r\n", i, IRAM_Vector_Table[i]);
	}
}

void Print_Flash_VectorTable(void)
{
	uint32_t entry = 0;
	if (flash_bank_config_mode_slot == BANK_SLOT_A)
	{
		// LOG_LEVEL("Flash Vector Table (from %08x)\r\n",MAIN_APP_SLOT_A_START_ADDR);
		for (uint8_t i = 0; i < 48; i++)
		{
			// entry = *(volatile uint32_t *)(MAIN_APP_SLOT_A_START_ADDR + i * 4);
			entry = *(volatile uint32_t *)((uintptr_t)flash_meta_infor.slot_a_addr + i * 4);
			LOG_LEVEL("Flash_Vector[%02d] = 0x%08X\r\n", i, entry);
		}
	}
	else
	{
		// LOG_LEVEL("Flash Vector Table (from %08x)\r\n",MAIN_APP_SLOT_B_START_ADDR);
		for (uint8_t i = 0; i < 48; i++)
		{
			// entry = *(volatile uint32_t *)(MAIN_APP_SLOT_B_START_ADDR + i * 4);
			entry = *(volatile uint32_t *)((uintptr_t)flash_meta_infor.slot_b_addr + i * 4);
			LOG_LEVEL("Flash_Vector[%02d] = 0x%08X\r\n", i, entry);
		}
	}
}

void Print_SRAM_VectorTable(void)
{
	// LOG_LEVEL("SRAM Interrupt Vector Table (from 0x00000000) ===\r\n");
	for (int i = 0; i < 48; i++)
	{
		// uint32_t entry = *(volatile uint32_t *)(0x00000000 + i * 4);
		uint32_t entry = *(volatile uint32_t *)((uintptr_t)0x00000000 + i * 4);
		LOG_LEVEL("SRAM_Vector[%02d] = 0x%08X\r\n", i, entry);
	}
}

void flash_print_mcu_meta_infor(void)
{
	LOG_LEVEL("Bootloader Address : 0x%08X\n", flash_meta_infor.loader_addr);
	LOG_LEVEL("Slot A Address     : 0x%08X\n", flash_meta_infor.slot_a_addr);
	LOG_LEVEL("Slot B Address     : 0x%08X\n", flash_meta_infor.slot_b_addr);
	LOG_LEVEL("Slot A/B Max Size  : 0x%08X\n", flash_get_app_max_size());
	LOG_LEVEL("Slot A Size        : 0x%08X\n", flash_meta_infor.slot_a_size);
	LOG_LEVEL("Slot B Size        : 0x%08X\n", flash_meta_infor.slot_b_size);

	LOG_LEVEL("CRC (Slot A)       : 0x%08X\n", flash_meta_infor.slot_a_crc);
	LOG_LEVEL("CRC (Slot B)       : 0x%08X\n", flash_meta_infor.slot_b_crc);

	LOG_LEVEL("Appp Status Flags  : 0x%08X\n", flash_meta_infor.slot_stat_flags);
	LOG_LEVEL("Meters Data Flags  : 0x%08X\n", flash_meta_infor.mete_data_flags);
	LOG_LEVEL("Config Data Flags  : 0x%08X\n", flash_meta_infor.user_data_flags);
	LOG_LEVEL("Bank Actived Slot  : 0x%02X\n", flash_meta_infor.bank_slot_activated);

	LOG_LEVEL("Loader Bank Mode   : 0x%02X\n", flash_meta_infor.bank_slot_mode);
	LOG_LEVEL("Flash Meta Address : 0x%08X\n", FLASH_META_DATA_START_ADDRESS);
	// LOG_LEVEL("VECT_TAB_OFFSET   : 0x%08X\n", SCB->AIRCR);
}

flash_meta_infor_t *flash_get_meta_infor(void)
{
	return &flash_meta_infor;
}

void flash_vector_table_config(boot_mode_t boot_mode, uint8_t bank_slot, uint32_t slot_address, bool mapping_vector)
{
	flash_bank_config_mode_slot = bank_slot;
	flash_bank_config_mode_boot = boot_mode;
	flash_bank_config_rom_address = slot_address;

	if (!mapping_vector)
		return;
#ifdef TASK_MANAGER_STATE_MACHINE_MCU
	DISABLE_IRQ;
	switch (flash_bank_config_mode_slot)
	{
	case BANK_SLOT_A:
		for (uint8_t i = 0; i < 48; i++)
		{
			IRAM_Vector_Table[i] = *(volatile uint32_t *)(flash_bank_config_rom_address + i * 4);
		}

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
		break;

	case BANK_SLOT_B:
		for (uint8_t i = 0; i < 48; i++)
		{
			IRAM_Vector_Table[i] = *(volatile uint32_t *)(flash_bank_config_rom_address + i * 4);
		}

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
		break;

	case BANK_SLOT_LOADER:
		for (uint8_t i = 0; i < 48; i++)
		{
			IRAM_Vector_Table[i] = *(volatile uint32_t *)(FLASH_BOOTLOADER_START_ADDR + i * 4);
		}

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
		break;

	case BANK_SLOT_INVALID:
	default:
		break;
	}

	ENABLE_IRQ;
#endif
}

void flash_reconfig_bank_address(void)
{
	flash_meta_infor.bank_slot_activated = flash_bank_config_mode_slot;
	flash_meta_infor.bank_slot_mode = flash_bank_config_mode_boot;
	uint32_t total_size = flash_get_app_max_size();
	switch (flash_meta_infor.bank_slot_mode)
	{
	case BOOT_MODE_SINGLE_BANK_NONE:	  // No bootloader or second bank present
	case BOOT_MODE_SINGLE_BANK_NO_LOADER: // Single bank without a bootloader
		flash_meta_infor.loader_addr = FLASH_BOOTLOADER_START_ADDR;
		flash_meta_infor.slot_a_addr = FLASH_BASE_START_ADDR;
		flash_meta_infor.slot_b_addr = 0;

		if (flash_meta_infor.slot_a_size == 0 || flash_meta_infor.slot_a_size > total_size)
			flash_meta_infor.slot_a_size = total_size;

		flash_meta_infor.slot_b_size = 0;
		break;

	case BOOT_MODE_SINGLE_BANK_WITH_LOADER: // Single bank with bootloader present
		flash_meta_infor.loader_addr = FLASH_BOOTLOADER_START_ADDR;

		flash_meta_infor.slot_a_addr = FLASH_BOOTLOADER_END_ADDR + FLASH_BLOCK_SIZE;
		flash_meta_infor.slot_b_addr = 0;

		if ((flash_meta_infor.slot_a_size == 0) || (flash_meta_infor.slot_a_size > total_size))
			flash_meta_infor.slot_a_size = total_size;

		flash_meta_infor.slot_b_size = 0;

		break;

	case BOOT_MODE_DUAL_BANK_NO_LOADER: // Two banks, but no dedicated bootloader
		flash_meta_infor.loader_addr = FLASH_BOOTLOADER_START_ADDR;

		if (flash_meta_infor.slot_a_size == 0 || flash_meta_infor.slot_a_size > total_size)
			flash_meta_infor.slot_a_size = total_size;

		if (flash_meta_infor.slot_b_size == 0 || flash_meta_infor.slot_b_size > total_size)
			flash_meta_infor.slot_b_size = total_size;

		flash_meta_infor.slot_a_addr = FLASH_BASE_START_ADDR;
		flash_meta_infor.slot_b_addr = flash_meta_infor.slot_a_size + FLASH_BLOCK_SIZE;
		break;

	case BOOT_MODE_DUAL_BANK_WITH_LOADER: // Two banks and a bootloader present
		break;

	default:
		flash_meta_infor.loader_addr = FLASH_BOOTLOADER_START_ADDR;
		if (flash_meta_infor.slot_a_size == 0 || flash_meta_infor.slot_a_size > total_size)
			flash_meta_infor.slot_a_size = total_size;
		break;
	}

	// LOG_LEVEL("bank_slot_mode=%02d slot_a_size=%08X total_size=%08X\r\n", flash_meta_infor.bank_slot_mode, flash_meta_infor.slot_a_size, total_size);
	if ((flash_meta_infor.slot_stat_flags & 0xF0000000) == 0xF0000000)
		flash_meta_infor.slot_stat_flags = 0;
}

void flash_load_sync_data_infor(void)
{
	//	  uint32_t bootloader_addr;       // Bootloader entry address

	//    uint32_t slot_a_addr;           // Flash address of Application Slot A
	//    uint32_t slot_b_addr;           // Flash address of Application Slot B

	//    uint32_t app_crc_slot_a;        // CRC32 checksum for application in Slot A
	//    uint32_t app_crc_slot_b;        // CRC32 checksum for application in Slot B

	//    uint32_t slot_stat_flags;       // Flags to indicate app update or state conditions
	//    uint32_t mete_data_flags;      // Flags to indicate meter data status
	//    uint32_t user_data_flags;     // Flags to indicate other config data status

	//    uint8_t  active_slot;           // Currently active slot: 0 for A, 1 for B
	//    uint8_t  last_boot_ok;          // Last boot result: 0 = failed, 1 = successful
	//    uint8_t  reserved[2];           // Reserved for alignment and future use
	LOG_NONE("\r\n");
	uint32_t calculated_crc = 0;
	flash_read_all_infor();
	flash_reconfig_bank_address();

	if (flash_bank_config_mode_slot == BANK_SLOT_A)
	{
		calculated_crc = flash_meta_infor.slot_a_crc;
		// flash_meta_infor.slot_a_version = build_version_code();

		// if (!(flash_meta_infor.slot_stat_flags & APP_FLAG_VALID_A) && !IS_SLOT_A_NEED_UPGRADE(flash_meta_infor.slot_stat_flags)) //
		if (flash_is_first_boot(BANK_SLOT_A))
		{
			LOG_LEVEL("First-Stage bank slot a\r\n");
			LOG_LEVEL("Starting data synchronization...\r\n");
			// flash_meta_infor.slot_a_size = MAIN_APP_SIZE;
			if (flash_meta_infor.slot_a_size <= flash_get_app_max_size())
			{
				DISABLE_IRQ;
				calculated_crc = calculate_crc_32((uint8_t *)(uintptr_t)flash_meta_infor.slot_a_addr, flash_meta_infor.slot_a_size);
				ENABLE_IRQ;
			}
			flash_meta_infor.slot_a_crc = calculated_crc;
			flash_meta_infor.slot_stat_flags |= APP_FLAG_VALID_A;
		}
	}
	else if (flash_bank_config_mode_slot == BANK_SLOT_B)
	{
		// flash_meta_infor.slot_b_version = build_version_code();
		// if (!(flash_meta_infor.slot_stat_flags & APP_FLAG_VALID_B) && !IS_SLOT_B_NEED_UPGRADE(flash_meta_infor.slot_stat_flags)) //
		if (flash_is_first_boot(BANK_SLOT_B))
		{
			LOG_LEVEL("First-Stage bank slot b\r\n");
			LOG_LEVEL("Starting data synchronization...\r\n");

			// flash_meta_infor.slot_b_size = MAIN_APP_SIZE;
			if (flash_meta_infor.slot_b_size <= flash_get_app_max_size())
			{
				DISABLE_IRQ;
				calculated_crc = calculate_crc_32((uint8_t *)(uintptr_t)flash_meta_infor.slot_b_addr, flash_meta_infor.slot_b_size);
				DISABLE_IRQ;
			}

			flash_meta_infor.slot_b_crc = calculated_crc;
			flash_meta_infor.slot_stat_flags |= APP_FLAG_VALID_B;
		}
	}
	else if (flash_bank_config_mode_slot == BANK_SLOT_LOADER)
	{
		if (flash_meta_infor.bank_slot_mode == BOOT_MODE_SINGLE_BANK_WITH_LOADER)
		{
			// if ((!(flash_meta_infor.slot_stat_flags & APP_FLAG_VALID_A) && !IS_SLOT_A_NEED_UPGRADE(flash_meta_infor.slot_stat_flags)) ||
			//(flash_meta_infor.slot_a_size == flash_get_app_max_size()))
			if (flash_is_first_boot(BANK_SLOT_A))
			{
				LOG_LEVEL("First-Stage Boot Detected slot a\r\n");
				LOG_LEVEL("Starting data synchronization...\r\n");
				// flash_meta_infor.slot_a_size = flash_get_app_max_size() - FLASH_BLOCK_SIZE;
				// flash_meta_infor.slot_a_size = MAIN_APP_SIZE;
				if (flash_meta_infor.slot_a_size <= flash_get_app_max_size())
				{
					DISABLE_IRQ;
					calculated_crc = calculate_crc_32((uint8_t *)(uintptr_t)flash_meta_infor.slot_a_addr, flash_meta_infor.slot_a_size);
					ENABLE_IRQ;
				}
				flash_meta_infor.slot_a_crc = calculated_crc;
				flash_meta_infor.slot_stat_flags |= APP_FLAG_VALID_A;
				flash_writ_all_infor();
			}
		}
	}

#ifdef TASK_MANAGER_STATE_MACHINE_MCU
	///	Print_VectorTable();
	/// Print_Flash_VectorTable();
	/// Print_SRAM_VectorTable();
	LOG_LEVEL("__get_MSP          : 0x%08X\r\n", __get_MSP());
	LOG_LEVEL("Reset_Handler      : 0x%08X\r\n", ((uint32_t *)0x00000000)[1]);
	LOG_LEVEL("SysTick_Handler    : 0x%08X\r\n", ((uint32_t *)0x00000000)[15]);
#endif
}

/**
 * @brief Main bootloader logic: verify and jump to the valid application
 */
void flash_loader_active_user_app(uint8_t bank_slot, const char *date_str, const char *time_str)
{
	flash_bank_config_mode_slot = bank_slot;
	flash_meta_infor.bank_slot_activated = flash_bank_config_mode_slot;
	// LOG_LEVEL("bank_slot_activated=%02x flash_bank_config_mode_slot=%02x\r\n",flash_meta_infor.bank_slot_activated,flash_bank_config_mode_slot);
	flash_print_logo();
	flash_load_sync_data_infor(); // Read application metadata from EEPROM
	flash_init_version(date_str, time_str);
	flash_print_mcu_meta_infor();

	uint32_t active_app_addr = 0;
	uint32_t expected_crc = 0;
	uint32_t slot_length = 0;
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	// START BEGIN IN BOOTLOADER MODE
	if (flash_get_current_bank() == BANK_SLOT_LOADER)
	{
		if (flash_check_enter_upgrade_mode())
		{
			LOG_LEVEL("The bank(%s) needs to be upgraded before proceeding.\r\n", flash_get_current_bank_name());
			goto ENTER_BOOTLOADER_MODE;
		}
		if (IS_SLOT_A_VALID(flash_meta_infor.slot_stat_flags))
		{
			active_app_addr = flash_meta_infor.slot_a_addr;
			expected_crc = flash_meta_infor.slot_a_crc;
			slot_length = flash_meta_infor.slot_a_size;
			LOG_LEVEL("Bootloader start to active slot A...\r\n");
			if (flash_check_vector_table(BANK_SLOT_A, active_app_addr))
				flash_goto_terget_bank(active_app_addr, expected_crc, slot_length);

			LOG_LEVEL("Bank slot A is invalid fallback to loader...\r\n");
		}

		if (IS_SLOT_B_VALID(flash_meta_infor.slot_stat_flags))
		{
			active_app_addr = flash_meta_infor.slot_b_addr;
			expected_crc = flash_meta_infor.slot_b_crc;
			slot_length = flash_meta_infor.slot_b_size;
			LOG_LEVEL("Bootloader start to active slot B...\r\n");

			if (flash_check_vector_table(BANK_SLOT_B, active_app_addr))
				flash_goto_terget_bank(active_app_addr, expected_crc, slot_length);

			LOG_LEVEL("Bank slot B is invalid fallback to loader...\r\n");
		}

		goto ENTER_BOOTLOADER_MODE;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	// A/B dual mode checking
	if (flash_meta_infor.bank_slot_mode == BOOT_MODE_DUAL_BANK_NO_LOADER)
	{
		if (compare_versions(flash_meta_infor.slot_a_version, flash_meta_infor.slot_b_version) >= 0)
		{ // AB MODE
			if (IS_SLOT_A_VALID(flash_meta_infor.slot_stat_flags))
			{
				active_app_addr = flash_meta_infor.slot_a_addr;
				expected_crc = flash_meta_infor.slot_a_crc;
				slot_length = flash_meta_infor.slot_a_size;
				LOG_LEVEL("Bank Active Slot   : A,flash_get_current_bank()=%d\r\n", flash_get_current_bank());
				if (flash_get_current_bank() == BANK_SLOT_A)
					return;
			}
			else if (IS_SLOT_B_VALID(flash_meta_infor.slot_stat_flags))
			{
				active_app_addr = flash_meta_infor.slot_b_addr;
				expected_crc = flash_meta_infor.slot_b_crc;
				slot_length = flash_meta_infor.slot_b_size;
				LOG_LEVEL("Slot A activation failed. fallback to Slot B initiated.\r\n");
				if (flash_get_current_bank() == BANK_SLOT_B)
				{
					return;
				}
			}
		}
		else
		{ // AB MODE
			if (IS_SLOT_B_VALID(flash_meta_infor.slot_stat_flags))
			{
				active_app_addr = flash_meta_infor.slot_b_addr;
				expected_crc = flash_meta_infor.slot_b_crc;
				slot_length = flash_meta_infor.slot_b_size;
				LOG_LEVEL("Bank Active Slot   : B,flash_get_current_bank()=%d\r\n", flash_get_current_bank());
				if (flash_get_current_bank() == BANK_SLOT_B)
					return;
			}
			else if (IS_SLOT_A_VALID(flash_meta_infor.slot_stat_flags))
			{
				active_app_addr = flash_meta_infor.slot_a_addr;
				expected_crc = flash_meta_infor.slot_a_crc;
				slot_length = flash_meta_infor.slot_a_size;
				LOG_LEVEL("Slot B activation failed. fallback to Slot A initiated.\r\n");
				if (flash_get_current_bank() == BANK_SLOT_A)
					return;
			}
		}
		if (flash_check_vector_table(flash_get_current_bank(), active_app_addr))
			flash_goto_terget_bank(active_app_addr, expected_crc, slot_length);
		return; // already in A/B mode,return directly keep A/B
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
// L/A,L/A/B mode
// ENTER_BOOTLOADER_MODE MODE
ENTER_BOOTLOADER_MODE:
	if (flash_get_current_bank() == BANK_SLOT_LOADER)
	{
		LOG_LEVEL("Entering bootloader upgrade mode(%s)...\r\n", flash_get_current_bank_name());
		switch (flash_meta_infor.bank_slot_mode)
		{
		case BOOT_MODE_SINGLE_BANK_WITH_LOADER:
			SET_FLAG(flash_meta_infor.slot_stat_flags, APP_FLAG_SLOT_A_NEED_UPGRADE);
			break;
		case BOOT_MODE_DUAL_BANK_WITH_LOADER:
			SET_FLAG(flash_meta_infor.slot_stat_flags, APP_FLAG_SLOT_A_NEED_UPGRADE);
			SET_FLAG(flash_meta_infor.slot_stat_flags, APP_FLAG_SLOT_B_NEED_UPGRADE);
			break;
		}
	}
	else
	{
		LOG_LEVEL("Entering running (%s)...\r\n", flash_get_current_bank_name());
	}
}

void flash_goto_terget_bank(uint32_t active_app_addr, uint32_t expected_crc, uint32_t slot_length)
{
	// Verify CRC of the active application
	if (flash_verify_bank_slot_crc(active_app_addr, slot_length, expected_crc))
	{
		LOG_LEVEL("Bank verified successfuly & jumping to application at 0x%08X...\r\n", active_app_addr);
		flash_JumpToApplication(active_app_addr); // Hand over control to application
		LOG_LEVEL("Bank jump failed! Can Not Jumping to 0x%08X...\r\n", active_app_addr);
	}
	else
	{
		LOG_LEVEL("Bank verified crc failed Can Not Jumping to application at 0x%08X...\r\n", active_app_addr);

		if (flash_get_current_bank() == BANK_SLOT_A)
		{
			return;
		}

		if (flash_get_current_bank() == BANK_SLOT_B)
		{
			return;
		}

		if (flash_get_current_bank() == BANK_SLOT_LOADER)
		{
			return;
		}
	}
}

bool flash_is_first_boot(uint8_t bank_slot)
{
	// if (!IS_SLOT_A_VALID(flash_meta_infor.slot_stat_flags) && !IS_SLOT_A_NEED_UPGRADE(flash_meta_infor.slot_stat_flags)) ||
	if (flash_meta_infor.mete_data_flags != FLASH_META_DATAS_VALID_FLAG)
		return true;

	switch (bank_slot)
	{
	case BANK_SLOT_LOADER:
		return false;

	case BANK_SLOT_A:
		if (flash_meta_infor.slot_a_crc == 0xFFFFFFFF || flash_meta_infor.slot_a_crc == 0)
		{
			return true;
		}
		else
		{
			return false;
		}

	case BANK_SLOT_B:
		if (flash_meta_infor.slot_b_crc == 0xFFFFFFFF || flash_meta_infor.slot_b_crc == 0)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	return false;
}

bool flash_is_meta_infor_valid(void)
{
	if (flash_meta_infor.bank_slot_mode <= BOOT_MODE_SINGLE_BANK_NONE || flash_meta_infor.bank_slot_mode >= BOOT_MODE_MAX)
		return false;

	if ((flash_get_current_bank() != BANK_SLOT_LOADER) && (flash_get_current_bank() != BANK_SLOT_A) && (flash_get_current_bank() != BANK_SLOT_B))
		return false;

	if (((flash_meta_infor.slot_a_addr == 0) || (flash_meta_infor.slot_b_addr == 0)) && (flash_meta_infor.bank_slot_mode >= BOOT_MODE_DUAL_BANK_NO_LOADER))
		return false;

	if ((flash_meta_infor.slot_a_addr == 0) && (flash_meta_infor.slot_b_addr == 0) && (flash_meta_infor.bank_slot_mode < BOOT_MODE_DUAL_BANK_NO_LOADER))
		return false;

	if ((flash_meta_infor.slot_a_size == 0) && (flash_meta_infor.slot_a_size == 0))
		return false;

	if ((flash_get_current_bank() == BANK_SLOT_LOADER) && (flash_meta_infor.loader_addr != FLASH_BOOTLOADER_START_ADDR))
		return false;

	// can detect the range of memory addresses for further checking
	return true;
}

bool flash_is_bank_address_valid(uint32_t b_address, uint32_t address)
{
	if (b_address == flash_meta_infor.slot_a_addr)
	{
		return (address >= flash_meta_infor.slot_a_addr) && (address < (flash_meta_infor.slot_a_addr + flash_get_app_max_size()));
	}
	else if (b_address == flash_meta_infor.slot_b_addr)
	{
		return (address >= flash_meta_infor.slot_b_addr) && (address < (flash_meta_infor.slot_b_addr + flash_get_app_max_size()));
	}
	else if (b_address == flash_meta_infor.loader_addr)
	{
		return (address >= flash_meta_infor.loader_addr) && (address < FLASH_BOOTLOADER_END_ADDR);
	}
	else
	{
		return false;
	}
}

bool flash_is_allow_update_bank(uint8_t bank_slot)
{
	switch (bank_slot)
	{
	case BANK_SLOT_LOADER:
		return false;
	case BANK_SLOT_A:
		return true;
	case BANK_SLOT_B:
		return ((flash_meta_infor.bank_slot_mode == BOOT_MODE_DUAL_BANK_NO_LOADER) || (flash_meta_infor.bank_slot_mode == BOOT_MODE_DUAL_BANK_WITH_LOADER));
	default:
		return false;
	}
}

bool flash_is_allow_update_address(uint32_t address)
{
	if (address >= FLASH_BOOTLOADER_START_ADDR && address <= FLASH_BOOTLOADER_END_ADDR)
		return false;
	else
		return true;
}

uint32_t flash_get_current_bank(void)
{
	return flash_meta_infor.bank_slot_activated;
}

uint32_t flash_get_bank_slot_mode(void)
{
	return flash_meta_infor.bank_slot_mode;
}

const char *flash_get_current_bank_name(void)
{
	return flash_get_bank_name(flash_get_current_bank());
}

const char *flash_get_bank_name(uint8_t bank)
{
	switch (bank)
	{
	case BANK_SLOT_LOADER:
		return "Loader";
	case BANK_SLOT_A:
		return "A";
	case BANK_SLOT_B:
		return "B";
	default:
		return "Unknown";
	}
}

uint32_t flash_get_bank_address(uint8_t bank_type)
{
	switch (bank_type)
	{
	case BANK_SLOT_LOADER:
		return FLASH_BOOTLOADER_START_ADDR;
	case BANK_SLOT_A:
		return flash_meta_infor.slot_a_addr;
	case BANK_SLOT_B:
		return flash_meta_infor.slot_b_addr;
	default:
		return 0;
	}
}

uint32_t flash_get_bank_offset_address(uint8_t bank_slot)
{
	switch (bank_slot)
	{
	case BANK_SLOT_LOADER:
		return FLASH_BOOTLOADER_START_ADDR & FLASH_BANK_UNMASK;
	case BANK_SLOT_A:
		return flash_meta_infor.slot_a_addr & FLASH_BANK_UNMASK;
	case BANK_SLOT_B:
		return flash_meta_infor.slot_b_addr & FLASH_BANK_UNMASK;
	default:
		return 0;
	}
}

uint32_t flash_get_app_max_size(void)
{
	switch (flash_meta_infor.bank_slot_mode)
	{
	case BOOT_MODE_SINGLE_BANK_NONE:	  // No bootloader or second bank present
	case BOOT_MODE_SINGLE_BANK_NO_LOADER: // Single bank without a bootloader
		return (FLASH_TOTAL_BLOCK - 2) * FLASH_BLOCK_SIZE;

	case BOOT_MODE_SINGLE_BANK_WITH_LOADER: // Single bank with bootloader present
		return (FLASH_TOTAL_BLOCK - FLASH_BOOTLOADER_BLOCK_COUNT - FLASH_DATA_BLOCK - 1) * FLASH_BLOCK_SIZE;

	case BOOT_MODE_DUAL_BANK_NO_LOADER: // Two banks, but no dedicated bootloader
		return ((FLASH_TOTAL_BLOCK - FLASH_DATA_BLOCK - 1) / 2) * FLASH_BLOCK_SIZE;

	case BOOT_MODE_DUAL_BANK_WITH_LOADER: // Two banks and a bootloader present
		return ((FLASH_TOTAL_BLOCK - FLASH_BOOTLOADER_BLOCK_COUNT - FLASH_DATA_BLOCK - 2) / 2) * FLASH_BLOCK_SIZE;
	}
	return 0;
}
/**
 * @brief Verify application CRC integrity
 *
 * @param app_addr      Flash start address of the application
 * @param expected_crc  CRC32 value stored in metadata
 * @return true if CRC is valid, false otherwise
 */
bool flash_verify_bank_slot_crc(uint32_t slot_addr, uint32_t slot_size, uint32_t expected_crc)
{
	/// uint8_t *app_data = (uint8_t *)app_addr;

	if (slot_addr == 0 || slot_size == 0 || expected_crc == 0)
	{
		LOG_LEVEL("Bank crc verify at 0x%08X size:0x%08X expected crc:0x%08X\r\n", slot_addr, slot_size, expected_crc);
		return false;
	}

	if (slot_size > flash_get_app_max_size())
	{
		LOG_LEVEL("CRC check failed because the bank size is incorrect\r\n", slot_size);
		return false;
	}

	LOG_LEVEL("Bank crc verify at 0x%08X size:0x%08X expected crc:0x%08X\r\n", slot_addr, slot_size, expected_crc);

	DISABLE_IRQ;
	uint32_t calculated_crc = calculate_crc_32((uint8_t *)(uintptr_t)slot_addr, slot_size);
	ENABLE_IRQ;

	LOG_LEVEL("Bank crc verify at 0x%08X size:0x%08X calculat crc:0x%08X\r\n", slot_addr, slot_size, calculated_crc);

	return (calculated_crc == expected_crc);
}

/**
 * @brief 检查应用程序向量表是否合法，决定 Bootloader 是否可以跳转
 * @param vector_table 起始地址（例如 0x0800A400）
 * @return true: 向量表合法，可跳转; false: 不合法，禁止跳转
 */
bool flash_check_vector_table(uint8_t bank_slot, uint32_t vector_address)
{
	if ((vector_address == 0) || (vector_address > (FLASH_BASE_END_ADDR - 2 * FLASH_BLOCK_SIZE)))
		return false;

	uint32_t *vector_table = (uint32_t *)(uintptr_t)vector_address;
	uint32_t sp_initial = vector_table[0];
	uint32_t reset_handler = vector_table[1];
	uint32_t systick_handler = vector_table[15];

	uint32_t start_address = 0;
	uint32_t end_address = 0;

	switch (bank_slot)
	{
	case BANK_SLOT_LOADER:
		start_address = FLASH_BOOTLOADER_START_ADDR;
		end_address = FLASH_BOOTLOADER_END_ADDR;
		break;
	case BANK_SLOT_A:
		start_address = flash_meta_infor.slot_a_addr;
		end_address = flash_meta_infor.slot_a_addr + flash_meta_infor.slot_a_size;
		break;
	case BANK_SLOT_B:
		start_address = flash_meta_infor.slot_b_addr;
		end_address = flash_meta_infor.slot_b_addr + flash_meta_infor.slot_b_size;
		break;
	}

	// 检查初始栈指针是否在 RAM 区间
	if (sp_initial < 0x20000000 || sp_initial > 0x20040000)
		return false;

	// 检查 Reset Handler 是否在 Flash/ROM 区间
	if (reset_handler < start_address || reset_handler > end_address)
		return false;

	if (systick_handler < start_address || systick_handler > end_address)
		return false;

	// 可选：检查前几个 IRQ 向量是否在 Flash 范围
	for (int i = 2; i < 16; i++)
	{
		uint32_t irq = vector_table[i];
		if (irq != 0 && (irq < start_address || irq > end_address))
			return false;
	}
	LOG_LEVEL("check vector table successfuly.\r\n");
	return true; // 向量表合法
}

bool flash_check_enter_upgrade_mode(void)
{
	if (IS_SLOT_A_NEED_UPGRADE(flash_meta_infor.slot_stat_flags))
		return true;
	else if (IS_SLOT_B_NEED_UPGRADE(flash_meta_infor.slot_stat_flags))
		return true;
	else
		return false;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  Safely jump from bootloader to user application at given address.
 *         For Cortex-M0 (no VTOR), assumes the application will remap RAM at 0x00000000
 *         and copy its own vector table before enabling interrupts.
 *
 * @param  app_address: Start address of the user application (where its vector table is located).
 *         Typically something like 0x08010000.
 */
#ifdef TASK_MANAGER_STATE_MACHINE_MCU
void flash_delay_ms(uint32_t ms)
{
	uint32_t i, count;
	count = (SystemCoreClock / 1000) * ms;
	for (i = 0; i < count; i++)
	{
		__NOP(); // __NOP()
	}
}
void flash_JumpToApplication(uint32_t app_address)
{
	typedef void (*pFunction)(void); // Function pointer type for Reset_Handler
	pFunction jump_to_app;

	// Read application's initial MSP and Reset_Handler address
	uint32_t app_msp = *(volatile uint32_t *)(app_address + 0x00);
	// Read Reset_Handler address from application vector table
	uint32_t app_reset = *(volatile uint32_t *)(app_address + 0x04);

	// Validate MSP: it must point to valid SRAM (0x20000000 ~ 0x2001FFFF typically)
	if ((app_msp & 0x2FFE0000) != 0x20000000)
	{
		LOG_LEVEL("Invalid Application MSP: 0x%08X,aborting jump.\r\n", app_msp);
		return;
	}

	LOG_LEVEL("Jumping to 0x%08X: MSP=0x%08X,Reset_Handler=0x%08X\r\n", app_address, app_msp, app_reset);
	flash_delay_ms(10);
	// Disable global interrupts
	DISABLE_IRQ;

	// Stop SysTick to avoid unwanted interrupts
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	// Disable all NVIC interrupts and clear pending ones (M0 has up to 32 IRQs)
	for (uint32_t i = 0; i < 1; ++i)
	{
		NVIC->ICER[i] = 0xFFFFFFFF; // Disable IRQs
		NVIC->ICPR[i] = 0xFFFFFFFF; // Clear pending IRQs
	}

	// Ensure all memory and peripheral accesses complete before jump
	__DSB();
	__ISB();

	// Set Main Stack Pointer to application's initial MSP
	__set_MSP(app_msp);

	// Cast application's Reset_Handler address to function pointer and call
	jump_to_app = (pFunction)app_reset;

	jump_to_app(); // Jump to application (this should never return)

	// Execution should never return here. If it does, halt safely.
	while (1)
		;
}
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t flash_erase_user_app_bank(uint8_t bank_slot)
{
	uint32_t ret = 0;
	uint16_t pages_count = 0;
	if (bank_slot == BANK_SLOT_LOADER)
	{
		LOG_LEVEL("Erase not permitted at address 0x%08X (size 0x%08X)\r\n", flash_meta_infor.loader_addr, FLASH_BOOTLOADER_SIZE);
		// DISABLE_IRQ;
		// ret = hal_flash_erase_page_(flash_meta_infor.slot_b_addr, flash_meta_infor.slot_a_size);
		// ENABLE_IRQ;
	}
	else if (bank_slot == BANK_SLOT_A)
	{
		LOG_LEVEL("Erase address: 0x%08X size:0x%08X\r\n", flash_meta_infor.slot_a_addr, flash_meta_infor.slot_a_size);
		if (flash_meta_infor.slot_a_addr >= FLASH_BOOTLOADER_START_ADDR && flash_meta_infor.slot_a_addr <= FLASH_BOOTLOADER_END_ADDR)
		{
			LOG_LEVEL("Erase not permitted at address 0x%08X (size 0x%08X)\r\n", flash_meta_infor.slot_a_addr, flash_meta_infor.slot_a_size);
			return 0;
		}
		pages_count = flash_meta_infor.slot_a_size / FLASH_BLOCK_SIZE + 1;
		DISABLE_IRQ;
		ret = hal_flash_erase_page_(flash_meta_infor.slot_a_addr, pages_count);
		ENABLE_IRQ;
	}
	else if (bank_slot == BANK_SLOT_B)
	{
		LOG_LEVEL("Erase address: 0x%08X size:0x%08X\r\n", flash_meta_infor.slot_b_addr, flash_meta_infor.slot_b_size);
		if (flash_meta_infor.slot_b_addr >= FLASH_BOOTLOADER_START_ADDR && flash_meta_infor.slot_b_addr <= FLASH_BOOTLOADER_END_ADDR)
		{
			LOG_LEVEL("Erase not permitted at address 0x%08X (size 0x%08X)\r\n", flash_meta_infor.slot_b_addr, flash_meta_infor.slot_b_size);
			return 0;
		}
		pages_count = flash_meta_infor.slot_b_size / FLASH_BLOCK_SIZE + 1;
		DISABLE_IRQ;
		ret = hal_flash_erase_page_(flash_meta_infor.slot_b_addr, pages_count);
		ENABLE_IRQ;
	}
	else
	{
		LOG_LEVEL("Erase not permitted at bank_slot: %d\r\n", bank_slot);
	}
	return ret;
}

uint32_t FlashErasePage(uint32_t addr, uint8_t page_count)
{
	uint8_t pages = 0;
	if (!flash_is_allow_update_address(addr))
	{
		LOG_LEVEL("Erase not permitted at address 0x%08X (page_count 0x%08X)\r\n", addr, page_count);
		return 0;
	}
	DISABLE_IRQ;
	pages = hal_flash_erase_page_(addr, page_count); // FlashErase(FLASH_META_DATA_START_ADDRESS, 2);
	ENABLE_IRQ;
	return pages;
}

//__attribute__((section(".ramfunc")))
uint32_t FlashWritBuffTo(uint32_t addr, uint8_t *buf, uint32_t length)
{
	uint32_t writed_bytes = 0;
	if (!flash_is_allow_update_address(addr))
	{
		LOG_LEVEL("Erase not permitted at address 0x%08X (length 0x%08X)\r\n", addr, length);
		return 0;
	}
	DISABLE_IRQ;
	writed_bytes = hal_flash_write_(addr, buf, length);
	ENABLE_IRQ;
	return writed_bytes;
}

uint32_t FlashReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length)
{
	uint32_t readed_bytes = 0;
	if (buf)
	{
		readed_bytes = hal_flash_read_(addr, buf, length);
	}
	return readed_bytes;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void E2ROMReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length)
{
	if (buf)
	{
		hal_eeprom_read_(addr, buf, length);
	}
}

void E2ROMWritBuffTo(uint32_t addr, uint8_t *buf, uint32_t length)
{
	if (buf)
	{
		hal_eeprom_write_(addr, buf, length);
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// #define FLASH_META_DATA_START_ADDRESS     (FLASH_BASE_END_ADDR - FLASH_BLOCK_SIZE)
// #define FLASH_SYSTEM_DATA_START_ADDRESS   (FLASH_META_DATA_START_ADDRESS + 128)
// #define FLASH_METER_DATA_START_ADDRESS    (FLASH_SYSTEM_DATA_START_ADDRESS + 128)
// #define FLASH_CARINFOR_DATA_START_ADDRESS (FLASH_METER_DATA_START_ADDRESS + 128)
void E2ROM_writ_meter_infor(void)
{
	// uint8_t pages = 0;
	E2ROMWritBuffTo(EEROM_SYSTEM_METER_ADDRESS, (uint8_t *)&system_meter_infor, sizeof(system_meter_infor_t));
	if (task_carinfo_get_meter_info())
	{
		E2ROMWritBuffTo(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
		LOG_LEVEL("Save eerom meta information meter_info()->trip_odo=%d... \r\n", task_carinfo_get_meter_info()->trip_odo);
	}
}

void E2ROM_read_meter_infor(void)
{
	// uint8_t pages = 0;
	E2ROMReadToBuff(EEROM_SYSTEM_METER_ADDRESS, (uint8_t *)&system_meter_infor, sizeof(system_meter_infor_t));
	if (task_carinfo_get_meter_info())
	{
		E2ROMReadToBuff(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
		LOG_LEVEL("task_carinfo_get_meter_info()->trip_odo=%d\r\n", task_carinfo_get_meter_info()->trip_odo);
	}
}

void E2ROM_writ_metas_infor(void)
{
	E2ROMWritBuffTo(EEROM_FLASH_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
}

void E2ROM_read_metas_infor(void)
{
	E2ROMReadToBuff(EEROM_FLASH_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void flash_writ_all_infor(void)
{
	uint8_t pages = 0;
	flash_meta_infor.mete_data_flags = FLASH_META_DATAS_VALID_FLAG;

#ifdef FLASH_USE_EEROM_FOR_DATA_SAVING
	if (task_carinfo_get_meter_info())
	{
		system_meter_infor.trip_odo = task_carinfo_get_meter_info()->trip_odo;
		system_meter_infor.speed_average = task_carinfo_get_meter_info()->speed_average;
	}
	E2ROMWritBuffTo(EEROM_FLASH_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
	E2ROMWritBuffTo(EEROM_SYSTEM_METER_ADDRESS, (uint8_t *)&system_meter_infor, sizeof(system_meter_infor_t));
	if (task_carinfo_get_meter_info())
	{
		// LOG_BUFF_LEVEL((uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
		// LOG_LEVEL("task_carinfo_get_meter_info().trip_odo:%d\r\n", task_carinfo_get_meter_info()->trip_odo);
		E2ROMWritBuffTo(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
	}
#else
	if (FLASH_USER_DATA_BLOCK <= 0)
	{
		LOG_LEVEL("No enougth space for user data pages:%d\r\n", FLASH_USER_DATA_BLOCK);
		return;
	}

	pages = FlashErasePage(FLASH_META_DATA_START_ADDRESS, FLASH_USER_DATA_BLOCK);
	LOG_LEVEL("flash_writ_all_infor need %d pages...\r\n", pages);
	if (task_carinfo_get_meter_info())
	{
		system_meter_infor.trip_odo = task_carinfo_get_meter_info()->trip_odo;
		system_meter_infor.speed_average = task_carinfo_get_meter_info()->speed_average;
	}
	pages = FlashWritBuffTo(FLASH_META_DATA_START_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
	LOG_LEVEL("Save flash meta information count=%d... \r\n", pages);
	pages = FlashWritBuffTo(FLASH_SYSTEM_DATA_START_ADDRESS, (uint8_t *)&system_meter_infor, sizeof(system_meter_infor_t));
	LOG_LEVEL("Save syste meta information count=%d... \r\n", pages);
	if (task_carinfo_get_meter_info())
	{
		// pages = FlashWriteBuffTo(FLASH_METER_DATA_START_ADDRESS, (uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
		// LOG_LEVEL("Save flash meta information task_carinfo_get_meter_info()=%d... \r\n", task_carinfo_get_meter_info()->trip_odo);
		E2ROM_writ_meter_infor();
		LOG_LEVEL("Save meter information count=%d... \r\n", pages);
	}

#endif
}

void flash_read_all_infor(void)
{
#ifdef FLASH_USE_EEROM_FOR_DATA_SAVING
	E2ROMReadToBuff(EEROM_FLASH_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
	E2ROMReadToBuff(EEROM_SYSTEM_METER_ADDRESS, (uint8_t *)&system_meter_infor, sizeof(system_meter_infor_t));
	if (task_carinfo_get_meter_info())
	{
		E2ROMReadToBuff(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
	}

#else
	FlashReadToBuff(FLASH_META_DATA_START_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
	FlashReadToBuff(FLASH_SYSTEM_DATA_START_ADDRESS, (uint8_t *)&system_meter_infor, sizeof(system_meter_infor_t));

	if (task_carinfo_get_meter_info())
	{
		// FlashReadToBuff(FLASH_METER_DATA_START_ADDRESS, (uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
		// task_carinfo_get_meter_info()->trip_odo = task_carinfo_get_meter_info()->trip_odo;
		E2ROM_read_meter_infor();
	}
#endif

	if (flash_meta_infor.mete_data_flags != FLASH_META_DATAS_VALID_FLAG)
	{
		if (task_carinfo_get_meter_info())
		{
			task_carinfo_get_meter_info()->trip_odo = 0;
			task_carinfo_get_meter_info()->speed_actual = 0;
			task_carinfo_get_meter_info()->speed_average = 0;
			task_carinfo_get_meter_info()->trip_odo = 0;
			task_carinfo_get_meter_info()->trip_distance = 0;
			task_carinfo_get_meter_info()->trip_time = 0;
		}

		system_meter_infor.trip_odo = 0;
		system_meter_infor.speed_actual = 0;
		system_meter_infor.speed_average = 0;
	}

	if (task_carinfo_get_meter_info())
	{
		if (task_carinfo_get_meter_info()->trip_odo >= (UINT32_MAX - 1000))
		{
			task_carinfo_get_meter_info()->trip_odo = 0;
		}

		if (system_meter_infor.trip_odo >= (UINT32_MAX - 1000))
		{
			system_meter_infor.trip_odo = 0;
		}
	}

	LOG_LEVEL("Load metas data[%03d]: ", sizeof(flash_meta_infor_t));
	LOG_BUFF((uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
	if (task_carinfo_get_meter_info() && (flash_meta_infor.mete_data_flags == FLASH_META_DATAS_VALID_FLAG))
	{
		LOG_LEVEL("Load meter data[%03d]: ", sizeof(carinfo_meter_t));
		LOG_BUFF((uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
	}
	else
	{
		LOG_LEVEL("Load meter data[%03d]: ", sizeof(carinfo_meter_t));
	}

	LOG_NONE("\r\n");
	// LOG_LEVEL("flash meta information task_carinfo_get_meter_info().trip_odo=%d... \r\n", task_carinfo_get_meter_info()->trip_odo);
}

// void flash_set_app_meta_
void flash_save_carinfor_meter(void)
{
	flash_writ_all_infor();
}

bool flash_decode_active_version(uint8_t bank_slot, char *out_str, size_t max_len, const char *date_str, const char *time_str)
{
	uint32_t version = 0;
	uint16_t y1;
	uint8_t m1, d1, h1, min1, code1;

	// #ifdef FLASH_BANK_CONFIG_MODE_SLOT
	if (bank_slot == BANK_SLOT_A)
		version = flash_meta_infor.slot_a_version;
	else if (bank_slot == BANK_SLOT_B)
		version = flash_meta_infor.slot_b_version;
	else if (bank_slot == BANK_SLOT_LOADER)
		version = flash_meta_infor.loader_version;
	else
		// #endif
		version = build_version_code(date_str, time_str);

	decode_datetime_version(version, &y1, &m1, &d1, &h1, &min1, &code1);
	snprintf(out_str, max_len, "%04u%02u%02u%02u%02u_%03u", y1, m1, d1, h1, min1, code1);
	return true;
}
