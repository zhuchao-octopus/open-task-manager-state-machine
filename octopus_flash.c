/*******************************************************************************
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
 ******************************************************************************/

#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "octopus_flash.h"
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
#include "octopus_carinfor.h"
#endif

void Print_VectorTable(void);
void Print_Flash_VectorTable(void);
void Print_SRAM_VectorTable(void);
void flash_print_mcu_meta_infor(void);
/////////////////////////////////////////////////////////////////////////////////////
flash_meta_infor_t flash_meta_infor = {0};

#ifdef FLASH_MAPPING_VECT_TABLE_TO_SRAM
#if (defined(__CC_ARM))
__IO uint32_t Vector_Table[48] __attribute__((at(0x20000000)));
#elif defined(__GNUC__)
__IO uint32_t Vector_Table[48] __attribute__((section(".SRAM_VECTOR_TABLE")));
#elif defined(__TASKING__)
__IO uint32_t Vector_Table[48] __at(0x20000000);
#endif
#endif
/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
flash_meta_infor_t *flash_get_meta_infor(void)
{
	return &flash_meta_infor;
}

#ifdef FLASH_MAPPING_VECT_TABLE_TO_SRAM
void flash_vector_table_config(uint8_t active_slot)
{
	if (active_slot == BANK_SLOT_AUTO)
	{
		if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
		{
			// LOG_LEVEL("BANK_SLOT_A Vector Table (from %08x)\r\n",MAIN_APP_SLOT_A_START_ADDR);
			for (uint8_t i = 0; i < 48; i++)
			{
				Vector_Table[i] = *(volatile uint32_t *)(MAIN_APP_SLOT_A_START_ADDR + i * 4);
			}
		}
		else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_B)
		{
			// LOG_LEVEL("MAIN_APP_SLOT_B_START_ADDR Vector Table (from %08x)\r\n",MAIN_APP_SLOT_B_START_ADDR);
			for (uint8_t i = 0; i < 48; i++)
			{
				Vector_Table[i] = *(volatile uint32_t *)(MAIN_APP_SLOT_B_START_ADDR + i * 4);
			}
		}
		else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_LOADER)
		{
			for (uint8_t i = 0; i < 48; i++)
			{
				Vector_Table[i] = *(volatile uint32_t *)(BOOTLOADER_START_ADDR + i * 4);
			}
		}
	}
	else
	{
		if (active_slot == BANK_SLOT_A)
		{
			// LOG_LEVEL("BANK_SLOT_A Vector Table (from %08x)\r\n",MAIN_APP_SLOT_A_START_ADDR);
			for (uint8_t i = 0; i < 48; i++)
			{
				Vector_Table[i] = *(volatile uint32_t *)(MAIN_APP_SLOT_A_START_ADDR + i * 4);
			}
		}
		else if (active_slot == BANK_SLOT_B)
		{
			// LOG_LEVEL("MAIN_APP_SLOT_B_START_ADDR Vector Table (from %08x)\r\n",MAIN_APP_SLOT_B_START_ADDR);
			for (uint8_t i = 0; i < 48; i++)
			{
				Vector_Table[i] = *(volatile uint32_t *)(MAIN_APP_SLOT_B_START_ADDR + i * 4);
			}
		}
	}

	/* Enable the SYSCFG peripheral clock*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* Remap SRAM at 0x00000000 */
	SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);
}

void Print_VectorTable(void)
{
	// LOG_LEVEL("Interrupt Vector Table\r\n");
	for (int i = 0; i < 48; i++)
	{
		LOG_LEVEL("Vector_Table[%02d] = 0x%08X\r\n", i, Vector_Table[i]);
	}
}
#endif

void Print_Flash_VectorTable(void)
{
	uint32_t entry = 0;
	if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
	{
		// LOG_LEVEL("Flash Vector Table (from %08x)\r\n",MAIN_APP_SLOT_A_START_ADDR);
		for (uint8_t i = 0; i < 48; i++)
		{
			// entry = *(volatile uint32_t *)(MAIN_APP_SLOT_A_START_ADDR + i * 4);
			entry = *(volatile uint32_t *)((uintptr_t)MAIN_APP_SLOT_A_START_ADDR + i * 4);
			LOG_LEVEL("Flash_Vector[%02d] = 0x%08X\r\n", i, entry);
		}
	}
	else
	{
		// LOG_LEVEL("Flash Vector Table (from %08x)\r\n",MAIN_APP_SLOT_B_START_ADDR);
		for (uint8_t i = 0; i < 48; i++)
		{
			// entry = *(volatile uint32_t *)(MAIN_APP_SLOT_B_START_ADDR + i * 4);
			entry = *(volatile uint32_t *)((uintptr_t)MAIN_APP_SLOT_B_START_ADDR + i * 4);
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
	LOG_LEVEL("Bootloader Address    : 0x%08X\n", flash_meta_infor.loader_addr);
	LOG_LEVEL("Slot A Address        : 0x%08X\n", flash_meta_infor.slot_a_addr);
	LOG_LEVEL("Slot B Address        : 0x%08X\n", flash_meta_infor.slot_b_addr);
	LOG_LEVEL("Slot A/B Max Size     : 0x%08X\n", MAIN_APP_SIZE);
	LOG_LEVEL("Slot A Size           : 0x%08X\n", flash_meta_infor.slot_a_size);
	LOG_LEVEL("Slot B Size           : 0x%08X\n", flash_meta_infor.slot_b_size);

	LOG_LEVEL("CRC (Slot A)          : 0x%08X\n", flash_meta_infor.slot_a_crc);
	LOG_LEVEL("CRC (Slot B)          : 0x%08X\n", flash_meta_infor.slot_b_crc);

	LOG_LEVEL("Appp Status Flags     : 0x%08X\n", flash_meta_infor.app_state_flags);
	LOG_LEVEL("Meters Data Flags     : 0x%08X\n", flash_meta_infor.meter_data_flags);
	LOG_LEVEL("Config Data Flags     : 0x%08X\n", flash_meta_infor.config_data_flags);
	LOG_LEVEL("Active Slot           : %u\n", flash_meta_infor.active_slot);
	// LOG_LEVEL("Last Boot Ok        : %u\n", flash_meta_infor.last_boot_ok);
	LOG_LEVEL("Loader Bank Mode      : 0x%02X\n", flash_meta_infor.bank_slot_mode);
	// LOG_LEVEL("User Data Address    : 0x%08X\n", EEROM_DATAS_ADDRESS);
	// LOG_LEVEL("VECT_TAB_OFFSET      : 0x%08X\n", SCB->AIRCR);
}
void flash_init(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	hal_flash_init(0);
#endif
}

void flash_load_sync_data_infor(void)
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//	  uint32_t bootloader_addr;       // Bootloader entry address

	//    uint32_t slot_a_addr;           // Flash address of Application Slot A
	//    uint32_t slot_b_addr;           // Flash address of Application Slot B

	//    uint32_t app_crc_slot_a;        // CRC32 checksum for application in Slot A
	//    uint32_t app_crc_slot_b;        // CRC32 checksum for application in Slot B

	//    uint32_t app_state_flags;       // Flags to indicate app update or state conditions
	//    uint32_t meter_data_flags;      // Flags to indicate meter data status
	//    uint32_t config_data_flags;     // Flags to indicate other config data status

	//    uint8_t  active_slot;           // Currently active slot: 0 for A, 1 for B
	//    uint8_t  last_boot_ok;          // Last boot result: 0 = failed, 1 = successful
	//    uint8_t  reserved[2];           // Reserved for alignment and future use
	LOG_NONE("\r\n");
#ifdef FLASH_USE_EEROM_FOR_DATA_SAVING
	uint32_t calculated_crc = 0;
	E2ROMReadToBuff(EEROM_APP_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
	flash_meta_infor.active_slot = FLASH_BANK_CONFIG_MODE_SLOT;
	flash_meta_infor.loader_addr = BOOTLOADER_START_ADDR;
	flash_meta_infor.slot_a_addr = MAIN_APP_SLOT_A_START_ADDR;
	flash_meta_infor.bank_slot_mode = BOOTLOADER_CONFIG_MODE_TYPE;
	flash_meta_infor.slot_b_addr = MAIN_APP_SLOT_B_START_ADDR;
	// flash_meta_infor.last_boot_ok = flash_meta_infor.active_slot;
	// if(flash_meta_infor.app_state_flags == 0xFFFFFFFF) flash_meta_infor.app_state_flags = 0;
	if ((flash_meta_infor.app_state_flags & 0xF0000000) == 0xF0000000)
		flash_meta_infor.app_state_flags = 0;

	if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
	{
		calculated_crc = flash_meta_infor.slot_a_crc;
		flash_meta_infor.slot_a_version = build_version_code();

		if (!(flash_meta_infor.app_state_flags & APP_FLAG_VALID_A) && !IS_SLOT_A_UPGRADED(flash_meta_infor.app_state_flags)) //
		{
			LOG_LEVEL("First-Stage Boot Detected.\r\n");
			LOG_LEVEL("Starting data synchronization...\r\n");
			flash_meta_infor.slot_a_size = MAIN_APP_SIZE;
			if (flash_meta_infor.slot_a_size <= MAIN_APP_SIZE)
			{
				DISABLE_IRQ;
				calculated_crc = calculate_crc_32((uint8_t *)flash_meta_infor.slot_a_addr, flash_meta_infor.slot_a_size);
				ENABLE_IRQ;
			}
			flash_meta_infor.slot_a_crc = calculated_crc;
			flash_meta_infor.app_state_flags |= APP_FLAG_VALID_A;
			// E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
		}
	}
	else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_B)
	{
		flash_meta_infor.slot_b_version = build_version_code();
		if (!(flash_meta_infor.app_state_flags & APP_FLAG_VALID_B) && !IS_SLOT_B_UPGRADED(flash_meta_infor.app_state_flags)) //
		{
			LOG_LEVEL("First-Stage Boot Detected.\r\n");
			LOG_LEVEL("Starting data synchronization...\r\n");

			flash_meta_infor.slot_b_size = MAIN_APP_SIZE;
			if (flash_meta_infor.slot_b_size <= MAIN_APP_SIZE)
			{
				DISABLE_IRQ;
				calculated_crc = calculate_crc_32((uint8_t *)flash_meta_infor.slot_b_addr, flash_meta_infor.slot_b_size);
				DISABLE_IRQ;
			}

			flash_meta_infor.slot_b_crc = calculated_crc;
			flash_meta_infor.app_state_flags |= APP_FLAG_VALID_B;
			// E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
		}
	}
	else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_LOADER)
	{
		flash_meta_infor.loader_version = build_version_code();
	}
	else
	{
	}

	E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
	flash_print_mcu_meta_infor();

	if (flash_meta_infor.meter_data_flags == EEROM_DATAS_VALID_FLAG)
	{
		LOG_LEVEL("load meter data[%02d] ", sizeof(carinfo_meter_t));
		E2ROMReadToBuff(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
		LOG_BUFF((uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
	}
#endif

#ifdef FLASH_MAPPING_VECT_TABLE_TO_SRAM
	///	Print_VectorTable();
	/// Print_Flash_VectorTable();
	/// Print_SRAM_VectorTable();
	LOG_LEVEL("__get_MSP       = 0x%08X\r\n", __get_MSP());
	LOG_LEVEL("Reset_Handler   = 0x%08X\r\n", ((uint32_t *)0x00000000)[1]);
	LOG_LEVEL("SysTick_Handler = 0x%08X\r\n", ((uint32_t *)0x00000000)[15]);
#endif
}

bool flash_is_meta_infor_valid(void)
{
	flash_print_mcu_meta_infor();
	if ((flash_meta_infor.active_slot == BANK_SLOT_INVALID) || (flash_meta_infor.bank_slot_mode == BOOT_MODE_SINGLE_BANK_NONE))
		return false;
	else if ((flash_meta_infor.slot_a_addr == 0) || (flash_meta_infor.slot_b_addr == 0))
		return false;
	else
		return true;
	// return flash_is_valid_bank_address(0, flash_meta_infor.active_slot);
}

bool flash_is_allow_update_bank(uint8_t bank_type)
{
	switch (bank_type)
	{
	case BANK_SLOT_LOADER:
		return true;
	case BANK_SLOT_A:
		return true;
	case BANK_SLOT_B:
		return ((flash_meta_infor.bank_slot_mode == BOOT_MODE_DUAL_BANK_NO_LOADER) || (flash_meta_infor.bank_slot_mode == BOOT_MODE_DUAL_BANK_WITH_LOADER));
	default:
		return false;
	}
}
bool flash_is_valid_bank_address(uint32_t b_address, uint32_t address)
{
	if ((b_address & FLASH_BANK_MASK) == MAIN_APP_SLOT_A_START_ADDR)
		return (address >= MAIN_APP_SLOT_A_START_ADDR) && (address < MAIN_APP_SLOT_B_START_ADDR);
	else if ((b_address & FLASH_BANK_MASK) == MAIN_APP_SLOT_B_START_ADDR)
		return (address >= MAIN_APP_SLOT_B_START_ADDR) && (address < (MAIN_APP_SLOT_B_START_ADDR + MAIN_APP_SIZE));
	else if (b_address == 0)
	{
		return (((address >= MAIN_APP_SLOT_A_START_ADDR) && (address < MAIN_APP_SLOT_B_START_ADDR)) ||
				((address >= MAIN_APP_SLOT_B_START_ADDR) && (address < (MAIN_APP_SLOT_B_START_ADDR + MAIN_APP_SIZE))));
	}
	else
		return false;
}

uint32_t flash_get_bank_address(uint8_t bank_type)
{
	switch (bank_type)
	{
	case BANK_SLOT_LOADER:
		return BOOTLOADER_START_ADDR;
	case BANK_SLOT_A:
		return MAIN_APP_SLOT_A_START_ADDR;
	case BANK_SLOT_B:
		return MAIN_APP_SLOT_B_START_ADDR;
	default:
		return 0;
	}
}

uint32_t flash_get_bank_offset_address(uint8_t bank_type)
{
	switch (bank_type)
	{
	case BANK_SLOT_LOADER:
		return BOOTLOADER_START_ADDR & FLASH_BANK_UNMASK;
	case BANK_SLOT_A:
		return MAIN_APP_SLOT_A_START_ADDR & FLASH_BANK_UNMASK;
	case BANK_SLOT_B:
		return MAIN_APP_SLOT_B_START_ADDR & FLASH_BANK_UNMASK;
	default:
		return 0;
	}
}

uint32_t flash_erase_user_app_arear(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	uint32_t ret = 0;
	if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
	{
		LOG_LEVEL("erase address: 0x%08X size:%08X\r\n", MAIN_APP_SLOT_B_START_ADDR, MAIN_APP_BLOCK_COUNT);
		DISABLE_IRQ;
		ret = hal_flash_erase_page_(MAIN_APP_SLOT_B_START_ADDR, MAIN_APP_BLOCK_COUNT);
		ENABLE_IRQ;
	}
	else
	{
		LOG_LEVEL("erase address: 0x%08X size:%08X\r\n", MAIN_APP_SLOT_A_START_ADDR, MAIN_APP_BLOCK_COUNT);
		DISABLE_IRQ;
		ret = hal_flash_erase_page_(MAIN_APP_SLOT_A_START_ADDR, MAIN_APP_BLOCK_COUNT);
		ENABLE_IRQ;
	}
	return ret;
#else
	return 0;
#endif
}

void flash_save_app_meter_infor(void)
{
#ifdef FLASH_USE_EEROM_FOR_DATA_SAVING
	// LOG_LEVEL("flash_save_app_meter\r\n");
	E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
	// E2ROMWriteBuffTo(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
#endif
}

// void flash_set_app_meta_
void flash_save_carinfor_meter(void)
{
#ifdef FLASH_USE_EEROM_FOR_DATA_SAVING
	LOG_BUFF_LEVEL((uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
	LOG_LEVEL("lt_carinfo_meter.odo:%d\r\n", lt_carinfo_meter.odo);
	if (lt_carinfo_meter.odo == 0)
	{
		return;
	}
	flash_meta_infor.meter_data_flags = EEROM_DATAS_VALID_FLAG;
	// E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&flash_meta_infor, sizeof(flash_meta_infor_t));
	E2ROMWriteBuffTo(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
#endif
}

#ifdef TASK_MANAGER_STATE_MACHINE_MCU
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
		LOG_LEVEL("Bank crc verify at: 0x%08X, size:0x%08X, expected crc:0x%08X \r\n", slot_addr, slot_size, expected_crc);
		return false;
	}
	if (slot_size > MAIN_APP_SIZE)
	{
		LOG_LEVEL("CRC check failed because the bank size is incorrect.\r\n", slot_size);
		return false;
	}

	DISABLE_IRQ;
	uint32_t calculated_crc = calculate_crc_32((uint8_t *)slot_addr, slot_size);
	ENABLE_IRQ;
	LOG_LEVEL("Bank crc verify at: 0x%08X, size:0x%08X, expected crc:0x%08X, calculate crc:%08x\r\n", slot_addr, slot_size, expected_crc, calculated_crc);

	return (calculated_crc == expected_crc);
}
/**
 * @brief  Safely jump from bootloader to user application at given address.
 *         For Cortex-M0 (no VTOR), assumes the application will remap RAM at 0x00000000
 *         and copy its own vector table before enabling interrupts.
 *
 * @param  app_address: Start address of the user application (where its vector table is located).
 *         Typically something like 0x08010000.
 */
void JumpToApplication(uint32_t app_address)
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

	LOG_LEVEL("Jumping To Application: MSP=0x%08X,Reset_Handler=0x%08X\r\n", app_msp, app_reset);

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

/**
 * @brief Main bootloader logic: verify and jump to the valid application
 */
void boot_loader_active_user_app(void)
{
	// Read application metadata from EEPROM
	flash_load_sync_data_infor();

	uint32_t active_app_addr = 0;
	uint32_t expected_crc = 0;
	uint32_t slot_length = 0;

	if (compare_versions(flash_meta_infor.slot_a_version, flash_meta_infor.slot_b_version) >= 0)
	{
		if (IS_SLOT_A_VALID(flash_meta_infor.app_state_flags))
		{
			active_app_addr = flash_meta_infor.slot_a_addr;
			expected_crc = flash_meta_infor.slot_a_crc;
			slot_length = flash_meta_infor.slot_a_size;
			LOG_LEVEL("Active slot: A\r\n");
			if (flash_meta_infor.active_slot == BANK_SLOT_A)
				return;
		}
		else if (IS_SLOT_B_VALID(flash_meta_infor.app_state_flags))
		{
			active_app_addr = flash_meta_infor.slot_b_addr;
			expected_crc = flash_meta_infor.slot_b_crc;
			slot_length = flash_meta_infor.slot_b_size;
			LOG_LEVEL("Active slot A is invalid! try to active slot B\r\n");
			if (flash_meta_infor.active_slot == BANK_SLOT_B)
				return;
		}
	}
	else
	{
		if (IS_SLOT_B_VALID(flash_meta_infor.app_state_flags))
		{
			active_app_addr = flash_meta_infor.slot_b_addr;
			expected_crc = flash_meta_infor.slot_b_crc;
			slot_length = flash_meta_infor.slot_b_size;
			LOG_LEVEL("Active slot: B\r\n");
			if (flash_meta_infor.active_slot == BANK_SLOT_B)
				return;
		}
		else if (IS_SLOT_A_VALID(flash_meta_infor.app_state_flags))
		{
			active_app_addr = flash_meta_infor.slot_a_addr;
			expected_crc = flash_meta_infor.slot_a_crc;
			slot_length = flash_meta_infor.slot_a_size;
			LOG_LEVEL("Active slot B is invalid! try to active slot A\r\n");
			if (flash_meta_infor.active_slot == BANK_SLOT_A)
				return;
		}
	}

	// Verify CRC of the active application
	if (flash_verify_bank_slot_crc(active_app_addr, slot_length, expected_crc))
	{
		LOG_LEVEL("Bank verified crc passed,Jumping to application at 0x%08X...\r\n", active_app_addr);
		JumpToApplication(active_app_addr); // Hand over control to application
	}
	else
	{
		if (flash_meta_infor.active_slot == BANK_SLOT_A)
			return;
		if (flash_meta_infor.active_slot == BANK_SLOT_B)
			return;

		if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_LOADER)
		{
			// Both slots failed integrity check, enter firmware update mode
			LOG_LEVEL("Active slot A/B failed! Both slots failed verification. Entering upgrade mode.\r\n");
			// EnterFirmwareUpgradeMode();  // Implement your IAP or OTA entry point
		}
	}
}
#endif
/**
 * @fn void printfFuncHex(const char *fun, int line, char *str, uint8_t *dat, int len)
 * @brief print input data in hex
 * @param fun: print function name
 * @param line: Print line number
 * @param str: print data in string
 * @param dat: print data in hex
 * @param len: length of data
 * @return NONE.
 */

// PrintfBuffHex(__func__, __LINE__, "READ After Write By Dma", tempStr, osal_strlen(str));
void PrintfBuffHex(const char *fun, int line, char *str, uint8_t *dat, int len)
{
	/// LOG_("%s(%d):%s:", fun, line, str);
	/// for (int ii = 0; ii < len; ii++)
	///{
	///	LOG_("%02x ", dat[ii]);
	/// }
	/// LOG_("\r\n");
}

//__attribute__((section(".ramfunc")))
uint32_t FlashWriteBuffTo(uint32_t addr, uint8_t *buf, uint32_t length)
{
	uint32_t writed_bytes = 0;
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	DISABLE_IRQ;
	writed_bytes = hal_flash_write_(addr, buf, length);
	ENABLE_IRQ;
#endif
	return writed_bytes;
}

void FlashReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	hal_flash_read_(addr, buf, length);
#endif
}

void E2ROMReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length)
{
#ifdef FLASH_USE_EEROM_FOR_DATA_SAVING
	hal_eeprom_read_(addr, buf, length);
#endif
}

void E2ROMWriteBuffTo(uint32_t addr, uint8_t *buf, uint32_t length)
{
#ifdef FLASH_USE_EEROM_FOR_DATA_SAVING
	hal_eeprom_write_(addr, buf, length);
#endif
}

void flash_decode_active_version(char *out_str, size_t max_len)
{
	uint32_t version = 0;
	uint16_t y1;
	uint8_t m1, d1, h1, min1, code1;

	if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
		version = flash_meta_infor.slot_a_version;
	else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_B)
		version = flash_meta_infor.slot_b_version;
	else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_LOADER)
		version = flash_meta_infor.loader_version;
	else
		version = build_version_code();

	decode_datetime_version(version, &y1, &m1, &d1, &h1, &min1, &code1);
	snprintf(out_str, max_len, "%04u%02u%02u%02u%02u_%03u", y1, m1, d1, h1, min1, code1);
}
