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

/////////////////////////////////////////////////////////////////////////////////////
//typedef void (*AppEntryPoint)(void);  // Function pointer type for application entry
//#define SCB_VTOR_ADDRESS (0xE000ED08) // SCB Vector Table Offset Register (VTOR)
//#define SCB_VTOR (*(volatile uint32_t *)SCB_VTOR_ADDRESS)
#define CRC32_POLYNOMIAL  (0x04C11DB7)  // Standard CRC32 polynomial
//#define MAIN_APP_MAX_SIZE (100 * 1024) // 100KB Main Application size

app_meta_data_t app_meta_data = {0};

#ifdef MAPPING_VECT_TABle_TO_SRAM
#if (defined ( __CC_ARM ))
__IO uint32_t Vector_Table[48] __attribute__((at(0x20000000)));
#elif defined   (  __GNUC__  )
__IO uint32_t Vector_Table[48] __attribute__((section(".SRAM_VECTOR_TABLE")));
#elif defined ( __TASKING__ )
__IO uint32_t Vector_Table[48] __at(0x20000000);
#endif
#endif
/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
#ifdef MAPPING_VECT_TABle_TO_SRAM
void flash_vector_table_config(uint8_t active_slot)
{
	if(active_slot == BANK_SLOT_AUTO)
	{
		if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
		{
				//LOG_LEVEL("BANK_SLOT_A Vector Table (from %08x)\r\n",MAIN_APP_SLOT_A_START_ADDR);
				for(uint8_t i = 0;i < 48; i++)
				{
				 Vector_Table[i] = *(volatile uint32_t *)(MAIN_APP_SLOT_A_START_ADDR + i * 4);
				}
		}
		else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_B)
		{
				//LOG_LEVEL("MAIN_APP_SLOT_B_START_ADDR Vector Table (from %08x)\r\n",MAIN_APP_SLOT_B_START_ADDR);
				for(uint8_t i =0;i < 48;i++)
				{
				 Vector_Table[i] = *(volatile uint32_t *)(MAIN_APP_SLOT_B_START_ADDR + i * 4);
				}
	  }
	 else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_LOADER)
	   {
				for(uint8_t i =0;i < 48;i++)
				{
				 Vector_Table[i] = *(volatile uint32_t *)(BOOTLOADER_START_ADDR + i * 4);
				}
		}
  }
	else
	{
	  if(active_slot == BANK_SLOT_A)
		{	
			//LOG_LEVEL("BANK_SLOT_A Vector Table (from %08x)\r\n",MAIN_APP_SLOT_A_START_ADDR);
			for(uint8_t i = 0;i < 48; i++)
			{
			 Vector_Table[i] = *(volatile uint32_t *)(MAIN_APP_SLOT_A_START_ADDR + i * 4);
			}
	  }
    else  if(active_slot == BANK_SLOT_B) 
		{
			//LOG_LEVEL("MAIN_APP_SLOT_B_START_ADDR Vector Table (from %08x)\r\n",MAIN_APP_SLOT_B_START_ADDR);
			for(uint8_t i =0;i < 48;i++)
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
    //LOG_LEVEL("Interrupt Vector Table\r\n");
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
		//LOG_LEVEL("Flash Vector Table (from %08x)\r\n",MAIN_APP_SLOT_A_START_ADDR);
		for(uint8_t i = 0;i < 48; i++)
		{
		 entry = *(volatile uint32_t *)(MAIN_APP_SLOT_A_START_ADDR + i * 4);
		 LOG_LEVEL("Flash_Vector[%02d] = 0x%08X\r\n", i, entry);
		}
	}
	else
	{
		//LOG_LEVEL("Flash Vector Table (from %08x)\r\n",MAIN_APP_SLOT_B_START_ADDR);
		for(uint8_t i =0;i< 48;i++)
		{
		 entry = *(volatile uint32_t *)(MAIN_APP_SLOT_B_START_ADDR + i * 4);
		 LOG_LEVEL("Flash_Vector[%02d] = 0x%08X\r\n", i, entry);
		}
	}
}

void Print_SRAM_VectorTable(void)
{
    //LOG_LEVEL("SRAM Interrupt Vector Table (from 0x00000000) ===\r\n");
    for (int i = 0; i < 48; i++)
    {
        uint32_t entry = *(volatile uint32_t *)(0x00000000 + i * 4);
        LOG_LEVEL("SRAM_Vector[%02d] = 0x%08X\r\n", i, entry);
    }
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
#ifdef USE_EEROM_FOR_DATA_SAVING
  uint32_t calculated_crc = 0;
	E2ROMReadToBuff(EEROM_APP_MATA_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));
	app_meta_data.active_slot = FLASH_BANK_CONFIG_MODE_SLOT;
	app_meta_data.loader_addr = BOOTLOADER_START_ADDR;
	app_meta_data.slot_a_addr = MAIN_APP_SLOT_A_START_ADDR;
	app_meta_data.bank_slot_mode = BOOTLOADER_CONFIG_MODE_TYPE;
	app_meta_data.slot_b_addr = MAIN_APP_SLOT_B_START_ADDR;	
	//app_meta_data.last_boot_ok = app_meta_data.active_slot;	
	//if(app_meta_data.app_state_flags == 0xFFFFFFFF) app_meta_data.app_state_flags = 0;
	if((app_meta_data.app_state_flags & 0xF0000000) == 0xF0000000) app_meta_data.app_state_flags = 0;
	
	if(FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
	{
		calculated_crc = app_meta_data.slot_a_crc;
		app_meta_data.slot_a_version = build_version_code();
		
		if (!(app_meta_data.app_state_flags & APP_FLAG_VALID_A) || !IS_SLOT_A_UPGRADED(app_meta_data.app_state_flags)) 
		{		
			LOG_LEVEL("First-Stage Boot Detected,Starting data synchronization...\r\n");
			app_meta_data.slot_a_size = MAIN_APP_SIZE;
		 	if(app_meta_data.slot_a_size <= MAIN_APP_SIZE)
			{
				DISABLE_IRQ;
				calculated_crc = flash_calculate_crc_32((uint8_t *)app_meta_data.slot_a_addr, app_meta_data.slot_a_size);
				ENABLE_IRQ;				
		  }			
			app_meta_data.slot_a_crc = calculated_crc;
			app_meta_data.app_state_flags |= APP_FLAG_VALID_A;
			//E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));		
		}
	}
	else if(FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_B)
	{
		app_meta_data.slot_b_version = build_version_code();
		if (!(app_meta_data.app_state_flags & APP_FLAG_VALID_B) || !IS_SLOT_B_UPGRADED(app_meta_data.app_state_flags)) 
		{
			LOG_LEVEL("First-Stage Boot Detected,Starting data synchronization...\r\n");
			app_meta_data.slot_b_size = MAIN_APP_SIZE;
			if(app_meta_data.slot_b_size <= MAIN_APP_SIZE)
			{
				DISABLE_IRQ;
				calculated_crc = flash_calculate_crc_32((uint8_t *)app_meta_data.slot_b_addr, app_meta_data.slot_b_size);  
				DISABLE_IRQ;
			}

			app_meta_data.slot_b_crc = calculated_crc;
			app_meta_data.app_state_flags |= APP_FLAG_VALID_B;
			//E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));	
		}	
	}
	else if(FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_LOADER){
		app_meta_data.loader_version = build_version_code();
	}
	else{}
	E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));	
	LOG_LEVEL("Bootloader Address    : 0x%08X\n", app_meta_data.loader_addr);
	LOG_LEVEL("Slot A Address        : 0x%08X\n", app_meta_data.slot_a_addr);
	LOG_LEVEL("Slot B Address        : 0x%08X\n", app_meta_data.slot_b_addr);
	LOG_LEVEL("Slot A/B Max Size     : 0x%08X\n", MAIN_APP_SIZE);
	LOG_LEVEL("Slot A Size           : 0x%08X\n", app_meta_data.slot_a_size);
	LOG_LEVEL("Slot B Size           : 0x%08X\n", app_meta_data.slot_b_size);
	
	LOG_LEVEL("CRC (Slot A)          : 0x%08X\n", app_meta_data.slot_a_crc);
	LOG_LEVEL("CRC (Slot B)          : 0x%08X\n", app_meta_data.slot_b_crc);
	
	LOG_LEVEL("Appp Status Flags     : 0x%08X\n", app_meta_data.app_state_flags);
	LOG_LEVEL("Meters Data Flags     : 0x%08X\n", app_meta_data.meter_data_flags);
	LOG_LEVEL("Config Data Flags     : 0x%08X\n", app_meta_data.config_data_flags);
	LOG_LEVEL("Active Slot           : %u\n", app_meta_data.active_slot);
	//LOG_LEVEL("Last Boot Ok        : %u\n", app_meta_data.last_boot_ok);
	LOG_LEVEL("Loader Bank Mode      : 0x%02X\n", app_meta_data.bank_slot_mode);
	//LOG_LEVEL("User Data Address    : 0x%08X\n", EEROM_DATAS_ADDRESS);
	//LOG_LEVEL("VECT_TAB_OFFSET      : 0x%08X\n", SCB->AIRCR);
#endif
	
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
	if (app_meta_data.meter_data_flags == EEROM_DATAS_VALID_FLAG)
	{
		LOG_LEVEL("load meter data[%02d] ", sizeof(carinfo_meter_t));
		E2ROMReadToBuff(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
		LOG_BUFF((uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
	}
#endif
	
///#ifdef MAPPING_VECT_TABle_TO_SRAM
///	Print_VectorTable();
///#endif
///Print_Flash_VectorTable();
///Print_SRAM_VectorTable();
	LOG_LEVEL("__get_MSP       = 0x%08X\r\n", __get_MSP());
	LOG_LEVEL("Reset_Handler   = 0x%08X\r\n", ((uint32_t*)0x00000000)[1]);
	LOG_LEVEL("SysTick_Handler = 0x%08X\r\n", ((uint32_t*)0x00000000)[15]);
}

uint32_t flash_erase_user_app_arear(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
		return hal_flash_erase_page_(MAIN_APP_SLOT_B_START_ADDR, MAIN_APP_BLOCK_COUNT);
	else
		return hal_flash_erase_page_(MAIN_APP_SLOT_A_START_ADDR, MAIN_APP_BLOCK_COUNT);
	//return 0;
	
#else
	return 0;
#endif
}

#define FLASH_COPY_BLOCK_SIZE 64  // Size of each block to copy from source to destination

/**
 * @brief Synchronize (copy) flash data from bank B to bank A.
 *
 * This function reads data from bank_b_address and writes it to bank_a_address.
 * It assumes both banks have the same size, and the destination region (bank A)
 * is already erased if required by the flash hardware.
 *
 * @param bank_a_address  Destination flash start address (e.g., active bank)
 * @param bank_b_address  Source flash start address (e.g., backup or update bank)
 */
void flash_synch_banck_slot(uint32_t destination_address, uint32_t source_address)
{
    uint8_t buffer[FLASH_COPY_BLOCK_SIZE];

    // Total size to copy (e.g., 20 KB)
    //const uint32_t flash_size = 20 * 1024;

    for (uint32_t offset = 0; offset < MAIN_APP_SIZE; offset += FLASH_COPY_BLOCK_SIZE)
    {
        // Copy data from source bank (bank B) into RAM buffer
        memcpy(buffer, (const void*)(source_address + offset), FLASH_COPY_BLOCK_SIZE);

        // Write buffer to destination bank (bank A)
        if (FlashWriteBuffTo(destination_address + offset, buffer, FLASH_COPY_BLOCK_SIZE) != 0)
        {
            // If write fails, log error and break loop
            printf("Flash write failed at offset 0x%08X\n", offset);
            break;
        }
    }

    LOG_LEVEL("Flash synchronization complete.\n");
}

void flash_save_app_meter_infor(void)
{
#ifdef USE_EEROM_FOR_DATA_SAVING
  //LOG_LEVEL("flash_save_app_meter\r\n");
	E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));
	//E2ROMWriteBuffTo(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
#endif
}

// void flash_set_app_meta_
void flash_save_carinfor_meter(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
	LOG_BUFF_LEVEL((uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
	LOG_LEVEL("lt_carinfo_meter.odo:%d\r\n", lt_carinfo_meter.odo);
	if (lt_carinfo_meter.odo == 0)
	{
		return;
	}
	app_meta_data.meter_data_flags = EEROM_DATAS_VALID_FLAG;
	//E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));
	E2ROMWriteBuffTo(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
#endif
}

/*******************************************************************************
 * CRC Calculation
 *******************************************************************************/
uint32_t flash_calculate_crc_32(uint8_t *data, uint32_t length)
{
	uint32_t crc = 0xFFFFFFFF;
	for (uint32_t i = 0; i < length; i++)
	{
		crc ^= (uint32_t)data[i] << 24;
		for (uint8_t j = 0; j < 8; j++)
		{
			if (crc & 0x80000000)
			{
				crc = (crc << 1) ^ CRC32_POLYNOMIAL;
			}
			else
			{
				crc <<= 1;
			}
		}
	}
	return crc ^ 0xFFFFFFFF;
}

/**
 * @brief Verify application CRC integrity
 *
 * @param app_addr      Flash start address of the application
 * @param expected_crc  CRC32 value stored in metadata
 * @return true if CRC is valid, false otherwise
 */
bool flash_verify_bank_slot_crc(uint32_t slot_addr,uint32_t slot_size, uint32_t expected_crc)
{
	///uint8_t *app_data = (uint8_t *)app_addr;
	
	if(slot_addr == 0 || slot_size == 0 || expected_crc == 0)
	{
		LOG_LEVEL("Bank crc verify at: 0x%08X, size:0x%08X, expected crc:0x%08X \r\n", slot_addr,slot_size,expected_crc);
		return false;
	}
	if(slot_size > MAIN_APP_SIZE)
	{
		LOG_LEVEL("CRC check failed because the bank size is incorrect.\r\n", slot_size);
		return false;
	}
	
	DISABLE_IRQ;
	uint32_t calculated_crc = flash_calculate_crc_32((uint8_t *)slot_addr, slot_size);
	ENABLE_IRQ;
	LOG_LEVEL("Bank crc verify at: 0x%08X, size:0x%08X, expected crc:0x%08X, calculate crc:%08x\r\n", slot_addr,slot_size,expected_crc,calculated_crc);

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
    typedef void (*pFunction)(void);  // Function pointer type for Reset_Handler
    pFunction jump_to_app;

    // Read application's initial MSP and Reset_Handler address
    uint32_t app_msp   = *(volatile uint32_t*)(app_address + 0x00);
 	// Read Reset_Handler address from application vector table
    uint32_t app_reset = *(volatile uint32_t*)(app_address + 0x04);

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
    SysTick->VAL  = 0;

    // Disable all NVIC interrupts and clear pending ones (M0 has up to 32 IRQs)
    for (uint32_t i = 0; i < 1; ++i)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;  // Disable IRQs
        NVIC->ICPR[i] = 0xFFFFFFFF;  // Clear pending IRQs
    }

    // Ensure all memory and peripheral accesses complete before jump
    __DSB();
    __ISB();

    // Set Main Stack Pointer to application's initial MSP
    __set_MSP(app_msp);

    // Cast application's Reset_Handler address to function pointer and call
    jump_to_app = (pFunction)app_reset;
		
    jump_to_app();  // Jump to application (this should never return)

    // Execution should never return here. If it does, halt safely.
    while (1);
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
	//uint32_t state_flags = 0;
	
	//state_flags = app_meta_data.app_state_flags;
	

	//Select the currently active slot (A or B)
	//if (app_meta_data.active_slot == BANK_SLOT_A)
	if(compare_versions(app_meta_data.slot_a_version,app_meta_data.slot_b_version) >= 0)
	{
		if (IS_SLOT_A_VALID(app_meta_data.app_state_flags))
		{
			active_app_addr = app_meta_data.slot_a_addr;
			expected_crc = app_meta_data.slot_a_crc;
			slot_length = app_meta_data.slot_a_size;
			LOG_LEVEL("Active slot: A\r\n");
			if(app_meta_data.active_slot == BANK_SLOT_A) return;
		}
		else if (IS_SLOT_B_VALID(app_meta_data.app_state_flags))
		{
			active_app_addr = app_meta_data.slot_b_addr;
			expected_crc = app_meta_data.slot_b_crc;
			slot_length = app_meta_data.slot_b_size;
			LOG_LEVEL("Active slot A is invalid! try to active slot B\r\n");
			if(app_meta_data.active_slot == BANK_SLOT_B) return;
		}
	}
	else// if (app_meta_data.active_slot == BANK_SLOT_B)
	{
		if (IS_SLOT_B_VALID(app_meta_data.app_state_flags))
		{
			active_app_addr = app_meta_data.slot_b_addr;
			expected_crc = app_meta_data.slot_b_crc;
			slot_length = app_meta_data.slot_b_size;
			LOG_LEVEL("Active slot: B\r\n");
			if(app_meta_data.active_slot == BANK_SLOT_B) return;
		}
		else if (IS_SLOT_A_VALID(app_meta_data.app_state_flags))
		{
			active_app_addr = app_meta_data.slot_a_addr;
			expected_crc = app_meta_data.slot_a_crc;
			slot_length = app_meta_data.slot_a_size;
			LOG_LEVEL("Active slot B is invalid! try to active slot A\r\n");
			if(app_meta_data.active_slot == BANK_SLOT_A) return;
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
		if(app_meta_data.active_slot == BANK_SLOT_A) return;
		if(app_meta_data.active_slot == BANK_SLOT_B) return;
		
		if(FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_LOADER)
		{
			// Both slots failed integrity check, enter firmware update mode
			LOG_LEVEL("Active slot A/B failed! Both slots failed verification. Entering upgrade mode.\r\n");
			// EnterFirmwareUpgradeMode();  // Implement your IAP or OTA entry point
		}
	}
}

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
	DISABLE_IRQ;
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	writed_bytes = hal_flash_write_(addr, buf, length);
#endif
	ENABLE_IRQ;
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
#ifdef USE_EEROM_FOR_DATA_SAVING	
	hal_eeprom_read_(addr, buf, length);
#endif
}

void E2ROMWriteBuffTo(uint32_t addr, uint8_t *buf, uint32_t length)
{
#ifdef USE_EEROM_FOR_DATA_SAVING	
	hal_eeprom_write_(addr, buf, length);
#endif
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Convert month string to numeric index (1-12)
uint8_t get_month_index(const char* month_str) {
    static const char* months[] = {
        "Jan","Feb","Mar","Apr","May","Jun",
        "Jul","Aug","Sep","Oct","Nov","Dec"
    };
    for (uint8_t i = 0; i < 12; i++) {
        if (month_str[0] == months[i][0] &&
            month_str[1] == months[i][1] &&
            month_str[2] == months[i][2]) {
            return i + 1;
        }
    }
    return 0; // Invalid month
}

// Parse build date (__DATE__) and time (__TIME__)
void parse_build_date_time(uint16_t* year, uint8_t* month, uint8_t* day,
                           uint8_t* hour, uint8_t* minute) {
    char month_str[4];
    int y, d, h, m, s;

    sscanf(__DATE__, "%3s %d %d", month_str, &d, &y);
    *year  = (uint16_t)y;
    *month = get_month_index(month_str);
    *day   = (uint8_t)d;

    sscanf(__TIME__, "%d:%d:%d", &h, &m, &s);
    *hour   = (uint8_t)h;
    *minute = (uint8_t)m;
}

// Encode version info into 32-bit value
// Encode version info into a 32-bit value
uint32_t encode_datetime_version(uint16_t year, uint8_t month, uint8_t day,
                                 uint8_t hour, uint8_t minute,
                                 uint8_t version_code) {
    // Ensure the year is within the valid range (2000-2063)
    if (year < 2000) year = 2000;
    if (year > 2063) year = 2063; // 6 bits: stores 0-63 (2000-2063)

    // Ensure month, day, hour, minute, and version_code are within valid ranges
    month = (month > 12) ? 12 : month; // 1-12 months
    day = (day > 31) ? 31 : day;       // 1-31 days
    hour = (hour > 23) ? 23 : hour;    // 0-23 hours
    minute = (minute > 59) ? 59 : minute; // 0-59 minutes
    version_code = (version_code > 127) ? 127 : version_code; // 1-127 version codes

    uint32_t val = 0;
    val |= ((year - 2000) & 0x3F) << 26;  // 6 bits for year (2000-2063)
    val |= (month & 0x0F) << 22;          // 4 bits for month (1-12)
    val |= (day & 0x1F) << 17;            // 5 bits for day (1-31)
    val |= (hour & 0x1F) << 12;           // 5 bits for hour (0-23)
    val |= (minute & 0x1F) << 7;          // 5 bits for minute (0-59)
    val |= (version_code & 0x7F);         // 7 bits for version code (1-127)

    return val;
}

// Decode 32-bit value back to version info
void decode_datetime_version(uint32_t encoded,
                             uint16_t* year, uint8_t* month, uint8_t* day,
                             uint8_t* hour, uint8_t* minute,
                             uint8_t* version_code) {
    *year         = 2000 + ((encoded >> 26) & 0x3F);
    *month        = (encoded >> 22) & 0x0F;
    *day          = (encoded >> 17) & 0x1F;
    *hour         = (encoded >> 12) & 0x1F;
    *minute       = (encoded >> 7)  & 0x1F;
    *version_code = encoded & 0x7F;
}

// Build encoded version number using current compile time
uint32_t build_version_code(void) {
    uint16_t y;
    uint8_t m, d, h, min;
    parse_build_date_time(&y, &m, &d, &h, &min);
    return encode_datetime_version(y, m, d, h, min, OTMS_VERSION_CODE);
}

// Format version info as string: YYYYMMDDHHMM_VER
void get_version_string(char* out_str, size_t max_len) {
    uint16_t y;
    uint8_t m, d, h, min;
    parse_build_date_time(&y, &m, &d, &h, &min);
    snprintf(out_str, max_len, "%04u%02u%02u%02u%02u_%03u", y, m, d, h, min, OTMS_VERSION_CODE);
}

void decode_version_string(char* out_str, size_t max_len) {
	  uint32_t version = 0;
    uint16_t y1;
    uint8_t m1, d1, h1, min1, code1;
	  if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
			version = app_meta_data.slot_a_version;
		else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_B)
			version = app_meta_data.slot_b_version;
		else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_LOADER)
			version = app_meta_data.loader_version;
		
    decode_datetime_version(version, &y1, &m1, &d1, &h1, &min1, &code1);
    snprintf(out_str, max_len, "%04u%02u%02u%02u%02u_%03u", y1, m1, d1, h1, min1, OTMS_VERSION_CODE);
}
/**
 * @brief Compare two encoded version values.
 * 
 * @param v1 First version (encoded uint32_t)
 * @param v2 Second version (encoded uint32_t)
 * @return int 
 *        < 0 if v1 < v2 (v2 is newer)
 *        = 0 if v1 == v2
 *        > 0 if v1 > v2 (v1 is newer)
 */
int32_t compare_versions(uint32_t v1, uint32_t v2) {
    uint16_t y1, y2;
    uint8_t m1, d1, h1, min1, code1;
    uint8_t m2, d2, h2, min2, code2;

    decode_datetime_version(v1, &y1, &m1, &d1, &h1, &min1, &code1);
    decode_datetime_version(v2, &y2, &m2, &d2, &h2, &min2, &code2);

    LOG_LEVEL("Bank Slot A Version: %04d%02d%02d%02d%02d (Code: %d)\n", y1, m1, d1, h1, min1, code1);
    LOG_LEVEL("Bank Slot B Version: %04d%02d%02d%02d%02d (Code: %d)\n", y2, m2, d2, h2, min2, code2);

    return (int32_t)(v1 - v2);
}

// Validate the encoded version information
bool check_encoded_version_valid(uint32_t encoded_version) {
    uint16_t year, month, day, hour, minute, version_code;

    // Decode fields from the 32-bit version code
    year         = 2000 + ((encoded_version >> 26) & 0x3F);  // 6 bits for year (2000-2063)
    month        = (encoded_version >> 22) & 0x0F;           // 4 bits for month (1-12)
    day          = (encoded_version >> 17) & 0x1F;           // 5 bits for day (1-31)
    hour         = (encoded_version >> 12) & 0x1F;           // 5 bits for hour (0-23)
    minute       = (encoded_version >> 7)  & 0x1F;           // 5 bits for minute (0-59)
    version_code = encoded_version & 0x7F;                    // 7 bits for version code (1-127)

    // Validate ranges for year, month, day, hour, minute, and version code
    if (year < 2000 || year > 2063)        return false;  // Year should be between 2000 and 2063
    if (month == 0 || month > 12)          return false;  // Month should be between 1 and 12
    if (day == 0 || day > 31)              return false;  // Day should be between 1 and 31
    if (hour > 23)                         return false;  // Hour should be between 0 and 23
    if (minute > 59)                       return false;  // Minute should be between 0 and 59
    if (version_code == 0 || version_code > 127) return false;  // Version code should be between 1 and 127

    return true;
}
