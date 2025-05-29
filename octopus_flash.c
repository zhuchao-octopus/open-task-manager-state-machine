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
#include "octopus_carinfor.h"
/////////////////////////////////////////////////////////////////////////////////////
typedef void (*AppEntryPoint)(void);  // Function pointer type for application entry
#define SCB_VTOR_ADDRESS (0xE000ED08) // SCB Vector Table Offset Register (VTOR)
#define SCB_VTOR (*(volatile uint32_t *)SCB_VTOR_ADDRESS)
#define CRC32_POLYNOMIAL (0x04C11DB7)  // Standard CRC32 polynomial
#define MAIN_APP_MAX_SIZE (100 * 1024) // 100KB Main Application size

app_meta_data_t app_meta_data = {0};


/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
void flash_init(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	hal_flash_init(0);
#endif
}

void flash_load_user_data_infor(void)
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
	
#ifdef USE_EEROM_FOR_DATA_SAVING
	  E2ROMReadToBuff(EEROM_APP_MATA_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));
		uint32_t calculated_crc = 0;//CalculateCRC32((uint8_t *)app_meta_data.slot_a_addr, MAIN_APP_SIZE);
		
		if(app_meta_data.slot_a_addr != MAIN_APP_SLOT_A_START_ADDR || 
			app_meta_data.slot_b_addr != MAIN_APP_SLOT_B_START_ADDR  ||
		  app_meta_data.bootloader_addr != BOOTLOADER_START_ADDR  ||
		  app_meta_data.boot_mode != BOOTLOADER_CONFIG_MODE_TYPE  ||
		  app_meta_data.app_crc_slot_a != calculated_crc
		 ) 
		{
			app_meta_data.bootloader_addr = BOOTLOADER_START_ADDR;
			app_meta_data.slot_a_addr = MAIN_APP_SLOT_A_START_ADDR;
			app_meta_data.slot_b_addr = MAIN_APP_SLOT_B_START_ADDR;
			app_meta_data.boot_mode = BOOTLOADER_CONFIG_MODE_TYPE;
			app_meta_data.last_boot_ok = BOOTLOADER_CONFIG_MODE_BANK;
			app_meta_data.active_slot = BOOTLOADER_CONFIG_MODE_BANK;
			app_meta_data.app_crc_slot_a = calculated_crc;
			E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));
		}
		
    LOG_LEVEL("Bootloader Address   : 0x%08X\n", app_meta_data.bootloader_addr);
    LOG_LEVEL("Slot A Address       : 0x%08X\n", app_meta_data.slot_a_addr);
    LOG_LEVEL("Slot B Address       : 0x%08X\n", app_meta_data.slot_b_addr);
		LOG_LEVEL("Slot A/B Size        : 0x%08X\n", MAIN_APP_SIZE);
		
    LOG_LEVEL("CRC (Slot A)         : 0x%08X\n", app_meta_data.app_crc_slot_a);
    LOG_LEVEL("CRC (Slot B)         : 0x%08X\n", app_meta_data.app_crc_slot_b);
    LOG_LEVEL("App State Flags      : 0x%08X\n", app_meta_data.app_state_flags);
    LOG_LEVEL("Meter Data Flags     : 0x%08X\n", app_meta_data.meter_data_flags);
    LOG_LEVEL("Config Data Flags    : 0x%08X\n", app_meta_data.config_data_flags);
    LOG_LEVEL("Active Slot          : %u\n",     app_meta_data.active_slot);
    LOG_LEVEL("Last Boot OK         : %u\n",     app_meta_data.last_boot_ok);
    LOG_LEVEL("Boot Mode            : 0x%02X\n", app_meta_data.boot_mode);
	  LOG_LEVEL("user data address    : 0x%08X\n", EEROM_DATAS_ADDRESS);
#endif
#ifdef USE_EEROM_FOR_DATA_SAVING
	if (app_meta_data.meter_data_flags == EEROM_DATAS_VALID_FLAG)
	{
			LOG_LEVEL("load meter data[%02d] ", sizeof(carinfo_meter_t));
			E2ROMReadToBuff(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
			LOG_BUFF((uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
	}
#endif
}

//void flash_set_app_meta_
void flash_save_carinfor_meter(void)
{
#ifdef USE_EEROM_FOR_DATA_SAVING
    LOG_BUFF_LEVEL((uint8_t *)task_carinfo_get_meter_info(), sizeof(carinfo_meter_t));
		LOG_LEVEL("lt_carinfo_meter.odo:%d\r\n",lt_carinfo_meter.odo);
	  if(lt_carinfo_meter.odo == 0)
		{
			return;
		}
    app_meta_data.meter_data_flags = EEROM_DATAS_VALID_FLAG;
    E2ROMWriteBuffTo(EEROM_APP_MATA_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));
    E2ROMWriteBuffTo(EEROM_CARINFOR_METER_ADDRESS, (uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t));
#endif
}
/*******************************************************************************
 * Jump to Main Application
 *******************************************************************************/
void JumpToApplication(uint32_t app_addr)
{
// Deinitialize hardware if needed
// Disable interrupts
#ifdef TASK_MANAGER_STATE_MACHINE_BOOTLOADER
	__disable_irq();

	// Get the main stack pointer (MSP) value from the application vector table
	uint32_t mainStackPointer = *(volatile uint32_t *)(app_addr);

	// Get the application entry point address
	AppEntryPoint appEntry = (AppEntryPoint)(*(volatile uint32_t *)(app_addr + 4));

	// Set the vector table to the application start address
	SCB_VTOR = MAIN_APP_START_ADDR;

	// Set the main stack pointer
	__set_MSP(mainStackPointer);

	// Jump to the application
	appEntry();
#endif
}
/*******************************************************************************
 * CRC Calculation
 *******************************************************************************/
uint32_t CalculateCRC32(uint8_t *data, uint32_t length)
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
bool VerifyAppCRC(uint32_t app_addr, uint32_t expected_crc)
{
    uint8_t *app_data = (uint8_t *)app_addr;
    uint32_t calculated_crc = CalculateCRC32(app_data, MAIN_APP_SIZE);

    LOG_LEVEL("CRC verify: app at 0x%08X | expected: 0x%08X | calculated: 0x%08X\r\n",
              app_addr, expected_crc, calculated_crc);

    return (calculated_crc == expected_crc);
}

/**
 * @brief Main bootloader logic: verify and jump to the valid application
 */
void BootloaderMainLoopEvent(void)
{
    // Read application metadata from EEPROM
    //E2ROMReadToBuff(E2ROM_META_ADDR, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));
    uint32_t active_app_addr = 0;
    uint32_t expected_crc = 0;

    // Select the currently active slot (A or B)
    if (app_meta_data.active_slot == BOOTLOADER_ACTIVE_SLOT_A)
    {
        active_app_addr = app_meta_data.slot_a_addr;
        expected_crc = app_meta_data.app_crc_slot_a;
        LOG_LEVEL("Active slot: A\r\n");
    }
    else
    {
        active_app_addr = app_meta_data.slot_b_addr;
        expected_crc =    app_meta_data.app_crc_slot_b;
        LOG_LEVEL("Active slot: B\r\n");
    }

    // Verify CRC of the active application
    if (VerifyAppCRC(active_app_addr, expected_crc))
    {
        LOG_LEVEL("App verified. Jumping to application at 0x%08X...\r\n", active_app_addr);
        JumpToApplication(active_app_addr);  // Hand over control to application
    }
    else
    {
        LOG_LEVEL("Active slot CRC verification failed!\r\n");

        // Try the backup slot if the active one is corrupted
        uint32_t backup_app_addr = (app_meta_data.active_slot == BOOTLOADER_ACTIVE_SLOT_A) ? app_meta_data.slot_b_addr : app_meta_data.slot_a_addr;
        uint32_t backup_crc = (app_meta_data.active_slot == BOOTLOADER_ACTIVE_SLOT_A) ? app_meta_data.app_crc_slot_b : app_meta_data.app_crc_slot_a;

        if (VerifyAppCRC(backup_app_addr, backup_crc))
        {
            LOG_LEVEL("Backup slot verified. Jumping to backup app at 0x%08X...\r\n", backup_app_addr);
            JumpToApplication(backup_app_addr);
        }
        else
        {
            // Both slots failed integrity check, enter firmware update mode
            LOG_LEVEL("Both slots failed verification. Entering upgrade mode.\r\n");
            //EnterFirmwareUpgradeMode();  // Implement your IAP or OTA entry point
        }
    }
}

uint32_t Flash_erase_user_app_arear(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	 if (app_meta_data.active_slot == BOOTLOADER_ACTIVE_SLOT_A)
	;//return hal_flash_erase_page_(MAIN_APP_SLOT_B_START_ADDR, MAIN_APP_BLOCK_COUNT);
	 else
	;//return hal_flash_erase_page_(MAIN_APP_SLOT_A_START_ADDR, MAIN_APP_BLOCK_COUNT);
	return 0;
#else
	return 0;
#endif
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

uint32_t FlashWriteBuffTo(uint32_t addr, uint8_t *buf, uint32_t length)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
#ifdef PLATFORM_CST_OSAL_RTOS
	return 0;
#else
	return hal_flash_write_(addr, buf, length);
#endif
#else
	return 0;
#endif
}
void FlashReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	hal_flash_read_(addr, buf, length);
#endif
}

void E2ROMReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length)
{
	hal_eeprom_read_(addr, buf, length);
}

void E2ROMWriteBuffTo(uint32_t addr, uint8_t *buf, uint32_t length)
{
	hal_eeprom_write_(addr, buf, length);
}
