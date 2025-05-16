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

void flash_print_user_data_infor(void)
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	DEBUG_BOOTLOADER_ADDR_INFO();
	DEBUG_MAIN_APP_ADDR_INFO();
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_EEROM_FOR_DATA_SAVING
	E2ROMReadToBuff(EEROM_START_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));

	LOG_LEVEL("eeprom data address    : 0x%08X\n", EEROM_DATAS_ADDRESS);
	LOG_LEVEL("eeprom apppp valid flag: 0x%08X\n", app_meta_data.user_APP_flag);
	LOG_LEVEL("eeprom datas valid flag: 0x%08X\n", app_meta_data.user_data_flag);
	LOG_LEVEL("eeprom apppp code crc  : 0x%08X\n", app_meta_data.crc);
#endif
}
/*******************************************************************************
 * Jump to Main Application
 *******************************************************************************/
void JumpToMainApplication(void)
{
	// Deinitialize hardware if needed
	// Disable interrupts
	__disable_irq();

	// Get the main stack pointer (MSP) value from the application vector table
	uint32_t mainStackPointer = *(volatile uint32_t *)(MAIN_APP_START_ADDR);

	// Get the application entry point address
	AppEntryPoint appEntry = (AppEntryPoint)(*(volatile uint32_t *)(MAIN_APP_START_ADDR + 4));

	// Set the vector table to the application start address
	SCB_VTOR = MAIN_APP_START_ADDR;

	// Set the main stack pointer
	__set_MSP(mainStackPointer);

	// Jump to the application
	appEntry();
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

/*******************************************************************************
 * Verify Main Application Integrity
 *******************************************************************************/
bool VerifyMainAppIntegrity(uint32_t expected_crc)
{
	uint8_t *app_data = (uint8_t *)MAIN_APP_START_ADDR;
	uint32_t app_size = MAIN_APP_MAX_SIZE;
	uint32_t calculated_crc = CalculateCRC32(app_data, app_size);
	LOG_LEVEL("calculated crc:%08x expected crc:%08x\r\n", calculated_crc, expected_crc);
	return (calculated_crc == expected_crc);
}
/*******************************************************************************
 * Bootloader Main Loop
 *******************************************************************************/
void BootloaderMainLoopEvent(void)
{
	// uint32_t expected_crc = 0xDEADBEEF; // This should be replaced with real CRC
	if (app_meta_data.crc == 0)
		E2ROMReadToBuff(EEROM_START_ADDRESS, (uint8_t *)&app_meta_data, sizeof(app_meta_data_t));

	if (VerifyMainAppIntegrity(app_meta_data.crc))
	{
		// If application is valid, jump to it
		LOG_LEVEL("verify user applicaton success jump to %08x ...\r\n", MAIN_APP_START_ADDR);
		JumpToMainApplication();
	}
	else
	{
		// Handle failure (e.g., log the error, enter upgrade mode, etc.)
		LOG_LEVEL("verify user applicaton failed at %08x\r\n", MAIN_APP_START_ADDR);
	}
}

uint32_t Flash_erase_user_app_arear(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	return hal_flash_erase_page(MAIN_APP_START_ADDR, MAIN_APP_BLOCK_COUNT);
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
	return hal_flash_write(addr, buf, length);
#else
	return 0;
#endif
}
void FlashReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length)
{
#ifdef TASK_MANAGER_STATE_MACHINE_FLASH
	hal_flash_read(addr, buf, length);
#endif
}

void E2ROMReadToBuff(uint32_t addr, uint8_t *buf, uint32_t length)
{
	hal_eeprom_read(addr, buf, length);
}

void E2ROMWriteBuffTo(uint32_t addr, uint8_t *buf, uint32_t length)
{
	hal_eeprom_write(addr, buf, length);
}
