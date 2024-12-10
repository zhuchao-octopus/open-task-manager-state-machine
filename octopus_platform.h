/**
 * ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 * Header file for the Octopus Task Manager module.
 * Defines macros, includes required libraries, and declares functions.
 */
#ifndef ___OCTOPUS_TASK_MANAGER_PLATFORM_H___
#define ___OCTOPUS_TASK_MANAGER_PLATFORM_H___

/*******************************************************************************
 * PROJECT SWITCH MACROS
 * Define which platform and RTOS to use.
 */
//#define PLATFORM_ITE_OPEN_RTOS  // Use the ITE platform with OPEN RTOS
#define PLATFORM_CST_OSAL_RTOS // Use the CST platform with OSAL RTOS
// #define PLATFORM_CST_WIND_RTOS // Use the CST platform with WIND RTOS

/*******************************************************************************
 * INCLUDE FILES
 * Include standard libraries and platform-specific headers.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#ifdef PLATFORM_ITE_OPEN_RTOS
#include <sys/ioctl.h>
#include <sys/time.h>
#include <pthread.h>
#include <unistd.h>
#include "ite/ith.h"              // ITE hardware-specific definitions
#include "ite/itp.h"              // ITE platform-specific definitions
#include "uart/uart.h"            // UART module
#include "saradc/saradc.h"        // SAR ADC module
#include "math.h"                 // Math library
#include "SDL/SDL.h"              // SDL library for graphics
#include "openrtos/FreeRTOS.h"    // FreeRTOS kernel
#include "openrtos/queue.h"       // FreeRTOS queue support
#endif

#ifdef PLATFORM_CST_OSAL_RTOS
#include <stdarg.h>
#include "OSAL.h"                 // OS abstraction layer
#include "OSAL_PwrMgr.h"          // OS power management
#include "OSAL_Memory.h"          // OS memory management
#include "gatt.h"                 // Bluetooth GATT protocol
#include "hci.h"                  // Host Controller Interface for Bluetooth
#include "gapgattserver.h"        // GAP GATT server
#include "gattservapp.h"          // GATT service application
#include "gatt_profile_uuid.h"    // GATT profile UUIDs
#include "linkdb.h"               // Link database for Bluetooth
#include "peripheral.h"           // Peripheral device support
#include "gapbondmgr.h"           // GAP bonding manager
#include "hidkbdservice.h"        // HID keyboard service
#include "hiddev.h"               // HID device interface
#include "global_config.h"        // Global configuration settings
#include "hidkbd.h"               // HID keyboard implementation
#include "uart.h"                 // UART module
#include "ll.h"                   // Link layer support
#include "ll_common.h"            // Common link layer definitions
#include "ll_def.h"               // Link layer definitions
#include "dma.h"                  // Direct Memory Access support
#include "key.h"                  // Key handling
#include "spi.h"                  // SPI module
#include "gpio.h"                 // GPIO module
#include "flash.h"                // Flash memory handling
#include "clock.h"                // Clock management
#include "error.h"                // Error handling
#include "pwrmgr.h"               // Power management
#include "rom_sym_def.h"          // ROM symbol definitions
#include "sdk_config.h"
#include "rom_sym_def.h"
#include "types.h"
#include "uart.h"
#endif


#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * DEBUG SWITCH MACROS
 * Add macros for enabling or disabling debug functionality.
 */

/*******************************************************************************
 * GENERAL MACROS
 * Define common bit manipulation macros and constants.
 */
#ifdef PLATFORM_CST_OSAL_RTOS
#define GET_SYSTEM_TICK_COUNT (hal_systick() * 625 / 1000) // Convert system ticks to milliseconds
#define DELAY_US(us) (WaitUs(us))                          // Microsecond delay

#elif PLATFORM_ITE_OPEN_RTOS

#define GET_SYSTEM_TICK_COUNT (hal_systick() * 625 / 1000) // Convert system ticks to milliseconds
#define DELAY_US(us) (WaitUs(us))                          // Microsecond delay

#else
#define DELAY_US(us) 
#define GET_SYSTEM_TICK_COUNT 0
#endif

/*******************************************************************************
 * TYPE DEFINITIONS
 * Define any necessary types or structs here.
 */

/*******************************************************************************
 * CONSTANTS
 * Define any necessary constant values here.
 */
// Bit manipulation macros
#define BIT_0 0x01
#define BIT_1 0x02
#define BIT_2 0x04
#define BIT_3 0x08
#define BIT_4 0x10
#define BIT_5 0x20
#define BIT_6 0x40
#define BIT_7 0x80

// Set, clear, toggle, and get bit values
#define SetBit(VAR, Place) ((VAR) |= (uint8_t)(1 << (Place)))      // Set a specific bit
#define ClrBit(VAR, Place) ((VAR) &= (uint8_t)~(1 << (Place)))     // Clear a specific bit
#define ValBit(VAR, Place) ((VAR) & (uint8_t)(1 << (Place)))       // Get the value of a specific bit
#define ChgBit(Var, Place) ((Var) ^= (uint8_t)(1 << (Place)))      // Toggle a specific bit
#define GetBit(Var, Place) (((Var) & (1 << (Place))) ? 1 : 0)      // Check if a specific bit is set

// Macros for extracting and combining bits/bytes
#define LSB_BIT(BYTE) ((BYTE) & 0x0F)               // Extract least significant nibble
#define MSB_BIT(BYTE) (((BYTE) >> 4) & 0x0F)        // Extract most significant nibble
#define LSB(WORD) ((uint8_t)((WORD) & 0xFF))        // Extract least significant byte
#define MSB(WORD) ((uint8_t)(((WORD) >> 8) & 0xFF)) // Extract most significant byte

#define MK_LSB(WORD)				(uint8_t)(WORD & 0xFF)
#define MK_MSB(WORD)				(uint8_t)((WORD >> 8) & 0xFF)

#define MK_LSBWORD(WORD)		(uint16_t)(WORD & 0xFFFF)
#define MK_MSBWORD(WORD)		(uint16_t)((WORD >> 16) & 0xFFFF)

#define MK_BYTE(MSB, LSB)   (uint8_t)(((uint8_t)MSB << 4) + LSB)
#define MK_WORD(MSB, LSB)		(uint16_t)(((uint16_t)MSB << 8) + LSB)
#define MK_DWORD(MSB, LSB)  (uint32_t)(((uint32_t)MSB << 16) + LSB)

// Combine bytes/words
#define BYTE(MSB, LSB) 			((uint8_t)(((MSB) << 4) | (LSB)))
#define WORD(MSB, LSB) 			((uint16_t)(((MSB) << 8) | (LSB)))
#define DWORD(MSB, LSB) 		((uint32_t)(((MSB) << 16) | (LSB)))
#define MAKE_WORD(high,low) ((uint16_t)(((uint8_t)(low)) | ((uint16_t)((uint8_t)(high))) << 8))
#define MAKE_SIG_WORD(word) (*(int16_t *)(&word))

// Mathematical constants
#define PI_FLOAT (3.14159f) // Value of PI
/*******************************************************************************
 * FUNCTION DECLARATIONS
 * Declare external functions used in the task manager.
 */

#ifdef __cplusplus
}
#endif

#endif // ___OCTOPUS_TASK_MANAGER_PLATFORM_H___

