/**
 * ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 * Header file for the Octopus Task Manager module.
 * Defines macros, includes required libraries, and declares functions.
 */

#ifndef __OCTOPUS_TASK_MANAGER_GPIO_HAL_H__
#define __OCTOPUS_TASK_MANAGER_GPIO_HAL_H__

#include "octopus_platform.h" // Include the Octopus module-related header file

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************* 
 * MACRO DEFINITIONS
 */

// Define GPIO pin macros and event identifiers if the platform is CST OSAL RTOS
#ifdef PLATFORM_CST_OSAL_RTOS

// Event identifiers for user test timers
#define USR_TEST_TIMER1_EVT                 0x0001
#define USR_TEST_TIMER2_EVT                 0x0002

// Define GPIO pins for different hardware components
#define GPIO_ACC_SOC_PIN 										P1
#define GPIO_ACC_PIN 											P0
#define GPIO_KEY_PIN 											P14
#define GPIO_DDD_PIN 											P31
#define GPIO_ZZD_PIN 											P32
#define GPIO_YZD_PIN 											P33
#define GPIO_SKD_PIN 											P34

#define GPIO_SIF_R_PIN 						  				    P15
#define GPIO_SIF_S_PIN 						  				    P24

// Macros for controlling GPIO pin states (Low/High)
#define GPIO_ACC_SOC_LOW()   		 (HalGpioSet(GPIO_ACC_SOC_PIN,Bit_DISABLE))  // Set GPIO_ACC_SOC_PIN to Low
#define GPIO_ACC_SOC_HIGH()  		 (HalGpioSet(GPIO_ACC_SOC_PIN,Bit_ENABLE))   // Set GPIO_ACC_SOC_PIN to High

// Macros for writing to and reading from GPIO pins
#define GPIO_PIN_WRITE(pin)  		 (HalGpioSet((GpioPin_t)pin))   // Write to the specified GPIO pin
#define GPIO_PIN_READ(pin)   		 (HalGpioGet((GpioPin_t)pin))   // Read the state of the specified GPIO pin

// Specific macros to read GPIO states for various pins
#define GPIO_PIN_READ_ACC()  		 (HalGpioGet(GPIO_ACC_PIN))   // Read the state of ACC_PIN
#define GPIO_PIN_READ_DDD()  		 (HalGpioGet(GPIO_DDD_PIN))   // Read the state of DDD_PIN
#define GPIO_PIN_READ_ZZD()  		 (HalGpioGet(GPIO_ZZD_PIN))   // Read the state of ZZD_PIN
#define GPIO_PIN_READ_YZD()  	 	 (HalGpioGet(GPIO_YZD_PIN))   // Read the state of YZD_PIN
#define GPIO_PIN_READ_SKD()  		 (HalGpioGet(GPIO_SKD_PIN))   // Read the state of SKD_PIN

// Macros for setting SIF pins
#define GPIO_PIN_SIF_SET_LOW()   (HalGpioSet(GPIO_SIF_S_PIN,Bit_DISABLE))  // Set GPIO_SIF_S_PIN to Low
#define GPIO_PIN_SIF_SET_HIGH()  (HalGpioSet(GPIO_SIF_S_PIN,Bit_ENABLE))   // Set GPIO_SIF_S_PIN to High
#define GPIO_PIN_READ_SIF()      (HalGpioGet(GPIO_SIF_R_PIN))  // Read the state of SIF_R_PIN

#elif PLATFORM_ITE_OPEN_RTOS
// Define macros for ITE Open RTOS platform (if needed, to be implemented)

#else
#define GPIO_ACC_SOC_PIN 		(0x00)								
#define GPIO_ACC_PIN 				(0x00)							
#define GPIO_KEY_PIN 				(0x00)							
#define GPIO_DDD_PIN 				(0x00)							
#define GPIO_ZZD_PIN 				(0x00)							
#define GPIO_YZD_PIN 				(0x00)							
#define GPIO_SKD_PIN 				(0x00)							

#define GPIO_SIF_R_PIN 						  				    
#define GPIO_SIF_S_PIN 						  				    

// Macros for controlling GPIO pin states (Low/High)
#define GPIO_ACC_SOC_LOW()   		 // Set GPIO_ACC_SOC_PIN to Low
#define GPIO_ACC_SOC_HIGH()  		 // Set GPIO_ACC_SOC_PIN to High

// Macros for writing to and reading from GPIO pins
#define GPIO_PIN_WRITE(pin)  		 (0x00)// Write to the specified GPIO pin
#define GPIO_PIN_READ(pin)   		 (0x00)// Read the state of the specified GPIO pin

// Specific macros to read GPIO states for various pins
#define GPIO_PIN_READ_ACC()  		 (0x00)// Read the state of ACC_PIN
#define GPIO_PIN_READ_DDD()  		 (0x00)// Read the state of DDD_PIN
#define GPIO_PIN_READ_ZZD()  		 (0x00)// Read the state of ZZD_PIN
#define GPIO_PIN_READ_YZD()  	 	 (0x00)// Read the state of YZD_PIN
#define GPIO_PIN_READ_SKD()  		 (0x00)// Read the state of SKD_PIN

// Macros for setting SIF pins
#define GPIO_PIN_SIF_SET_LOW()   			 // Set GPIO_SIF_S_PIN to Low
#define GPIO_PIN_SIF_SET_HIGH()  			 // Set GPIO_SIF_S_PIN to High
#define GPIO_PIN_READ_SIF()      (0x00)// Read the state of SIF_R_PIN
#endif

/*********************************************************************
 * FUNCTION DECLARATIONS
 */

// Function to initialize GPIOs
void hal_gpio_init(uint8_t task_id);

// Function to get GPIO key mask code based on the pin number
uint8_t hal_get_gpio_key_mask_code(uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_TASK_MANAGER_GPIO_HAL_H__ */

