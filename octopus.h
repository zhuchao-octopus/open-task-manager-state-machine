/*******************************************************************************
 * @file     octopus.h
 * @brief    Header file for the Octopus Task Manager.
 *           This file provides function declarations, macros, and type definitions
 *           for managing tasks and event loops in the Octopus system.
 *           It supports platform-specific configurations and RTOS choices.
 *
 * @note     This file is part of the Octopus project and should be included
 *           in the task manager implementation files.
 *           It allows users to define task states, events, and handle 
 *           the task scheduling mechanisms.
 *
 * @version  1.0.0
 * @date     2024-12-09
 * @author   Octopus Team

 ******************************************************************************/

#ifndef ___OCTOPUS_TASK_MANAGER_H___
#define ___OCTOPUS_TASK_MANAGER_H___


/*******************************************************************************
 * INCLUDE FILES
 * Include standard libraries and platform-specific headers.
 */
 
#include "octopus_platform.h"
#include "octopus_timer_hal.h"      // Hardware abstraction for timer
#include "octopus_uart_hal.h"
#include "octopus_gpio_hal.h"
#include "octopus_flash_hal.h"
#include "octopus_ble_hal.h"

#include "octopus_uart_ptl.h"       // UART protocol handling
#include "octopus_carinfor.h" 
#include "octopus_sif.h"            // SIF protocol interface
#include "octopus_tickcounter.h"    // Timer management
#include "octopus_msgqueue.h"       // Message queue management
#include "octopus_task_manager.h"   // Task manager
#include "octopus_log.h"            // Octopus-specific logging

#ifdef PLATFORM_CST_WIND_RTOS
// Include headers specific to WIND RTOS if required
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

/*******************************************************************************
 * TYPE DEFINITIONS
 * Define any necessary types or structs here.
 */

/*******************************************************************************
 * CONSTANTS
 * Define any necessary constant values here.
 */

/*******************************************************************************
 * FUNCTION DECLARATIONS
 * Declare external functions used in the task manager.
 */

// Retrieves the main user task ID.
uint8_t GetTaskManagerStateMachineId(void);

// Initializes the task manager's main task.
uint16_t TaskManagerStateMachineInit(uint8_t task_id);

// The main event loop for the task manager.
#ifdef PLATFORM_CST_OSAL_RTOS
uint16_t TaskManagerStateMachineEventLoop(uint8_t task_id, uint16_t events);
#elif defined(PLATFORM_ITE_OPEN_RTOS)
void* TaskManagerStateMachineEventLoop(void* arg);
#endif

#ifdef __cplusplus
}
#endif

#endif // ___OCTOPUS_TASK_MANAGER_H___

