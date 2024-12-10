/*******************************************************************************
 * @file     octopus.c
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


/*******************************************************************************
 * INCLUDES
 */
#include "octopus.h"
#include "octopus_platform.h"  // Core octopus library


/*******************************************************************************
 * DEBUG SWITCH MACROS
 */
/* Debug macros are not defined here, but this section is usually for enabling
   and disabling debug prints */

/*******************************************************************************
 * LOCAL FUNCTIONS DECLARE
 */
/* Local functions are declared here, but no specific ones are listed */

/*******************************************************************************
 * MACROS
 */
#define MAIN_TASK_TIMER_INTERVAL 10  // Main task loop timer interval (in ms)

/*******************************************************************************
 * TYPEDEFS
 */
/* Type definitions could be included here, but they are not defined */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
#ifdef PLATFORM_ITE_OPEN_RTOS
static pthread_t thread_task;  // Thread handle for task manager event loop
static pthread_attr_t thread_attr;  // Thread attributes (for setting stack size, etc.)
#endif

uint8_t TaskManagerStateMachine_Id_;  // Task manager state machine identifier

/*******************************************************************************
 * STATIC VARIABLES
 */
/* Static variables are typically used internally and not exposed globally */

/*******************************************************************************
 * FUNCTION DEFINITIONS
 */

/**
 * @brief Gets the current Task Manager State Machine ID.
 * @return The Task Manager State Machine ID.
 */
uint8_t GetTaskManagerStateMachineId(void)
{
    return TaskManagerStateMachine_Id_;
}

/**
 * @brief Initializes the Task Manager State Machine.
 * @param task_id The task ID to initialize.
 * @return 0 on success, non-zero on failure.
 */
uint16_t TaskManagerStateMachineInit(uint8_t task_id)
{
    TaskManagerStateMachine_Id_ = task_id;  // Store the task ID in the global variable
    LOG_("\r\n");	
    LOG_LEVEL(F_NAME,"OTMS initialization task_id=%02x\r\n", TaskManagerStateMachine_Id_);
    LOG_LEVEL(F_NAME,"OTMS datetime:%s\r\n", OTMS_RELEASE_DATA_TIME);
    LOG_LEVEL(F_NAME,"OTMS version :%s\r\n", OTMS_VERSION);
	  /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize hardware abstraction layers (HAL)
    hal_gpio_init(0);  // Initialize GPIO
    hal_timer_init(5);  // Initialize timer with interval of 5 (could be milliseconds)
	hal_flash_init(0);
    hal_com_uart_init(0);  // Initialize UART communication protocol
    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Initialize user task manager
    task_manager_init();  // Initialize the task manager
    task_manager_start();  // Start the task manager
    #ifdef TASK_MANAGER_STATE_MACHINE_MCU
	  system_handshake_with_app();
	#endif
	#ifdef TASK_MANAGER_STATE_MACHINE_SOC
	  system_handshake_with_mcu();
	#endif
    LOG_("\r\n###########################BOOT COMPLETE###########################\r\n\r\n");

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    // Enable task manager state matching main loop
    #ifdef PLATFORM_CST_OSAL_RTOS
    osal_start_reload_timer(TaskManagerStateMachine_Id_, DEVICE_TIMER_EVENT, MAIN_TASK_TIMER_INTERVAL);//timeout_value unit ms
    #endif

    #ifdef PLATFORM_ITE_OPEN_RTOS 
    pthread_attr_init(&thread_attr);  // Initialize thread attributes
    pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED);  // Set thread to detached state
    pthread_attr_setstacksize(&thread_attr, CFG_OTSM_STACK_SIZE);  // Set the stack size for the thread
    pthread_create(&thread_task, &thread_attr, TaskManagerStateMachineEventLoop, NULL);  // Create the task manager event loop thread
    #endif

    return 0;
}

#ifdef PLATFORM_CST_OSAL_RTOS
/**
 * @brief Handles events in the Task Manager State Machine for CST OSAL RTOS.
 * @param task_id The task ID of the calling task.
 * @param events The event mask that defines which events are active.
 * @return The events that were handled (removed from the active event mask).
 */
uint16 TaskManagerStateMachineEventLoop(uint8 task_id, uint16 events)
{
    if (events & DEVICE_TIMER_EVENT)  // If the timer event is triggered
    {
        /////////////////////////////////////////////////////////////////////////////////////////////////////
        task_manager_run();  // Run the task manager to handle pending tasks
        return (events ^ DEVICE_TIMER_EVENT);  // Remove the timer event from the active events
    }
    else if (events & DEVICE_BLE_PAIR)  // If BLE pairing event is triggered
    {
        LOG_LEVEL(F_NAME, "\r\ntask_id=%d events=%d ble pair\r\n", task_id, events);
        send_message(TASK_ID_BLE, MSG_DEVICE_NORMAL_EVENT, events, events);  // Send BLE pair event to BLE task
        return (events ^ DEVICE_BLE_PAIR);  // Remove the BLE pair event from the active events
    }
    else if (events & DEVICE_BLE_BONDED)  // If BLE bonded event is triggered
    {
        LOG_LEVEL(F_NAME, "\r\ntask_id=%d events=%d ble bonded\r\n", task_id, events);
        send_message(TASK_ID_BLE, MSG_DEVICE_NORMAL_EVENT, events, events);  // Send BLE bonded event to BLE task
        return (events ^ DEVICE_BLE_BONDED);  // Remove the BLE bonded event from the active events
    }
    else if (events & DEVICE_BLE_CONNECTED)  // If BLE connected event is triggered
    {
        LOG_LEVEL(F_NAME, "\r\ntask_id=%d events=%d ble connected\r\n", task_id, events);
        send_message(TASK_ID_BLE, MSG_DEVICE_NORMAL_EVENT, events, events);  // Send BLE connected event to BLE task
        return (events ^ DEVICE_BLE_CONNECTED);  // Remove the BLE connected event from the active events
    }
    else if (events & DEVICE_BLE_DISCONNECTED)  // If BLE disconnected event is triggered
    {
        LOG_LEVEL(F_NAME, "\r\ntask_id=%d events=%d ble disconnected\r\n", task_id, events);
        send_message(TASK_ID_BLE, MSG_DEVICE_NORMAL_EVENT, events, events);  // Send BLE disconnected event to BLE task
        return (events ^ DEVICE_BLE_DISCONNECTED);  // Remove the BLE disconnected event from the active events
    }
    else
    {
        LOG_LEVEL(F_NAME, "task_id=%d default events=%d\r\n", task_id, events);  // Log unhandled events
    }

    return 0;  // Return 0 if no events were handled
}

#elif defined(PLATFORM_ITE_OPEN_RTOS)
/**
 * @brief Task manager state machine event loop for ITE Open RTOS.
 * @param arg Arguments passed to the thread (not used here).
 * @return NULL when the thread exits.
 */
void* TaskManagerStateMachineEventLoop(void* arg)
{
    uint32_t wait_cnt = 0;
    while (1)
    {
        task_manager_run();  // Run the task manager to handle tasks in the event loop
        usleep(MAIN_TASK_TIMER_INTERVAL*1000);  // Sleep for 10 millisecond to control loop frequency
    }
    return 0;  // Exit the thread
}
#else

#endif

