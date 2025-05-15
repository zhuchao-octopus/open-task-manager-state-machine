/******************************************************************************
 * @file    gpio_implementation.c
 * @brief   GPIO management implementation for hardware platform.
 *          This module handles GPIO initialization, status polling, and key
 *          event processing, as well as interaction with the task manager.
 * @version 1.0
 * @date    2025-05-10
 *
 * @note    Platform-specific functions are defined in octopus_platform.h
 *          and GPIO configuration is handled for status and key polling.
 *          Interrupt-based handling can be added for real-time event capture.
 *          This implementation uses polling for simplicity and clarity.
 *
 *          Key Features:
 *          - GPIO Initialization
 *          - Polling for GPIO status
 *          - Polling for Key Press status
 *          - Event processing for key press and release
 *          - Task management state transitions
 *
 * @author  [ak47]
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "octopus_platform.h"     // Include platform-specific header for hardware platform details
#include "octopus_log.h"          // Include logging functions for debugging
#include "octopus_task_manager.h" // Include task manager for scheduling tasks
#include "octopus_gpio.h"         // Include GPIO control and configuration
#include "octopus_tickcounter.h"  // Include tick counter for timing operations
#include "octopus_msgqueue.h"     // Include message queue for inter-task communication
#include "octopus_flash.h"        // Include flash memory access functions
#include "octopus_key.h"          // Include key status and event handling

#ifdef TASK_MANAGER_STATE_MACHINE_GPIO
/*******************************************************************************
 * DEBUG SWITCH MACROS
 */

/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */

void GPIOInit(void);
void PollingGPIOStatus(GPIO_GROUP *gpiox, uint16_t pin, GPIO_STATUS *gpio_status);
void PollingGPIOKeyStatus(GPIO_GROUP *gpiox, uint16_t pin, GPIO_KEY_STATUS *key_status);
void ProcessKeyDispatchedEvent(GPIO_KEY_STATUS *key_status);

static uint32_t l_t_msg_wait_50_timer;

/*******************************************************************************
 * GLOBAL VARIABLES
 */

GPIO_STATUS acc_status = {false, true, 0, 0};
GPIO_STATUS ddd_status = {false, true, 0, 0};
GPIO_STATUS zzd_status = {false, true, 0, 0};
GPIO_STATUS yzd_status = {false, true, 0, 0};
GPIO_STATUS skd_status = {false, true, 0, 0};

GPIO_KEY_STATUS key_status_power = {OCTOPUS_KEY_POWER, 0};
GPIO_KEY_STATUS *gpio_key_array[] = {&key_status_power};
// static bool module_send_handler(ptl_frame_type_t frame_type, ptl_frame_cmd_t cmd, uint16_t param, ptl_proc_buff_t *buff);
// static bool module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void app_gpio_init_running(void)
{
    LOG_LEVEL("app_gpio_init_running\r\n");
    GPIOInit();
    // com_uart_ptl_register_module(MSGMODULE_SYSTEM, module_send_handler, module_receive_handler);
    OTMS(TASK_ID_GPIO, OTMS_S_INVALID);
}

void app_gpio_start_running(void)
{
    LOG_LEVEL("app_gpio_start_running\r\n");
    OTMS(TASK_ID_GPIO, OTMS_S_ASSERT_RUN);
}

void app_gpio_assert_running(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    StartTickCounter(&l_t_msg_wait_50_timer);
    OTMS(TASK_ID_GPIO, OTMS_S_RUNNING);
#endif
}

void app_gpio_running(void)
{
    if (GetTickCounter(&l_t_msg_wait_50_timer) >= GPIO_POLLING_PERIOD_MS)
    {
        // PollingGPIOStatus(GPIO_ACC_PIN,&acc_status);
        // PollingGPIOStatus(GPIO_DDD_PIN,&ddd_status);
        // PollingGPIOStatus(GPIO_ZZD_PIN,&zzd_status);
        // PollingGPIOStatus(GPIO_YZD_PIN,&yzd_status);
        // PollingGPIOStatus(GPIO_SKD_PIN,&skd_status);
        PollingGPIOKeyStatus(GPIO_POWER_KEY_GROUP, GPIO_POWER_KEY_PIN, &key_status_power);
        ProcessKeyDispatchedEvent(&key_status_power);
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
#if 0
			if(acc_status.changed)
			{
				LOG_LEVEL("get acc status=%d\r\n",acc_status.offon);
				send_message(TASK_ID_BLE, MSG_DEVICE_ACC_EVENT, GPIO_ACC_PIN, acc_status.offon);
				if(acc_status.offon)
				send_message(TASK_ID_SYSTEM, MSG_DEVICE_POWER_EVENT, CMD_MODSYSTEM_POWER_ON, acc_status.offon);
				else
				send_message(TASK_ID_SYSTEM, MSG_DEVICE_POWER_EVENT, CMD_MODSYSTEM_POWER_OFF, acc_status.offon);
				
				acc_status.changed=false;
			}	
			
	  	if(ddd_status.changed)
			{
				LOG_LEVEL("get ddd status=%d\r\n",ddd_status.offon);
				send_message(TASK_ID_CAR_INFOR, MSG_DEVICE_GPIO_EVENT, GPIO_DDD_PIN, ddd_status.offon);
				ddd_status.changed=false;
			}	
			
			if(zzd_status.changed)
			{
				LOG_LEVEL("get zzd status=%d\r\n",zzd_status.offon);
				send_message(TASK_ID_CAR_INFOR, MSG_DEVICE_GPIO_EVENT, GPIO_ZZD_PIN, zzd_status.offon);
				zzd_status.changed=false;
			}	
			
			if(yzd_status.changed)
			{
				LOG_LEVEL("get yzd status=%d\r\n",yzd_status.offon);
				send_message(TASK_ID_CAR_INFOR, MSG_DEVICE_GPIO_EVENT, GPIO_YZD_PIN, yzd_status.offon);
				yzd_status.changed=false;
			}	
			
			if(skd_status.changed)
			{
				LOG_LEVEL("get skd status=%d\r\n",skd_status.offon);
				send_message(TASK_ID_CAR_INFOR, MSG_DEVICE_GPIO_EVENT, GPIO_SKD_PIN, skd_status.offon);
				skd_status.changed=false;
			}
#endif

        StartTickCounter(&l_t_msg_wait_50_timer);
    }
}

void app_gpio_post_running(void)
{
}

void app_gpio_stop_running(void)
{
    OTMS(TASK_ID_GPIO, OTMS_S_INVALID);
}
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Initializes the GPIO configuration.
 *
 * This function is used to initialize the configuration of General Purpose
 * Input/Output (GPIO) pins. This includes setting the pin modes, directions,
 * and any pull-up or pull-down configurations. This function is currently empty
 * and should be implemented based on specific hardware requirements.
 */
void GPIOInit(void)
{
    // TODO: Implement GPIO initialization here
}

/**
 * @brief Polls the status of a specified GPIO pin and updates its status structure.
 *
 * This function continuously checks the state of a specified GPIO pin and
 * applies a redundancy mechanism to filter out noise or glitches. If the pin
 * remains in the same state for a predefined number of polling cycles
 * (`GPIO_STATUS_REDUNDANCY`), the status is considered stable and is reflected
 * in the `gpio_status` structure.
 *
 * @param gpiox Pointer to the GPIO group (port) to which the pin belongs.
 * @param pin The specific GPIO pin number to be polled.
 * @param gpio_status Pointer to the `GPIO_STATUS` structure to be updated with the current status.
 */
void PollingGPIOStatus(GPIO_GROUP *gpiox, uint16_t pin, GPIO_STATUS *gpio_status)
{
    // Check the current state of the GPIO pin
    if (GPIO_PIN_READ(gpiox, pin))
    {
        // If the pin is high, increment the high state counter and reset the low counter
        gpio_status->count1++;
        gpio_status->count2 = 0;

        // If the high state is consistent for more than the redundancy threshold
        if (gpio_status->count1 > GPIO_STATUS_REDUNDANCY)
        {
            // If the state has changed from low to high, mark it as changed
            if (!gpio_status->offon)
                gpio_status->changed = true;
            else
                gpio_status->changed = false;

            // Update the status
            gpio_status->offon = true;
            gpio_status->count1 = 0;
        }
    }
    else
    {
        // If the pin is low, reset the high counter and increment the low counter
        gpio_status->count1 = 0;
        gpio_status->count2++;

        // If the low state is consistent for more than the redundancy threshold
        if (gpio_status->count2 > GPIO_STATUS_REDUNDANCY)
        {
            // If the state has changed from high to low, mark it as changed
            if (gpio_status->offon)
                gpio_status->changed = true;
            else
                gpio_status->changed = false;

            // Update the status
            gpio_status->offon = false;
            gpio_status->count2 = 0;
        }
    }
}

/**
 * @brief Polls the status of a specified GPIO pin configured as a key (button).
 *
 * This function checks the current status of a button connected to a GPIO pin.
 * It identifies different button states including:
 * - Pressed
 * - Long Pressed
 * - Long-Long Pressed
 * - Released
 *
 * The status is updated in the provided `key_status` structure. The function
 * uses debounce logic to ensure reliable detection.
 *
 * @param gpiox Pointer to the GPIO group (port) to which the pin belongs.
 * @param pin The specific GPIO pin number to be polled.
 * @param key_status Pointer to the `GPIO_KEY_STATUS` structure to be updated with the current status.
 */
void PollingGPIOKeyStatus(GPIO_GROUP *gpiox, uint16_t pin, GPIO_KEY_STATUS *key_status)
{
    // Read the current status of the GPIO pin (1 for high, 0 for low)
    uint8_t g_status = GPIO_PIN_READ(gpiox, pin);

    // If the key is pressed (GPIO pin is low, assuming active-low logic)
    if (!g_status)
    {
        // Increment the press counter with redundancy protection
        if (key_status->press_count < GPIO_KEY_STATUS_MAX_REDUNDANCY)
        {
            key_status->press_count++;
            key_status->long_press_duration = key_status->press_count;
        }

        // If the press duration exceeds the defined short-press period
        if (key_status->press_count > GPIO_KEY_STATUS_PRESS_PERIOD && !key_status->pressed)
        {
            key_status->pressed = true;
            key_status->release = false;
            key_status->dispatched = false;
            key_status->ignore = false;
            key_status->state = KEY_STATE_PRESSED;
        }

        // If the press duration exceeds the defined long-press period
        if (key_status->press_count > GPIO_KEY_STATUS_LONG_PRESS_PERIOD && key_status->pressed)
        {
            key_status->pressed = true;
            key_status->release = false;
            key_status->dispatched = false;
            key_status->state = KEY_STATE_LONG_PRESSED;
        }

        // If the press duration exceeds the defined long-long-press period
        if (key_status->press_count > GPIO_KEY_STATUS_LONG_LONG_PRESS_PERIOD && key_status->pressed)
        {
            key_status->pressed = true;
            key_status->release = false;
            key_status->dispatched = false;
            key_status->state = KEY_STATE_LONG_LONG_PRESSED;
        }
    }
    else
    {
        // If the key is released (GPIO pin is high)
        if (key_status->pressed)
        {
            key_status->release = true;
            key_status->dispatched = false;
        }

        // Reset the pressed status and the counter
        key_status->pressed = false;
        key_status->press_count = 0;
    }
}

/**
 * @brief Processes key press and release events for a specified GPIO key.
 *
 * This function checks the status of the key (`GPIO_KEY_STATUS`). If an event
 * (press or release) has not been dispatched yet, it sends the appropriate
 * message using `send_message()` and marks the event as dispatched to prevent
 * redundant messages for the same event.
 *
 * @param key_status Pointer to the `GPIO_KEY_STATUS` structure representing the key's status.
 */
void ProcessKeyDispatchedEvent(GPIO_KEY_STATUS *key_status)
{
    // Check if the event has not been dispatched already or is ignored by user
    if (!key_status->dispatched && !key_status->ignore)
    {
        // If the key is in the "pressed" state, send a "key down" event
        if (key_status->pressed)
        {
            /**
             * TASK_ID_KEY            - Identifier for the task handling key events.
             * MSG_DEVICE_KEY_DOWN_EVENT - Message type indicating a key press.
             * key_status->key        - The key identifier.
             * KEY_STATE_PRESSED      - The current state of the key.
             */

            send_message(TASK_ID_KEY, MSG_DEVICE_KEY_DOWN_EVENT, key_status->key, KEY_STATE_PRESSED);

            // Mark the event as dispatched to prevent duplicate messages
            key_status->dispatched = true;
        }
        // If the key is in the "release" state, send a "key up" event
        else if (key_status->release)
        {
            /**
             * TASK_ID_KEY            - Identifier for the task handling key events.
             * MSG_DEVICE_KEY_UP_EVENT - Message type indicating a key release.
             * key_status->key        - The key identifier.
             * KEY_STATE_RELEASED     - The current state of the key.
             */
            send_message(TASK_ID_KEY, MSG_DEVICE_KEY_UP_EVENT, key_status->key, KEY_STATE_RELEASED);

            // Mark the event as dispatched to prevent duplicate messages
            key_status->dispatched = true;
        }
    }
}

/**
 * @brief Retrieves the status structure for a specific key.
 *
 * This function searches through an array of GPIO key status structures
 * (`gpio_key_array`) to find the matching key. If the key is found, a pointer
 * to its status structure is returned; otherwise, the function returns `NULL`.
 *
 * @param key The key identifier to search for.
 * @return GPIO_KEY_STATUS* Pointer to the key's status structure if found; otherwise, `NULL`.
 */
GPIO_KEY_STATUS *get_key_status_by_key(uint8_t key)
{
    // Calculate the size of the array
    size_t array_size = sizeof(gpio_key_array) / sizeof(gpio_key_array[0]);

    // Iterate over the array to find the key
    for (size_t i = 0; i < array_size; i++)
    {
        // If the key matches, return its status structure
        if (gpio_key_array[i]->key == key)
        {
            return gpio_key_array[i];
        }
    }

    // If the key is not found, return NULL
    return NULL;
}

bool IsPowerOn(void)
{
    return GPIO_PIN_READ(GPIO_POWER_F113_GROUP, GPIO_POWER_F113_PIN);
}

void power_on_off(bool onoff)
{
    if (onoff)
        GPIO_PIN_WRITE(GPIO_POWER_F113_GROUP, GPIO_POWER_F113_PIN, Bit_SET);
    else
        GPIO_PIN_WRITE(GPIO_POWER_F113_GROUP, GPIO_POWER_F113_PIN, Bit_RESET);
}

void gpio_on_off(GPIO_GROUP *gpiox, uint16_t pin, bool onoff)
{
    if (onoff)
        GPIO_PIN_WRITE(gpiox, pin, Bit_SET);
    else
        GPIO_PIN_WRITE(gpiox, pin, Bit_RESET);
}
/*
bool module_send_handler(ptl_frame_type_t frame_type, ptl_frame_cmd_t cmd, uint16_t param, ptl_proc_buff_t *buff)
{
    return true;
}


bool module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
  return true;
}*/

#endif
