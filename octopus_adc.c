

/*****************************************************************************************************************
 * INCLUDES
 */
#include "octopus_adc.h"
#include "octopus_flash.h"
#include "octopus_gpio.h"        // Include GPIO HAL for hardware-specific functionality
#include "octopus_uart_ptl.h"    // Include UART protocol header
#include "octopus_uart_upf.h"    // Include UART protocol header
#include "octopus_tickcounter.h" // Include tick counter for timing operations
#include "octopus_msgqueue.h"    // Include message queue header for task communication
#include "octopus_message.h"     // Include message id for inter-task communication

/*****************************************************************************************************************
 * 配置参数
/*****************************************************************************************************************/

// 平均值结果，每个通道一个值
uint16_t adc_channel_value[ADC_CHANNEL_NUM];

static uint32_t l_t_msg_wait_timer;
static void task_adc_action_handler(void);

void task_adc_key_polling_event_status(adc_channel_t channel);
void task_adc_key_polling_event_dispatcher(void);
/*****************************************************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void task_adc_init_running(void)
{
    LOG_LEVEL("task_key_init_running\r\n");
    OTMS(TASK_MODULE_ADC, OTMS_S_INVALID);
}

void task_adc_start_running(void)
{
    LOG_LEVEL("task_key_start_running\r\n");
    OTMS(TASK_MODULE_ADC, OTMS_S_ASSERT_RUN);
}

void task_adc_assert_running(void)
{

    StartTickCounter(&l_t_msg_wait_timer);
    // StartTickCounter(&l_t_msg_boot_wait_timer);
    OTMS(TASK_MODULE_ADC, OTMS_S_RUNNING);
}

void task_adc_running(void)
{
    task_adc_action_handler();
}

void task_adc_post_running(void)
{
    OTMS(TASK_MODULE_ADC, OTMS_S_ASSERT_RUN);
}

void task_adc_stop_running(void)
{
    LOG_LEVEL("_stop_running\r\n");
    OTMS(TASK_MODULE_ADC, OTMS_S_INVALID);
}

static void task_adc_action_handler(void)
{
    if (GetTickCounter(&l_t_msg_wait_timer) >= 10)
    {
        task_adc_key_polling_event_status(OCT_ADC_CHANNEL_MAX);
        task_adc_key_polling_event_dispatcher();
    }
}

/**
 * @brief  Checks whether a given ADC key is currently pressed.
 * @param  adc_key: Pointer to the ADC_KEY_STATUS structure representing the key.
 * @param  channel: ADC channel number from which the key value was read.
 * @param  adc_key_value: The raw ADC value read from the channel.
 * @retval true if the ADC value matches the key threshold; false otherwise.
 */
bool adc_key_is_valid(const ADC_KEY_STATUS *adc_key, adc_channel_t channel, uint16_t adc_key_value)
{
    // Validate ADC value within tolerance (+/-5)
    // and check if the key belongs to the current channel
    if ((adc_key_value <= (adc_key->value + 5)) &&
        (adc_key_value >= (adc_key->value - 5)) &&
        (adc_key->channel == channel))
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @brief  Scan all ADC keys and check for presses on a specific channel or all channels.
 * @param  channel: ADC channel to scan. Use OCT_ADC_CHANNEL_MAX to scan all channels.
 * @retval None
 * @note   This function supports multi-channel ADC key scanning and uses a
 *         NULL-terminated key array for iteration.
 */
void task_adc_key_polling_event_status(adc_channel_t channel)
{
    // Iterate through all ADC channels
    for (uint8_t ch = 0; ch < ADC_CHANNEL_NUM; ch++)
    {
        // If a specific channel is requested, skip other channels
        if (channel != OCT_ADC_CHANNEL_MAX && ch != channel)
            continue;

        uint16_t adc_key_value = adc_channel_value[ch];
        if (adc_key_value == 0xFFFF)
            continue;

        // Iterate through the ADC key table
        for (int i = 0; adc_key_array[i] != NULL; i++)
        {
            ADC_KEY_STATUS *adc_key = adc_key_array[i];

            // Skip keys that are marked as NONE (unused)
            if (adc_key->key_s == OCT_KEY_NONE && adc_key->key_l == OCT_KEY_NONE)
                continue;

            // Check if the current key is pressed
            if (adc_key_is_valid(adc_key, (adc_channel_t)ch, adc_key_value))
            {

                // TODO: Process key event here
                // e.g., update key state machine, send event to MMI
                // Increment the press counter with redundancy protection
                if (!IsTickCounterStart(&adc_key->status.start_tick_count))
                    StartTickCounter(&adc_key->status.start_tick_count);
                adc_key->status.press_duration = GetTickCounter(&adc_key->status.start_tick_count);

                // If the press duration exceeds the defined short-press period
                if (adc_key->status.press_duration > GPIO_KEY_STATUS_PRESS_PERIOD && !adc_key->status.pressed)
                {
                    adc_key->status.pressed = true;
                    adc_key->status.release = false;
                    adc_key->status.dispatched = false;
                    adc_key->status.ignore = false;
                    adc_key->status.state = KEY_STATE_PRESSED;
                }

                // If the press duration exceeds the defined long-press period
                if (adc_key->status.press_duration > GPIO_KEY_STATUS_LONG_PRESS_PERIOD && adc_key->status.pressed)
                {
                    adc_key->status.pressed = true;
                    adc_key->status.release = false;
                    adc_key->status.dispatched = false;
                    adc_key->status.state = KEY_STATE_LONG_PRESSED;
                }

                // If the press duration exceeds the defined long-long-press period
                if (adc_key->status.press_duration > GPIO_KEY_STATUS_LONG_LONG_PRESS_PERIOD && adc_key->status.pressed)
                {
                    adc_key->status.pressed = true;
                    adc_key->status.release = false;
                    adc_key->status.dispatched = false;
                    adc_key->status.state = KEY_STATE_LONG_LONG_PRESSED;
                }

                if (adc_key->status.press_duration > GPIO_KEY_STATUS_LONG_LONG_LONG_PRESS_PERIOD && adc_key->status.pressed)
                {
                    adc_key->status.pressed = true;
                    adc_key->status.release = false;
                    adc_key->status.dispatched = false;
                    adc_key->status.state = KEY_STATE_LONG_LONG_LONG_PRESSED;
                }
            }
            else
            {
                // If the key is released (GPIO pin is high)
                if (adc_key->status.pressed)
                {
                    adc_key->status.pressed = false;
                    adc_key->status.dispatched = false;
                    adc_key->status.release = true;
                    adc_key->status.state = KEY_STATE_RELEASED;
                }

                // Reset the pressed status and the counter
                adc_key->status.pressed = false;
                StopTickCounter(&adc_key->status.start_tick_count);
            }
        }
        adc_channel_value[ch] = 0xFFFF;
    }
}

void task_adc_key_polling_event_dispatcher(void)
{
    for (int i = 0; adc_key_array[i] != NULL; i++)
    {
        ADC_KEY_STATUS *adc_key = adc_key_array[i];
        if (adc_key->key_s == OCT_KEY_NONE && adc_key->key_l == OCT_KEY_NONE)
            return; //|| key_status->pin == 0
        // Check if the event has not been dispatched already or is ignored by user
        if (!adc_key->status.dispatched) //&& !key_status->ignore
        {
            // If the key is in the "pressed" state, send a "key down" event
            if (adc_key->status.pressed && !adc_key->status.ignore)
            {
                /**
                 * TASK_MODULE_KEY            - Identifier for the task handling key events.
                 * MSG_DEVICE_KEY_DOWN_EVENT - Message type indicating a key press.
                 * key_status->key        - The key identifier.
                 * KEY_STATE_PRESSED      - The current state of the key.
                 */

                send_message(TASK_MODULE_KEY, MSG_OTSM_DEVICE_KEY_DOWN_EVENT, adc_key->key_s, KEY_STATE_PRESSED);

                // Mark the event as dispatched to prevent duplicate messages
                adc_key->status.dispatched = true;
            }
            // If the key is in the "release" state, send a "key up" event
            else if (adc_key->status.release)
            {
                /**
                 * TASK_MODULE_KEY            - Identifier for the task handling key events.
                 * MSG_DEVICE_KEY_UP_EVENT - Message type indicating a key release.
                 * key_status->key        - The key identifier.
                 * KEY_STATE_RELEASED     - The current state of the key.
                 */
                send_message(TASK_MODULE_KEY, MSG_OTSM_DEVICE_KEY_UP_EVENT, adc_key->key_s, KEY_STATE_RELEASED);

                // Mark the event as dispatched to prevent duplicate messages
                adc_key->status.dispatched = true;
            }
        }
    }
}
