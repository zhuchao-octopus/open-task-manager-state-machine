
/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "octopus_flash.h"
#include "octopus_key.h"
#include "octopus_gpio.h" // Include GPIO HAL for hardware-specific functionality
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
#include "octopus_carinfor.h"
#endif
/*******************************************************************************
 * DEBUG SWITCH MACROS
 */

#define ADDR_OTA_FLAG 0x1FFF18FC

#ifdef TASK_MANAGER_STATE_MACHINE_KEY
/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
uint8_t get_dummy_key(uint8_t key);
/*******************************************************************************
 * GLOBAL VARIABLES
 */
uint8_t power_key_password[] = {OCTOPUS_KEY_POWER, OCTOPUS_KEY_POWER, OCTOPUS_KEY_POWER};
uint8_t power_key_password_index = 0;

static uint32_t l_t_msg_wait_timer;
static uint32_t l_t_msg_boot_wait_timer;

static bool key_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool key_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

static void app_do_key_action_hanlder(void);

void KeySendKeyCodeEvent(uint8_t key_code, uint8_t key_state);
void app_goto_bootloader(void);
void app_power_key_event_process(GPIO_KEY_STATUS *key_status);
/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void app_key_init_running(void)
{
    LOG_LEVEL("app_key_init_running\r\n");
    ptl_register_module(MCU_TO_SOC_MOD_KEY, key_send_handler, key_receive_handler);
    OTMS(TASK_ID_KEY, OTMS_S_INVALID);
}

void app_key_start_running(void)
{
    LOG_LEVEL("app_key_start_running\r\n");
    OTMS(TASK_ID_KEY, OTMS_S_ASSERT_RUN);
}

void app_key_assert_running(void)
{
    ptl_reqest_running(MCU_TO_SOC_MOD_KEY);

#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    StartTickCounter(&l_t_msg_wait_timer);
    StartTickCounter(&l_t_msg_boot_wait_timer);
    OTMS(TASK_ID_KEY, OTMS_S_RUNNING);
#endif
}

void app_key_running(void)
{
    app_do_key_action_hanlder();
}

void app_key_post_running(void)
{
}

void app_key_stop_running(void)
{
    OTMS(TASK_ID_KEY, OTMS_S_INVALID);
}

static void app_do_key_action_hanlder(void)
{
    if (GetTickCounter(&l_t_msg_wait_timer) < 10)
        return;
    StartTickCounter(&l_t_msg_wait_timer);

    Msg_t *msg = get_message(TASK_ID_KEY);

    if (msg->msg_id != NO_MSG && msg->msg_id == MSG_OTSM_DEVICE_KEY_DOWN_EVENT)
    {
        uint8_t key = msg->param1; // get_dummy_key(msg->param1);
        GPIO_KEY_STATUS *key_status = get_key_status_by_key(key);

        // LOG_LEVEL("key pressed key=%d key_status=%d\r\n",key,msg->param2);
        switch (key)
        {
        case OCTOPUS_KEY_POWER:
            app_power_key_event_process(key_status);
            break;
        default:
            break;
        }
    }

    else if (msg->msg_id != NO_MSG && msg->msg_id == MSG_OTSM_DEVICE_KEY_UP_EVENT)
    {
        uint8_t key = msg->param1; // get_dummy_key(msg->param1);
        GPIO_KEY_STATUS *key_status = get_key_status_by_key(key);
        // LOG_LEVEL("key release key=%d key_status=%d\r\n", key, msg->param2);
        switch (key)
        {
        case OCTOPUS_KEY_POWER:
            app_power_key_event_process(key_status);
            break;
        default:
            break;
        }
    }
}

void app_goto_bootloader(void)
{

    LOG_LEVEL("reboot to dul ota to upgrade mcu ble sw.\r\n");
#if 0
	write_reg(ADDR_OTA_FLAG,0x55AAAA55);
	GAPRole_TerminateConnection();
	WaitMs(500);
	NVIC_SystemReset();
#endif
}

void app_power_key_event_process(GPIO_KEY_STATUS *key_status)
{
    static uint32_t power_key_wait_timer;
    if (key_status->key != OCTOPUS_KEY_POWER)
        return;

    switch (key_status->state)
    {
    case KEY_STATE_RELEASED:
        LOG_LEVEL("OCTOPUS_KEY_POWER release key=%d key_status=%02x\r\n", key_status->key, key_status->state);
        // power_key_password_index=0;
        StopTickCounter(&power_key_wait_timer);
        break;

    case KEY_STATE_PRESSED:
        LOG_LEVEL("OCTOPUS_KEY_POWER pressed key=%d key_status=%02x\r\n", key_status->key, key_status->state);
        if ((IsTickCounterStart(&power_key_wait_timer)) && GetTickCounter(&power_key_wait_timer) >= 1000)
        {
            power_key_password_index = 0;
        }

        RestartTickCounter(&power_key_wait_timer);
        power_key_password_index++;
        hal_gpio_write(GPIO_POWER_ENABLE_GROUP, GPIO_POWER_ENABLE_PIN, BIT_SET); // prepare to power
        if (power_key_password_index == sizeof(power_key_password))
        {
            send_message(TASK_ID_SYSTEM, MSG_OTSM_DEVICE_BLE_EVENT, MSG_OTSM_CMD_BLE_PAIR_ON, 0);
            power_key_password_index = 0;
            StopTickCounter(&power_key_wait_timer);
        }
        break;
    case KEY_STATE_LONG_PRESSED:
        if (!key_status->ignore)
        {
            LOG_LEVEL("OCTOPUS_KEY_POWER long pressed key=%d duration=%d\r\n", key_status->key, key_status->state, key_status->press_duration);
            // if(is_power_on())
            //	hal_gpio_write(GPIO_POWER_SWITCH_GROUP, GPIO_POWER_SWITCH_PIN, BIT_SET);
            send_message(TASK_ID_SYSTEM, MSG_OTSM_DEVICE_POWER_EVENT, 0, 0);
            key_status->ignore = true;
        }
        break;

    case KEY_STATE_LONG_LONG_PRESSED:
        if (!key_status->ignore)
        {
        }
        break;

    case KEY_STATE_LONG_LONG_LONG_PRESSED:
        if (!key_status->ignore)
        {
        }
        break;
    case KEY_STATE_NONE:
    default:
        power_key_password_index = 0;
        break;
    }
}
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

uint8_t get_dummy_key(uint8_t key)
{
    switch (key)
    {
    case 0:
        return OCTOPUS_KEY_0;
    case 1:
        return OCTOPUS_KEY_1;
    case 2:
        return OCTOPUS_KEY_2;
    case 3:
        return OCTOPUS_KEY_3;

    case 14:
        return OCTOPUS_KEY_14;
    }
    return 0;
}

bool key_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    MY_ASSERT(buff);
    uint8_t tmp[8] = {0};

    if (MCU_TO_SOC_MOD_KEY == frame_type)
    {
        switch (param1)
        {
        case CMD_MODSETUP_KEY:         //
            tmp[0] = MSB_WORD(param2); // KEYCODE
            tmp[1] = LSB_WORD(param2); // KEYSTATE
            tmp[2] = 0;                //
            LOG_LEVEL("CMD_MODSETUP_KEY key %02x state %02x\n", tmp[0], tmp[1]);
            ptl_build_frame(MCU_TO_SOC_MOD_KEY, CMD_MODSETUP_KEY, tmp, 3, buff);
            return true;
        default:
            break;
        }
    }
    return false;
}

bool key_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    MY_ASSERT(payload);
    MY_ASSERT(ackbuff);
    uint8_t tmp;

    if (MCU_TO_SOC_MOD_KEY == payload->frame_type)
    {
        switch (payload->frame_cmd)
        {
        case CMD_MODSETUP_UPDATE_TIME:
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_SETUP, CMD_MODSETUP_UPDATE_TIME, &tmp, 1, ackbuff);
            return true;
        case CMD_MODSETUP_SET_TIME:
            // ACK, no thing to do
            return false;
        case CMD_MODSETUP_KEY:
        {
            uint8_t code = payload->data[0];
            uint8_t state = payload->data[1];
            LOG_LEVEL("CMD_MODSETUP_KEY key %02x state %02x\r\n", code, state);

            tmp = 0x01;
            /// ptl_build_frame(SOC_TO_MCU_MOD_SETUP, CMD_MODSETUP_KEY, &tmp, 1, ackbuff);
            return true;
        }
        default:
            break;
        }
    }
    return false;
}

#endif
