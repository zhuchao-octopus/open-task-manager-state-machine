
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
//#define ADDR_OTA_FLAG 0x1FFF18FC
#ifdef TASK_MANAGER_STATE_MACHINE_KEY

// encoder_handler_multi.c
// Multi-channel rotary encoder ADC handler with direction detection and debounce

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define ENCODER_TOLERANCE  3         // ADC tolerance for matching encoder levels
#define ENCODER_MAX_CNT    2         // Maximum number of supported encoders
#define MMI_MODULE         0x01      // Placeholder for MMI event module

// Placeholder for system-specific macros or functions
#define PostEvent(module, code, param)  // Event posting to MMI or equivalent
#define adc_channel_sample(ch)         (0)  // Dummy ADC read function
#define ABS(x)  ((x) > 0 ? (x) : -(x))
#define Get_ACC_Det_Flag 1
#define Is_Machine_Power 1

// Encoder decoder state structure
typedef struct {
    uint8_t level_table[4];       // 4-step voltage levels for encoder positions
    uint8_t keycode_cw;           // Keycode for clockwise rotation
    uint8_t keycode_ccw;          // Keycode for counter-clockwise rotation
    uint8_t last_position;        // Previous matched position index
    uint8_t last_key_code;        // Last key code issued
	  uint8_t debounce_timer;       // Debounce timer (units = 4ms or 10ms)
    uint8_t adc_channel;          // Associated ADC channel
    bool enabled;                 // Whether encoder is active
} EncoderDecoder;

/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
uint8_t power_key_password[] = {0};
uint8_t power_key_password_index = 0;

static uint32_t l_t_msg_wait_timer;
static uint32_t l_t_msg_boot_wait_timer;

static bool key_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool key_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);
static void task_key_action_hanlder(void);

void KeySendKeyCodeEvent(uint8_t key_code, uint8_t key_state);
void task_key_goto_bootloader(void);
void task_key_power_event_process(GPIO_KEY_STATUS *key_status);
/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void task_key_init_running(void)
{
    LOG_LEVEL("task_key_init_running\r\n");
    ptl_register_module(MCU_TO_SOC_MOD_KEY, key_send_handler, key_receive_handler);
    OTMS(TASK_MODULE_KEY, OTMS_S_INVALID);
}

void task_key_start_running(void)
{
    LOG_LEVEL("task_key_start_running\r\n");
    OTMS(TASK_MODULE_KEY, OTMS_S_ASSERT_RUN);
}

void task_key_assert_running(void)
{
    ptl_reqest_running(MCU_TO_SOC_MOD_KEY);

#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    StartTickCounter(&l_t_msg_wait_timer);
    StartTickCounter(&l_t_msg_boot_wait_timer);
    OTMS(TASK_MODULE_KEY, OTMS_S_RUNNING);
#endif
}

void task_key_running(void)
{
    task_key_action_hanlder();
}

void task_key_post_running(void)
{
	OTMS(TASK_MODULE_KEY, OTMS_S_ASSERT_RUN); 
}

void task_key_stop_running(void)
{
	LOG_LEVEL("_stop_running\r\n");
    OTMS(TASK_MODULE_KEY, OTMS_S_INVALID);
}

static void task_key_action_hanlder(void)
{
    if (GetTickCounter(&l_t_msg_wait_timer) < 10)
        return;
    StartTickCounter(&l_t_msg_wait_timer);

    Msg_t *msg = get_message(TASK_MODULE_KEY);

    if (msg->msg_id != NO_MSG && msg->msg_id == MSG_OTSM_DEVICE_KEY_DOWN_EVENT)
    {
        uint8_t key = msg->param1; // get_dummy_key(msg->param1);
        GPIO_KEY_STATUS *key_status = get_key_status_by_key(key);

        // LOG_LEVEL("key pressed key=%d key_status=%d\r\n",key,msg->param2);
        switch (key)
        {
        case MCU_KEY_POWER:
            task_key_power_event_process(key_status);
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
        case MCU_KEY_POWER:
            task_key_power_event_process(key_status);
            break;
        default:
            break;
        }
    }
}

void task_key_goto_bootloader(void)
{
  LOG_LEVEL("reboot to dul ota to upgrade mcu ble sw.\r\n");
#if 0
	write_reg(ADDR_OTA_FLAG,0x55AAAA55);
	GAPRole_TerminateConnection();
	WaitMs(500);
	NVIC_SystemReset();
#endif
}

void task_key_power_event_process(GPIO_KEY_STATUS *key_status)
{
    static uint32_t power_key_wait_timer;
    if (key_status->key != MCU_KEY_POWER)
        return;

    switch (key_status->state)
    {
    case KEY_STATE_RELEASED:
        LOG_LEVEL("OCTOPUS_KEY_POWER release key=%d key_status=%02x\r\n", key_status->key, key_status->state);
		StartTickCounter(&power_key_wait_timer);
        break;

    case KEY_STATE_PRESSED:
        LOG_LEVEL("OCTOPUS_KEY_POWER pressed key=%d key_status=%02x\r\n", key_status->key, key_status->state);
		   
        if (GetTickCounter(&power_key_wait_timer) >= 300)
        {
            power_key_password_index = 0;
        }
      
        power_key_password_index++;
        hal_gpio_write(GPIO_POWER_ENABLE_GROUP, GPIO_POWER_ENABLE_PIN, BIT_SET); // prepare to power
        if (power_key_password_index == sizeof(power_key_password))
        {
            send_message(TASK_MODULE_SYSTEM, MSG_OTSM_DEVICE_BLE_EVENT, MSG_OTSM_CMD_BLE_PAIR_ON, 0);
            power_key_password_index = 0;
            StopTickCounter(&power_key_wait_timer);
        }
        break;
    case KEY_STATE_LONG_PRESSED:
        if (!key_status->ignore)
        {
            LOG_LEVEL("OCTOPUS_KEY_POWER pressed key=%d long duration=%d\r\n", key_status->key, key_status->state, key_status->press_duration);
            // if(is_power_on())
            //	hal_gpio_write(GPIO_POWER_SWITCH_GROUP, GPIO_POWER_SWITCH_PIN, BIT_SET);
            send_message(TASK_MODULE_SYSTEM, MSG_OTSM_DEVICE_POWER_EVENT, 0, 0);
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

bool key_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    MY_ASSERT(buff);
    uint8_t tmp[8] = {0};

    if (MCU_TO_SOC_MOD_KEY == frame_type)
    {
        switch (param1)
        {
        case FRAME_CMD_SETUP_KEY:         //
            tmp[0] = MSB_WORD(param2); // KEYCODE
            tmp[1] = LSB_WORD(param2); // KEYSTATE
            tmp[2] = 0;                //
            LOG_LEVEL("FRAME_CMD_SETUP_KEY key %02x state %02x\n", tmp[0], tmp[1]);
            ptl_build_frame(MCU_TO_SOC_MOD_KEY, FRAME_CMD_SETUP_KEY, tmp, 3, buff);
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
        case FRAME_CMD_SETUP_UPDATE_TIME:
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_SETUP, FRAME_CMD_SETUP_UPDATE_TIME, &tmp, 1, ackbuff);
            return true;
        case FRAME_CMD_SETUP_SET_TIME:
            // ACK, no thing to do
            return false;
        case FRAME_CMD_SETUP_KEY:
        {
            uint8_t code = payload->data[0];
            uint8_t state = payload->data[1];
            LOG_LEVEL("FRAME_CMD_SETUP_KEY key %02x state %02x\r\n", code, state);

            tmp = 0x01;
            /// ptl_build_frame(SOC_TO_MCU_MOD_SETUP, FRAME_CMD_SETUP_KEY, &tmp, 1, ackbuff);
            return true;
        }
        default:
            break;
        }
    }
    return false;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Encoder instance array
static EncoderDecoder encoders[ENCODER_MAX_CNT] = {
    { // ENC1
        .level_table = {27, 40, 62, 254},
        .keycode_cw = MCU_KEY_UP,
        .keycode_ccw = MCU_KEY_DOWN,
        .adc_channel = 0,
        .enabled = true,
    },
    { // ENC2
        .level_table = {30, 44, 66, 250},
        .keycode_cw = MCU_KEY_UP,
        .keycode_ccw = MCU_KEY_DOWN,
        .adc_channel = 1,
        .enabled = true,
    },
};

// Match ADC value to closest predefined encoder level
static int key_match_encoder_level(uint8_t val, const uint8_t *level_table) {
    for (int i = 0; i < 4; i++) {
        if (ABS(val - level_table[i]) <= ENCODER_TOLERANCE)
            return i;
    }
    return -1;
}

// Decode the input signal of one encoder and issue key events
static void key_decode_encoder_signal(EncoderDecoder *dec) {
    if (!dec->enabled) return;

    uint8_t adc_val = adc_channel_sample(dec->adc_channel);
    int curr_index = key_match_encoder_level(adc_val, dec->level_table);

    if (curr_index < 0 || curr_index == dec->last_position)
        return;

    uint8_t keycode = 0;

    // Clockwise pattern (example for 4-step encoder)
    if ((dec->last_position == 0 && curr_index == 1) ||
        (dec->last_position == 1 && curr_index == 3) ||
        (dec->last_position == 3 && curr_index == 2) ||
        (dec->last_position == 2 && curr_index == 0)) {
        keycode = dec->keycode_cw;
    }
    // Counter-clockwise pattern
    else if ((dec->last_position == 0 && curr_index == 2) ||
             (dec->last_position == 2 && curr_index == 3) ||
             (dec->last_position == 3 && curr_index == 1) ||
             (dec->last_position == 1 && curr_index == 0)) {
        keycode = dec->keycode_ccw;
    }

    // Debounce + key reporting
    if (keycode) {
        if (dec->debounce_timer == 0 || dec->last_key_code == keycode) {
            PostEvent(MMI_MODULE, keycode, 0);
        }
        dec->debounce_timer = 8;  // ~32ms debounce
        dec->last_key_code = keycode;
    }

    dec->last_position = curr_index;
}

// Main polling function, called periodically to decode all encoders
void key_encoder_decoder_poll(void) {
    if (Get_ACC_Det_Flag == 0 || !Is_Machine_Power) 
		{
        for (int i = 0; i < ENCODER_MAX_CNT; i++) {
            encoders[i].debounce_timer = 0;
            encoders[i].last_key_code = 0;
            encoders[i].last_position = 0;
            encoders[i].enabled = false;
        }
        return;
    }

    for (int i = 0; i < ENCODER_MAX_CNT; i++) {
        if (encoders[i].enabled) {
            key_decode_encoder_signal(&encoders[i]);
        }
    }
}

// Called by timer interrupt every 4ms or 10ms to decrement debounce counters
void key_encoder_decoder_tick(void) {
    for (int i = 0; i < ENCODER_MAX_CNT; i++) 
	  {
        if (encoders[i].debounce_timer > 0)
            encoders[i].debounce_timer--;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const KEY_INFO g_panel_default_key[] = {

	{AD_PANEL_KEY_DET_2, 0,   MCU_KEY_FAKE_POWER_OFF, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 9,   MCU_KEY_BACK, MCU_KEY_HOME, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 17,  MCU_KEY_MUTE, MCU_KEY_TFT_STANDBY, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 25,  MCU_KEY_RADIO, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 36,  MCU_KEY_NEXT, MCU_KEY_FASTF, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 48,  MCU_KEY_PREV, MCU_KEY_FASTR, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 61,  MCU_KEY_RADIO_PS, MCU_KEY_AS, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 74,  MCU_KEY_DVD, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 91,  MCU_KEY_VOLUME_UP, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 110, MCU_KEY_VOLUME_DOWN, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 128, MCU_KEY_NAVI, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 147, MCU_KEY_ALL_APP, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 169, MCU_KEY_EQ, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 196, MCU_KEY_EJECT, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_2, 219, MCU_KEY_HOME, 0x00, KEY_STUDY_SWC_PU_LARGE},
	
	{AD_PANEL_KEY_DET_1, 0,   MCU_KEY_MUTE, MCU_KEY_FAKE_POWER_OFF, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 9,   MCU_KEY_EQ, MCU_KEY_FAKE_POWER_OFF, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 17,  MCU_KEY_LOUDNESS, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 25,  MCU_KEY_DIAL, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 36,  MCU_KEY_ST_PROG, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 48,  MCU_KEY_SOURCE, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 61,  MCU_KEY_MUTE, MCU_KEY_TFT_STANDBY, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 74,  MCU_KEY_NAVI, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 91,  MCU_KEY_TFT_STANDBY, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 110, MCU_KEY_DVD, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 128, MCU_KEY_RADIO, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 147, MCU_KEY_TV, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 169, MCU_KEY_PLAY_PAUSE, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 196, MCU_KEY_SETUP, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 219, MCU_KEY_HANG, 0x00, KEY_STUDY_SWC_PU_LARGE},
	{AD_PANEL_KEY_DET_1, 237, MCU_KEY_NISSAN_XTRAIL_CAM_SW, 0x00, KEY_STUDY_SWC_PU_LARGE},
};

KEY_INFO_STORE g_panel_key_store;


static void panel_key_set_default_value(void)
{
	u8 cnt;
	u8 len;
	len = sizeof(g_panel_default_key)/sizeof(KEY_INFO);

	for (cnt=0; cnt<len; cnt++) {
		if (g_panel_key_store.key_num >= MAX_PANEL_KEY_NUM) {
			break;
		}

		g_panel_key_store.key[g_panel_key_store.key_num].adc_channel = g_panel_default_key[cnt].adc_channel;
		g_panel_key_store.key[g_panel_key_store.key_num].adc_value = g_panel_default_key[cnt].adc_value;
		g_panel_key_store.key[g_panel_key_store.key_num].key_code_short = g_panel_default_key[cnt].key_code_short;
		g_panel_key_store.key[g_panel_key_store.key_num].key_code_long = g_panel_default_key[cnt].key_code_long;
		g_panel_key_store.key[g_panel_key_store.key_num].swc_pu_type = g_panel_default_key[cnt].swc_pu_type;

		++g_panel_key_store.key_num;
	}
}

#endif
