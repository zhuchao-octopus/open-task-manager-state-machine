/*******************************************************************************
 * @file     octopus_system.c
 * @brief    Implements the system control logic for managing states,
 *           message handling, and UART communication in an embedded application.
 *
 * This source file is responsible for initializing and controlling the system's
 * state machine. It handles incoming and outgoing messages through a UART
 * interface, processes system-level events, and manages the power state
 * and handshake procedures with both the MCU and external applications.
 *
 * The code uses a modular design to interface with other system components,
 * ensuring flexibility and scalability.
 *
 * @version  1.0.0
 * @date     2024-12-11
 * @author   Octopus Team
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "octopus_gpio.h"
#include "octopus_system.h"
#include "octopus_flash.h"
#include "octopus_uart_hal.h"
/*******************************************************************************
 * Debug Switch Macros
 * Define debug levels or other switches as required.
 ******************************************************************************/

/*******************************************************************************
 * MACROS
 * The following macros define key IDs and their respective actions.
 */

/*******************************************************************************
 * Local Function Declarations
 * Declare static functions used only within this file.
 ******************************************************************************/
 #ifdef TASK_MANAGER_STATE_MACHINE_SYSTEM
 
static bool system_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool system_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

static void system_power_manager(void);
static void system_backlight_manager(void);
static void system_voltage_manager(void);
	
void system_power_init(void);
void system_power_onoff_host(bool onoff);
void system_power_onoff_peripheral(bool onoff);
void system_power_onoff_auto(void);

void system_enter_standby(void);
void system_enter_power_on(void);
void system_enter_power_off(void);
void system_enter_acc_mode(uint16_t acc_mode);
void system_notify_host_standby(void);
void system_wakeup_host(void);
/*******************************************************************************
 * Global Variables
 * Define variables accessible across multiple files if needed.
 ******************************************************************************/

/*******************************************************************************
 * Local Variables
 * Define static variables used only within this file.
 ******************************************************************************/
static uint32_t l_t_msg_wait_10_timer; // Timer for 10 ms message waiting period

system_state_t system_state;
user_mata_infor_t user_mata_infor;
/*******************************************************************************
 * Global Function Implementations
 ******************************************************************************/

/**
 * @brief Initializes the system for running.
 *
 * This function registers the system module with the communication layer
 * and transitions the system task to an invalid state.
 */
void task_system_init_running(void)
{
    LOG_LEVEL("task_system_init_running\r\n");
    OTMS(TASK_MODULE_SYSTEM, OTMS_S_INVALID);

    // ptl_register_module(P2M_MOD_DEBUG, debug_send_handler, debug_receive_handler);
#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    ptl_register_module(MCU_TO_SOC_MOD_SYSTEM, system_send_handler, system_receive_handler);
#elif defined(TASK_MANAGER_STATE_MACHINE_SOC)
    ptl_register_module(SOC_TO_MCU_MOD_SYSTEM, system_send_handler, system_receive_handler);
#else
    ptl_register_module(MCU_TO_SOC_MOD_SYSTEM, system_send_handler, system_receive_handler);
#endif
}

/**
 * @brief Starts the system.
 *
 * This function transitions the system task to the "assert run" state.
 */
void task_system_start_running(void)
{
    LOG_LEVEL("task_system_start_running\r\n");
    OTMS(TASK_MODULE_SYSTEM, OTMS_S_ASSERT_RUN);
}

/**
 * @brief Asserts the system is running.
 *
 * This function starts timers, requests the system to start running,
 * and transitions the system task to the running state.
 */
void task_system_assert_running(void)
{
    StartTickCounter(&l_t_msg_wait_10_timer);

#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    ptl_reqest_running(MCU_TO_SOC_MOD_SYSTEM);
#elif defined(TASK_MANAGER_STATE_MACHINE_SOC)
    ptl_reqest_running(SOC_TO_MCU_MOD_SYSTEM);
#endif

    OTMS(TASK_MODULE_SYSTEM, OTMS_S_RUNNING);
}

/**
 * @brief Main system running state handler.
 *
 * Processes system messages and handles events like power on/off and device events.
 */
void task_system_running(void)
{
	  Msg_t *msg = get_message(TASK_MODULE_SYSTEM);
	
    switch (msg->msg_id)
    {
		case MSG_OTSM_DEVICE_POWER_EVENT:
				
				LOG_LEVEL("Got Event MSG_DEVICE_POWER_EVENT\r\n");
		
				if (msg->param1 == FRAME_CMD_SYSTEM_POWER_ON)
						system_enter_power_on();
				
				if (msg->param1 == FRAME_CMD_SYSTEM_POWER_OFF)
						system_enter_standby();	
				
				break;
					
		case MSG_OTSM_DEVICE_ACC_EVENT:
					system_enter_acc_mode(msg->param1);	
					break;					

    case MSG_OTSM_DEVICE_BLE_EVENT:
        if (msg->param1 == MSG_OTSM_CMD_BLE_PAIR_ON)
        {
            LOG_LEVEL("MSG_OTSM_DEVICE_BLE_EVENT notify ble to enable pair mode\r\n");
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_SYSTEM, msg->param1, msg->param2);
        }
        else
        {
            send_message(TASK_MODULE_PTL_1, SOC_TO_MCU_MOD_SYSTEM, msg->param1, msg->param2);
        }
        break;
    }
		system_power_manager();
		system_backlight_manager();
		system_voltage_manager();
}

void task_system_post_running(void)
{
    OTMS(TASK_MODULE_SYSTEM, OTMS_S_ASSERT_RUN);
}

void task_system_stop_running(void)
{
    LOG_LEVEL("_stop_running\r\n");
    OTMS(TASK_MODULE_SYSTEM, OTMS_S_INVALID);
}

/*******************************************************************************
 * FUNCTION: system_send_handler
 *
 * DESCRIPTION:
 * Handles the sending of system-related commands based on the frame type and parameters.
 * This function processes commands for system, setup, and app modules and sends appropriate responses.
 *
 * PARAMETERS:
 * - frame_type: Type of the frame (MCU_TO_SOC_MOD_SYSTEM, SOC_TO_MCU_MOD_SYSTEM, MCU_TO_SOC_MOD_SETUP).
 * - param1: Command identifier.
 * - param2: Additional parameter for the command.
 * - buff: Pointer to the buffer where the frame should be built.
 *
 * RETURNS:
 * - true if the command was processed successfully, false otherwise.
 ******************************************************************************/
bool system_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    MY_ASSERT(buff);      // Ensure the buffer is valid
    uint8_t tmp[8] = {0}; // Temporary buffer for command parameters

    // Handle commands for MCU_TO_SOC_MOD_SYSTEM frame type
    if (MCU_TO_SOC_MOD_SYSTEM == frame_type)
    {
        switch (param1)
        {
        case FRAME_CMD_SYSTEM_HANDSHAKE:
            tmp[0] = 0;
            tmp[1] = 0;
            LOG_LEVEL("system handshake frame_type=%02x param1=%02x param2=%02x\n", frame_type, tmp[0], tmp[1]);
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, FRAME_CMD_SYSTEM_HANDSHAKE, tmp, 2, buff);
            return true;

        case FRAME_CMD_SYSTEM_POWER_ON:
            // Acknowledgement, no additional action needed
            tmp[0] = 0;
            tmp[1] = 0;
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, FRAME_CMD_SYSTEM_POWER_ON, tmp, 2, buff);
            return true;

        case FRAME_CMD_SYSTEM_POWER_OFF:
            // Acknowledgement, no additional action needed
            tmp[0] = 0;
            tmp[1] = 0;
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, FRAME_CMD_SYSTEM_POWER_OFF, tmp, 2, buff);
            return true;

        case MSG_OTSM_CMD_BLE_PAIR_ON:
            tmp[0] = 0;
            tmp[1] = 0;
            LOG_LEVEL("MSG_OTSM_CMD_BLE_PAIR_ON \r\n");
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, (ptl_frame_cmd_t)MSG_OTSM_CMD_BLE_PAIR_ON, tmp, 2, buff);
            //hal_com_uart_send_buffer_3(buff->buff, buff->size);
            return false;

        default:
            break;
        }
    }
    // Handle commands for SOC_TO_MCU_MOD_SYSTEM frame type
    else if (SOC_TO_MCU_MOD_SYSTEM == frame_type)
    {
        switch (param1)
        {
        case FRAME_CMD_SYSTEM_HANDSHAKE:
            tmp[0] = 0;
            tmp[1] = 0;
            LOG_LEVEL("system handshake param1=%02x param2=%02x\n", tmp[0], tmp[1]);
            ptl_build_frame(SOC_TO_MCU_MOD_SYSTEM, FRAME_CMD_SYSTEM_HANDSHAKE, tmp, 2, buff);
            return true;

        case FRAME_CMD_SYSTEM_MCU_META:
            tmp[0] = 0;
            tmp[1] = 0;
            ptl_build_frame(SOC_TO_MCU_MOD_SYSTEM, FRAME_CMD_SYSTEM_MCU_META, tmp, 2, buff);
            return true;

        case MSG_OTSM_CMD_BLE_CONNECTED:
        case MSG_OTSM_CMD_BLE_DISCONNECTED:
            tmp[0] = param1; // Send MPU status
            tmp[1] = param2; // Additional status byte
            ptl_build_frame(SOC_TO_MCU_MOD_SYSTEM, (ptl_frame_cmd_t)param1, tmp, 2, buff);
            return true;
        default:
            break;
        }
    }
    return false; // Command not processed
}

/*******************************************************************************
 * FUNCTION: system_receive_handler
 *
 * DESCRIPTION:
 * Handles the reception of system-related commands based on the payload and sends appropriate responses.
 *
 * PARAMETERS:
 * - payload: Pointer to the received payload data.
 * - ackbuff: Pointer to the buffer where the acknowledgment frame will be built.
 *
 * RETURNS:
 * - true if the command was processed successfully, false otherwise.
 ******************************************************************************/
bool system_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    MY_ASSERT(payload); // Ensure payload is valid
    MY_ASSERT(ackbuff); // Ensure acknowledgment buffer is valid
    uint8_t tmp;        // Temporary variable for holding command data

    // Handle received commands for MCU_TO_SOC_MOD_SYSTEM frame type
    if (MCU_TO_SOC_MOD_SYSTEM == payload->frame_type)
    {
        switch (payload->frame_cmd)
        {
        case FRAME_CMD_SYSTEM_HANDSHAKE:
            LOG_LEVEL("system handshake from mcu payload->frame_type=%02x\r\n", payload->frame_type);
            return false;
        case FRAME_CMD_SYSTEM_ACC_STATE:
            LOG_LEVEL("FRAME_CMD_SYSTEM_ACC_STATE\r\n");
            return false;

        case FRAME_CMD_SYSTEM_MCU_META:
            if (payload->data_len == sizeof(flash_meta_infor_t))
            {
                memcpy(&flash_meta_infor, payload->data, sizeof(flash_meta_infor_t));
                LOG_LEVEL("FRAME_CMD_SYSTEM_MCU_META successfully %d / %d\r\n", payload->data_len, sizeof(flash_meta_infor_t));
                send_message(TASK_MODULE_IPC, MSG_OTSM_DEVICE_MCU_EVENT, MSG_OTSM_CMD_MCU_VERSION, 0);
            }
            else
            {
                LOG_LEVEL("FRAME_CMD_SYSTEM_MCU_META failed %d / %d\r\n", payload->data_len, sizeof(flash_meta_infor_t));
            }
            return false;

        case FRAME_CMD_SYSTEM_POWER_ON:
            LOG_LEVEL("got FRAME_CMD_SYSTEM_POWER_ON from mcu\r\n");
            system_enter_power_on();
            return false;
        case FRAME_CMD_SYSTEM_POWER_OFF:
            LOG_LEVEL("got FRAME_CMD_SYSTEM_POWER_OFF from mcu\r\n");
            system_enter_power_off();
            return false;

        case FRAME_CMD_SETUP_KEY:
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_SETUP, FRAME_CMD_SETUP_KEY, &tmp, 1, ackbuff);
            return false;

        case MSG_OTSM_CMD_BLE_PAIR_ON:
        case MSG_OTSM_CMD_BLE_PAIR_OFF:
            send_message(TASK_MODULE_BLE, MSG_OTSM_DEVICE_BLE_EVENT, MSG_OTSM_CMD_BLE_PAIR_ON, 0);
            return false;
        default:
            break;
        }
    }

    if (SOC_TO_MCU_MOD_SYSTEM == payload->frame_type)
    {
        switch (payload->frame_cmd)
        {
        case FRAME_CMD_SYSTEM_HANDSHAKE:
            LOG_LEVEL("system handshake from soc payload->frame_type=%02x\r\n", payload->frame_type);
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_INDICATOR, 0); // after got handshake then send indicate respond
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, FRAME_CMD_SYSTEM_MCU_META, (uint8_t *)(&flash_meta_infor), sizeof(flash_meta_infor_t), ackbuff);
            return true;
        case FRAME_CMD_SYSTEM_MCU_META:
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, FRAME_CMD_SYSTEM_MCU_META, (uint8_t *)(&flash_meta_infor), sizeof(flash_meta_infor_t), ackbuff);
            return true;
				
        case MSG_OTSM_CMD_BLE_CONNECTED:
            if (!gpio_is_power_on())
            {
                LOG_LEVEL("system got MSG_OTSM_CMD_BLE_CONNECTED prameter=%02x\r\n", payload->data[0]);
                system_enter_power_on();
            }
            break;
        case MSG_OTSM_CMD_BLE_DISCONNECTED:

            LOG_LEVEL("system got MSG_OTSM_CMD_BLE_DISCONNECTED prameter=%02x\r\n", payload->data[1]);
            if (payload->data[1] == FRAME_CMD_SYSTEM_POWER_OFF)
            {
                system_enter_power_off();
            }
            break;
        default:
            break;
        }
    }

    return false; // Command not processed
}

void system_handshake_with_mcu(void)
{
    LOG_LEVEL("system send handshake data to xxx\r\n");
    send_message(TASK_MODULE_PTL_1, SOC_TO_MCU_MOD_SYSTEM, FRAME_CMD_SYSTEM_HANDSHAKE, 0);
}

void system_handshake_with_app(void)
{
    LOG_LEVEL("system send handshake data to xxx\r\n");
    send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_SYSTEM, FRAME_CMD_SYSTEM_HANDSHAKE, 0);
}

static void system_power_manager(void)
{
		switch (system_state.power_state)
		{
				case POWER_STATE_START_INIT:
						system_state.power_state = POWER_STATE_BOOTING;
						break;

				case POWER_STATE_BOOTING:
						system_state.power_state = POWER_STATE_POWER_ON;				    
						break;
				
				case POWER_STATE_ACC_ON:
            system_state.power_state = POWER_STATE_POWER_ON;	   
            StopTickCounter(&system_state.acc_wait_time);				
					  break;
				
				case POWER_STATE_ACC_OFF:	
			      StartTickCounter(&system_state.acc_wait_time);
				    system_state.power_state = POWER_STATE_ACC_OFF_WAITING; 
					  break;

				case POWER_STATE_ACC_OFF_WAITING:
						if(GetTickCounter(&system_state.acc_wait_time) < user_mata_infor.acc_wait_timeout)
							break;
						
						system_power_onoff_peripheral(false);
						system_notify_host_standby();
						StartTickCounter(&system_state.host_wait_time);
						system_state.power_state = POWER_STATE_ACC_OFF_WAITING_HOST; 
						break;
				
				case POWER_STATE_ACC_OFF_WAITING_HOST://waiting host enter standby mode
						if(GetTickCounter(&system_state.host_wait_time) >= 1000*30)//30s
						{
							system_state.force_kill_host = true;
							system_state.power_state = POWER_STATE_POWER_OFF;
						}
						break;
						
				case POWER_STATE_ENTER_STANDBY:
						system_notify_host_standby();
						system_state.power_state = POWER_STATE_ACC_OFF_WAITING_HOST; 
						break;
				
				case POWER_STATE_ENTER_LOWPOWER:// mcu enter stanby mode
					
					  break;
				
				///////////////////////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////////////////////
				///////////////////////////////////////////////////////////////////////////////////////////////
				case POWER_STATE_POWER_ON:
					  system_power_onoff_host(true);
				    system_power_onoff_peripheral(true);
				    system_wakeup_host();
				    system_state.power_state = POWER_STATE_NORMAL_RUNNING; 
						break;

				case POWER_STATE_POWER_OFF:
					  system_power_onoff_peripheral(false);
				
					  if(system_state.force_kill_host)
						{
							system_power_onoff_host(false); //shutdown the host
							system_state.power_state = POWER_STATE_SHUTDOWN; 
						}
						else
						{
						  system_state.power_state = POWER_STATE_STANDBY; 	
						}
						
						break;
								
        case POWER_STATE_NORMAL_RUNNING:
					
					  break;
				
				case POWER_STATE_STANDBY:
					
						break;
				
				case POWER_STATE_SHUTDOWN:
					
						break;
				default:
						// Optional: Handle unknown state
						break;
		}
}

static void system_backlight_manager(void)
{
	
}

static void system_voltage_manager(void)
{
	uint16_t ADC_current;	
	static u16 s_volt_det_timer = 0;
	static u8 s_low_volt_cntr = 0;

	if (s_volt_det_timer < 60000) {
		++s_volt_det_timer;
	}
	if (s_low_volt_cntr < 250) {
		++s_low_volt_cntr;
	}

	ADC_current = adc_channel_sample(AD_BATT_DET);

	if (ADC_current<LOW_VOLT_PROTECT_OFF)
	{

		if(ADC_current < VOLT_6V)
		{
			if (ADC_current < VOLT_5V) {
				system_state.system_need_reset=1;
				//EmergencyPowerDown();
			}
		}
	}
	else if(ADC_current<LOW_VOLT_PROTECT_ON)
	{	
		s_volt_det_timer = 0;
		s_low_volt_cntr = 0;
	}
	else if(ADC_current<HIGH_VOLT_PROTECT_ON)
	{
		s_volt_det_timer = 0;
		s_low_volt_cntr = 0;
	}
	else if(ADC_current<HIGH_VOLT_PROTECT_OFF)
	{	
		s_volt_det_timer = 0;
		s_low_volt_cntr = 0;
	}
	else
	{
		s_low_volt_cntr = 0;
	}	
}

void system_power_manager_init(void)
{
	system_state.host_is_charging = false;
	system_state.host_is_sleeping = false;
	system_state.force_kill_host = false;
	system_state.acc_signal = 0;
	system_state.system_need_reset = false;
	system_state.power_state = POWER_STATE_START_INIT;
	StartTickCounter(&system_state.uptime_ms);
}

void system_wakeup_host(void)
{
	GPIO_SetBits(GPIO_HOST_PWR_KEY_GROUP, GPIO_HOST_PWR_KEY_PIN);
	delay_ms(400);
	GPIO_ResetBits(GPIO_HOST_PWR_KEY_GROUP, GPIO_HOST_PWR_KEY_PIN);	
}

void system_reboot_host(void)
{
  LOG_LEVEL("System is rebooting...\n");
}

void system_backlight_onoff(bool onoff)
{
	
}

void system_enter_power_on(void)
{
	system_state.power_state = POWER_STATE_POWER_ON;
}

void system_enter_power_off(void)
{
	system_state.power_state = POWER_STATE_POWER_OFF;
}

void system_enter_acc_mode(uint16_t acc_mode)
{
	system_state.acc_signal = acc_mode;
	if(acc_mode == MODULE_ON)
	{
		LOG_LEVEL("system_enter_acc_mode on\r\n");	
		system_state.power_state = POWER_STATE_ACC_ON;
	}
	else
	{
		LOG_LEVEL("system_enter_acc_mode off\r\n");	
		system_state.power_state = POWER_STATE_ACC_OFF;
	}	
}

void system_enter_standby(void)
{
	if(system_state.power_state == POWER_STATE_ACC_OFF_WAITING_HOST)
	{
		if(system_state.host_is_sleeping)
		  system_state.power_state = POWER_STATE_STANDBY;
	}
	else
	{
	  system_state.power_state = POWER_STATE_ENTER_STANDBY;
	}
}

void system_notify_host_standby(void)
{

}

void system_power_onoff_auto(void)
{
    if (gpio_is_power_on())
        system_power_onoff_host(false);
    else
        system_power_onoff_host(true);
}

void system_power_onoff_host(bool onoff)
{
    if (!onoff)
    {
      LOG_LEVEL("power down host\r\n");
			hal_gpio_write(GPIO_POWER_ENABLE_GROUP, GPIO_POWER_ENABLE_PIN, BIT_RESET);
			hal_gpio_write(GPIO_POWER_SWITCH_GROUP, GPIO_POWER_SWITCH_PIN, BIT_RESET);
      if (!gpio_is_power_on())
         LOG_LEVEL("power down host succesfully\r\n");
    }
    else
    {
      LOG_LEVEL("power on host\r\n");
			hal_gpio_write(GPIO_POWER_ENABLE_GROUP, GPIO_POWER_ENABLE_PIN, BIT_SET);
			hal_gpio_write(GPIO_POWER_SWITCH_GROUP, GPIO_POWER_SWITCH_PIN, BIT_SET);
      if (gpio_is_power_on())
         LOG_LEVEL("power on host succesfully\r\n");
    }
}

void system_power_onoff_peripheral(bool onoff)
{
    if (onoff)
    {
			hal_gpio_write(GPIO_GPS_ENABLE_GROUP, GPIO_GPS_ENABLE_PIN, BIT_SET);
			hal_gpio_write(GPIO_HDMI_ENABLE_GROUP, GPIO_HDMI_ENABLE_PIN, BIT_SET);
			hal_gpio_write(GPIO_TV_ENABLE_GROUP, GPIO_TV_ENABLE_PIN, BIT_SET);
			hal_gpio_write(GPIO_ANT_ENABLE_GROUP, GPIO_ANT_ENABLE_PIN, BIT_SET);
			hal_gpio_write(GPIO_LCD_ENABLE_GROUP, GPIO_LCD_ENABLE_PIN, BIT_SET);
    }
    else
    {
			hal_gpio_write(GPIO_GPS_ENABLE_GROUP, GPIO_GPS_ENABLE_PIN, BIT_RESET);
			hal_gpio_write(GPIO_HDMI_ENABLE_GROUP, GPIO_HDMI_ENABLE_PIN, BIT_RESET);
			hal_gpio_write(GPIO_TV_ENABLE_GROUP, GPIO_TV_ENABLE_PIN, BIT_RESET);
			hal_gpio_write(GPIO_ANT_ENABLE_GROUP, GPIO_ANT_ENABLE_PIN, BIT_RESET);
			hal_gpio_write(GPIO_LCD_ENABLE_GROUP, GPIO_LCD_ENABLE_PIN, BIT_RESET);
    }	
}

void system_enable_backlight(void)
{
	if (!GPIO_LCD_ENABLE_IS_ON()) {
		return;
	}
	hal_gpio_write(GPIO_LCD_ENABLE_GROUP, GPIO_LCD_ENABLE_PIN,BIT_SET);
} 

void system_disable_backlight(void)
{
	hal_gpio_write(GPIO_LCD_ENABLE_GROUP, GPIO_LCD_ENABLE_PIN,BIT_RESET);
}

mb_power_manager_state_t system_get_mb_state(void)
{
    return system_state.power_state;
}

void system_set_mb_state(mb_power_manager_state_t status)
{
    system_state.power_state = status;
}
#endif
