/*******************************************************************************
* @file		octopus_log.c
* @brief	Contains all functions support for uart driver

*
*******************************************************************************/
/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"
#include "octopus_log.h"
#include "octopus_gpio.h"
#include "octopus_system.h"
#include "octopus_timer.h"
#include "octopus_msgqueue.h"
#include "octopus_task_manager.h"
/*******************************************************************************
 * DEBUG SWITCH MACROS
*/

/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
void app_power_on_off(bool onoff);

 /*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */
static mb_state_t lt_mb_state;
//static uint8_t l_u8_mpu_handshake = 0;
static uint8_t l_u8_mpu_status = 0;
//static uint8_t l_u8_acc_status = 0;
static uint8_t l_u8_power_off_req = 0;
//static mb_state_t lt_mb_state = 0;
//static uint8_t l_u8_ver_str[64] = { 0 };

static uint32_t           l_t_msg_wait_10_timer;
//static uint32_t           l_t_msg_wait_50_timer;

static bool system_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool system_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void app_system_init_running(void)
{
    LOG_LEVEL("app_system_init\r\n");
    #ifdef TASK_MANAGER_STATE_MACHINE_MCU
    ptl_com_uart_register_module(M2A_MOD_SYSTEM, system_send_handler, system_receive_handler);
    #elif defined(TASK_MANAGER_STATE_MACHINE_SOC)
    ptl_com_uart_register_module(A2M_MOD_SYSTEM, system_send_handler, system_receive_handler);
    #endif
    OTMS(TASK_ID_SYSTEM, OTMS_S_INVALID);
}

void app_system_start_running(void)
{
    LOG_LEVEL("app_system_start\r\n");
    OTMS(TASK_ID_SYSTEM, OTMS_S_ASSERT_RUN);
}

void app_system_assert_running(void)
{
    StartTimer(&l_t_msg_wait_10_timer);
    //StartTimer(&l_t_msg_wait_50_timer);
    #ifdef TASK_MANAGER_STATE_MACHINE_MCU
	  com_uart_reqest_running(M2A_MOD_SYSTEM);
    #elif defined(TASK_MANAGER_STATE_MACHINE_SOC)
      com_uart_reqest_running(A2M_MOD_SYSTEM);
    #endif
    OTMS(TASK_ID_SYSTEM, OTMS_S_RUNNING);
}

void app_system_running(void)
{
   if (GetTimer(&l_t_msg_wait_10_timer) < 10)
        return;
    RestartTimer(&l_t_msg_wait_10_timer);
    //if (GetTimer(&l_t_msg_wait_50_timer) < 50)
    //    return;
    //RestartTimer(&l_t_msg_wait_50_timer);
		
		
    Msg_t* msg = get_message(TASK_ID_SYSTEM);	
    if(msg->id == NO_MSG) return;	
    
	 switch(msg->id)
    {
     case MSG_DEVICE_NORMAL_EVENT:
        //send_message(TASK_ID_PTL, M2A_MOD_METER , CMD_MODMETER_RPM_SPEED, 0);
        send_message(TASK_ID_PTL, M2A_MOD_INDICATOR , CMD_MODINDICATOR_INDICATOR, 0);
        //StartTimer(&l_t_msg_wait_50_timer);
        app_power_on_off(GPIO_PIN_READ_ACC());
        break;
     case MSG_DEVICE_POWER_EVENT:
        if(msg->param1 == CMD_MODSYSTEM_POWER_ON)
        {
        app_power_on_off(1);
        }
        else if(msg->param1 == CMD_MODSYSTEM_POWER_OFF)
        {
        app_power_on_off(0);
        }
       break;        
     
    }
	
}

void app_system_post_running(void)
{

}

void app_system_stop_running(void)
{
    OTMS(TASK_ID_SYSTEM, OTMS_S_INVALID);
}

void app_power_on_off(bool onoff)
{
 const char *power_state = onoff ? "on" : "off";  
 const char *acc_state = IsAccOn() ? "true" : "false"; 
 LOG_LEVEL("power status=%s,acc=%s\r\n", power_state, acc_state);

	if(onoff)
	{
		GPIO_ACC_SOC_HIGH();
	}
	else
	{
	  GPIO_ACC_SOC_LOW();	
	}	
}

bool system_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    assert(buff);
    uint8_t tmp[8] = {0};
		//LOG_LEVEL("frame_type=%02x d param1=%02x param2=%02x\n",frame_type,tmp[0],tmp[1]);
    if(M2A_MOD_SYSTEM == frame_type)
    {
        switch(param1)
        {
				case CMD_MODSYSTEM_HANDSHAKE:
            tmp[0] = 0x55;
            tmp[1] = 0xAA;
            LOG_LEVEL("system handshake param1=%02x param2=%02x\n",tmp[0],tmp[1]);
            ptl_com_uart_build_frame(A2M_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, tmp, 2, buff);
            return true;
				case CMD_MODSYSTEM_ACC_STATE:
            //ACK, no thing to do
            //tmp[0] = GetWakeupEvent(WAKEUP_EVENT_ACC);
            ptl_com_uart_build_frame(M2A_MOD_SYSTEM, CMD_MODSYSTEM_ACC_STATE, tmp, 1, buff);
            return true;
        case CMD_MODSYSTEM_POWER_OFF:
            //ACK, no thing to do
            tmp[0] = 0;
            tmp[1] = 0;
            ptl_com_uart_build_frame(M2A_MOD_SYSTEM, CMD_MODSYSTEM_POWER_OFF, tmp, 2, buff);
            return true;	
        case CMD_MODSETUP_UPDATE_TIME:
            //ACK, no thing to do
            tmp[0] = 23; //year
            tmp[1] = 3;  //month
            tmp[2] = 25; //day
            tmp[3] = 8;  //hour
            tmp[4] = 55; //minute
            tmp[5] = 0;  //second
            ptl_com_uart_build_frame(M2A_MOD_SETUP, CMD_MODSETUP_UPDATE_TIME, tmp, 6, buff);
            return true;
        case CMD_MODSETUP_KEY: //
            tmp[0] = MSB(param2); //KEYCODE
            tmp[1] = LSB(param2); //KEYSTATE
            tmp[2] = 0;  //
            LOG_LEVEL("CMD_MODSETUP_KEY  key %02x state %02x\n",tmp[0],tmp[1]);
            ptl_com_uart_build_frame(M2A_MOD_SETUP, CMD_MODSETUP_KEY, tmp, 3, buff);
            return true;
				
        default:
            break;
        }
    }
    else if(A2M_MOD_SYSTEM == frame_type)
    {
        switch(param1)
        {
        case CMD_MODSYSTEM_HANDSHAKE:
            tmp[0] = 0xAA;
            tmp[1] = 0x55;
            LOG_LEVEL("system handshake param1=%02x param2=%02x\n",tmp[0],tmp[1]);
            ptl_com_uart_build_frame(A2M_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, tmp, 2, buff);
            return true;
        case CMD_MODSYSTEM_APP_STATE:
            tmp[0] = l_u8_mpu_status;
            tmp[1] = 0x01;           
            ptl_com_uart_build_frame(A2M_MOD_SYSTEM, CMD_MODSYSTEM_APP_STATE, tmp, 2, buff);
            return true;
        default:
            break;
        }
    }
	else if(M2A_MOD_SETUP == frame_type)
    {
        switch(param1)
        {
        case CMD_MODSETUP_SET_TIME:
            return false;
        default:
            break;
        }
    }
    return false;
}

bool system_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    assert(payload);
    assert(ackbuff);
    uint8_t tmp;
	  ///uint8_t code;
	  ///uint8_t state;
    if(M2A_MOD_SYSTEM == payload->frame_type)
    {
        switch(payload->cmd)
        {
		case CMD_MODSYSTEM_HANDSHAKE:
            LOG_LEVEL("SYSTEM handshake Ok\r\n");
            ///l_u8_mpu_handshake = 1;
            ptl_com_uart_build_frame(M2A_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, (uint8_t*)VER_STR, sizeof(VER_STR), ackbuff);
            return true;
        case CMD_MODSYSTEM_ACC_STATE:
            //ACK, no thing to do
            LOG_LEVEL("CMD_MODSYSTEM_ACC_STATE\r\n");
            ///l_u8_mpu_handshake = 1;
            ptl_com_uart_build_frame(M2A_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, (uint8_t*)VER_STR, sizeof(VER_STR), ackbuff);
            return false;
        case CMD_MODSYSTEM_APP_STATE:
            ///l_u8_mpu_status = payload->data[0];
            LOG_LEVEL("CMD_MODSYSTEM_APP_STATE l_u8_mpu_status = %d\r\n",payload->data[0]);
            tmp = 0x01;
            ptl_com_uart_build_frame(M2A_MOD_SYSTEM, CMD_MODSYSTEM_APP_STATE, &tmp, 1, ackbuff);
            return true;
        case CMD_MODSYSTEM_POWER_OFF:
            //ACK, no thing to do
            return false;
				
        case CMD_MODSETUP_UPDATE_TIME: 
            tmp = 0x01;
            ptl_com_uart_build_frame(A2M_MOD_SETUP, CMD_MODSETUP_UPDATE_TIME, &tmp, 1, ackbuff);
            return false;
        case CMD_MODSETUP_SET_TIME: 
            tmp = 0x01;
            ptl_com_uart_build_frame(M2A_MOD_SETUP, CMD_MODSETUP_SET_TIME, &tmp, 1, ackbuff);
            return true;
        case CMD_MODSETUP_KEY: 
            ///code = payload->data[0];
            ///state = payload->data[1];
			
            //LOG_LEVEL(F_NAME,"CMD_MODSETUP_KEY  key %02x state %02x\r\n", code, state);
			
            ///KeySendKeyCodeEvent(code, state);
            tmp = 0x01;
            ptl_com_uart_build_frame(A2M_MOD_SETUP, CMD_MODSETUP_KEY, &tmp, 1, ackbuff);
            return false;
        default:
            break;
        }
    }
		
    return false;
}

void system_handshake_with_mcu(void)
{
    LOG_LEVEL("system send handshake data to mcu\r\n");
    send_message(TASK_ID_PTL, A2M_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, 0); 
}

void system_handshake_with_app(void)
  {
    LOG_LEVEL("system send handshake data to app\r\n");
    send_message(TASK_ID_PTL, M2A_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, 0); 
    ///ptl_proc_buff_t ackbuff;
    ///ptl_com_uart_build_frame(M2A_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, (uint8_t*)VER_STR, sizeof(VER_STR), &ackbuff);
  }


void system_set_mpu_status(uint8_t status)
{
    LOG_LEVEL("system set mpu status=%d \r\n", status);
    l_u8_mpu_status = status;
    send_message(TASK_ID_SYSTEM, CMD_MODSYSTEM_APP_STATE, l_u8_mpu_status, l_u8_mpu_status);
}

uint8_t system_get_mpu_status(void)
{
    return l_u8_mpu_status;
}

bool system_get_power_off_req(void)
{
    return l_u8_power_off_req;
}
mb_state_t system_get_mb_state(void)
{
    return lt_mb_state;
}


		
