
/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"
#include "octopus_log.h" 
#include "octopus_flash.h"
#include "octopus_key.h"
#include "octopus_timer.h"
#include "octopus_msgqueue.h"
#include "octopus_task_manager.h"

/*******************************************************************************
 * DEBUG SWITCH MACROS
*/

/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
uint8_t get_dummy_key(uint8_t key);
 /*******************************************************************************
 * GLOBAL VARIABLES
 */
uint8_t bt_mac[8];
 
static uint32_t           l_t_msg_wait_timer;

static bool module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void app_key_init_running(void)
{
    LOG_LEVEL("app_key_init\r\n");
    ptl_com_uart_register_module(M2A_MOD_SETUP, module_send_handler, module_receive_handler);
    OTMS(TASK_ID_KEY, OTMS_S_INVALID);
}

void app_key_start_running(void)
{
    LOG_LEVEL("app_key_start\r\n");
    OTMS(TASK_ID_KEY, OTMS_S_ASSERT_RUN);
}

void app_key_assert_running(void)
{
	  com_uart_reqest_running(M2A_MOD_SETUP);
    StartTimer(&l_t_msg_wait_timer);
    OTMS(TASK_ID_KEY, OTMS_S_RUNNING);
}

void app_key_running(void)
{
		if(GetTimer(&l_t_msg_wait_timer) < 20)
        return;
		RestartTimer(&l_t_msg_wait_timer);
		uint16_t param = 0;
    Msg_t* msg = get_message(TASK_ID_KEY);		
		if(msg->id != NO_MSG && (MsgId_t)msg->id == MSG_DEVICE_KEY_EVENT)
    {
			uint8_t key = get_dummy_key(msg->param1);
			///LOG_LEVEL("key pressed key=%d key_status=%d\r\n",key,msg->param2);
      switch (key)
			{
				 case OCTOPUS_KEY_0:
						break;
				 case OCTOPUS_KEY_1:
					 break;
				 case OCTOPUS_KEY_14:
					 //FlashReadToBuff(0x1107c004,bt_mac,6);
				   //PrintfBuffHex(__func__, __LINE__, "read bt mac", bt_mac, 6);
				 #if 0
				 #ifdef TASK_MANAGER_STATE_MACHINE_MCU
					system_handshake_with_app();
				 #endif
				 #ifdef TASK_MANAGER_STATE_MACHINE_SOC
					system_handshake_with_mcu();
				 #endif
				 #endif
				 param = MK_WORD(KEY_CODE_MENU,KEY_STATE_PRESSED);
                 send_message(TASK_ID_PTL, M2A_MOD_SETUP , CMD_MODSETUP_KEY, param);
				 break;
			}	
    }		
}

void app_key_post_running(void)
{

}

void app_key_stop_running(void)
{
    OTMS(TASK_ID_KEY, OTMS_S_INVALID);
}
////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

uint8_t get_dummy_key(uint8_t key)
{
 switch(key)
  {
    case 0:		return OCTOPUS_KEY_0;
    case 1:		return OCTOPUS_KEY_1;
    case 2:		return OCTOPUS_KEY_2;
    case 3:		return OCTOPUS_KEY_3;	

    case 14:	return OCTOPUS_KEY_14;	
  }	
	return 0;
}

bool module_send_handler(ptl_frame_type_t frame_type,  uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    assert(buff);
    uint8_t tmp[8] = {0};
    //PRINT("setting_module_send_handler  MOD %02x  CMD %02x PRARM %04x\n", module, cmd, param);
    if(M2A_MOD_SETUP == frame_type)
    {
      switch(param1)
      {
        case CMD_MODSETUP_KEY: 		//
            tmp[0] = MSB_WORD(param2); //KEYCODE
            tmp[1] = LSB_WORD(param2); //KEYSTATE
            tmp[2] = 0;  					//
            LOG_LEVEL("CMD_MODSETUP_KEY key %02x state %02x\n",tmp[0],tmp[1]);
            ptl_com_uart_build_frame(M2A_MOD_SETUP, CMD_MODSETUP_KEY, tmp, 3, buff);
            return true;
        default:
            break;
      }
    }
    return false;
}

bool module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    assert(payload);
    assert(ackbuff);
    uint8_t tmp;
	
    if(M2A_MOD_SETUP == payload->frame_type)
    {
        switch(payload->cmd)
        {
        case CMD_MODSETUP_UPDATE_TIME: 
            //ACK, no thing to do
            return false;
        case CMD_MODSETUP_SET_TIME: 
            tmp = 0x01;
            ptl_com_uart_build_frame(M2A_MOD_SETUP, CMD_MODSETUP_SET_TIME, &tmp, 1, ackbuff);
            return true;
        case CMD_MODSETUP_KEY: 
            //ACK, no thing to do
            return false;
        default:
            break;
        }
    }

    return false;
}


