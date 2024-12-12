/**
 * ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 * C file for the Octopus Task Manager module.
 * Defines macros, includes required libraries, and declares functions.
 */

/*******************************************************************************
 * INCLUDES
 */
 
#include "octopus_platform.h"
#include "octopus_log.h"
#include "octopus_ble_hal.h"
 
#include "octopus_ble.h"
#include "octopus_tickcounter.h"
#include "octopus_msgqueue.h"
#include "octopus_task_manager.h"
#include "octopus_flash.h"
/*******************************************************************************
 * DEBUG SWITCH MACROS
*/

/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
void CheckBLeBondedConnectionStatus(uint8_t *connected_mac, uint8_t c_type);

void StartToLock(void);
void StartToUnlock(void);
void BLE_connecttion_lock_polling(void);

 /*******************************************************************************
 * GLOBAL VARIABLES
 */
BLE_STATUS ble_status = {false,false,false, { 0, 0,0, 0, 0, 0, 0, 0 } };

uint8_t ble_bonded_mac[6][6] = { 
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },	
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
{ 0x14, 0xd7, 0x37, 0x9a, 0x5c, 0x94 }, 

{ 0x24, 0xE9, 0x66, 0x19, 0x26, 0x40 },
{ 0xae, 0xd8, 0xb4, 0xf6, 0x40, 0xc4 },	
};

static uint32_t 		  l_t_msg_wait_timer=0;
static uint32_t           l_t_msg_wait_10_timer=0;
static uint32_t           l_t_msg_wait_50_timer=0;

//static bool module_send_handler(ptl_frame_type_t frame_type, ptl_frame_cmd_t cmd, uint16_t param, ptl_proc_buff_t *buff);
//static bool module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void app_ble_init_running(void)
{ 
  OTMS(TASK_ID_BLE, OTMS_S_INVALID);
 	#ifdef TASK_MANAGER_STATE_MACHINE_BLE
	LOG_LEVEL("app_ble_init\r\n");
	//com_uart_ptl_register_module(MSGMODULE_SYSTEM, module_send_handler, module_receive_handler);
	hal_disable_bLe_pair_mode();
	#endif
}

void app_ble_start_running(void)
{
    LOG_LEVEL("app_ble_start\r\n");
    OTMS(TASK_ID_BLE, OTMS_S_ASSERT_RUN);
}

void app_ble_assert_running(void)
{
	  #ifdef TASK_MANAGER_STATE_MACHINE_BLE
    StartTickCounter(&l_t_msg_wait_10_timer);
    StartTickCounter(&l_t_msg_wait_50_timer);
    OTMS(TASK_ID_BLE, OTMS_S_RUNNING);
	  #endif
}

void app_ble_running(void)
{
   if (GetTickCounter(&l_t_msg_wait_10_timer) < 10)
        return;
		StartTickCounter(&l_t_msg_wait_10_timer);
	 
	 	BLE_connecttion_lock_polling();
	 
		if (GetTickCounter(&l_t_msg_wait_50_timer) < 50)
        return;
		StartTickCounter(&l_t_msg_wait_50_timer);
	
	
    Msg_t* msg = get_message(TASK_ID_BLE);		
		if(msg->id != NO_MSG)
    {
		  if((MsgId_t)msg->id == MSG_DEVICE_GPIO_EVENT)
			{
				if(msg->param1 == GPIO_ACC_PIN)
				{
					if(IsAccOn())		
					{
						ble_status.mode=hal_set_pairing_mode_onoff(true,ble_status.mode);
					}
					else
					{
						ble_status.mode=hal_set_pairing_mode_onoff(false,ble_status.mode);
					}
				}
		  }
			else 
       CheckBLeBondedConnectionStatus(ble_status.mac,msg->param2); 
    } 
}

void app_ble_post_running(void)
{

}

void app_ble_stop_running(void)
{
    OTMS(TASK_ID_BLE, OTMS_S_INVALID);
}

bool IsExistsBondedMAC(uint8_t *connected_mac)
{
 uint8_t length =  sizeof(ble_bonded_mac) / sizeof(ble_bonded_mac[0]);  	
 bool matched = false;	
 
 for (uint8_t i = 0; i < length; i++) 
	{
    if (connected_mac[0] == ble_bonded_mac[i][0] && connected_mac[1] == ble_bonded_mac[i][1] && connected_mac[2] == ble_bonded_mac[i][2]
      && connected_mac[3] == ble_bonded_mac[i][3]&& connected_mac[4] == ble_bonded_mac[i][4] && connected_mac[5] == ble_bonded_mac[i][5]) 
		{
        matched = true;
        break;
     }
  }
	return matched;
} 

void CheckBLeBondedConnectionStatus(uint8_t *connected_mac, uint8_t c_type)
{
	
	///uint8_t length =  sizeof(connected_mac) / sizeof(connected_mac[0]);  
	
  ///if(length < 6)
	///{
	///		LOG_LEVEL("OnCheckBondedMAC invalid mac length=%d c_type=%d\r\n", length, c_type);
	///		return;
	///}	
  if(c_type == DEVICE_BLE_DISCONNECTED || c_type == DEVICE_BLE_CONNECTED)	
	{
		if ((connected_mac[0] == 0) && (connected_mac[1] == 0) && (connected_mac[2] == 0) && (connected_mac[3] == 0) && (connected_mac[4] == 0) && (connected_mac[5] == 0))
		{
			LOG_LEVEL("BLeConnectionStatus no valid mac c_type=%d\r\n", c_type);
			return;
		}
		if ((connected_mac[0] == 0xFF) && (connected_mac[1] == 0xFF) && (connected_mac[2] == 0xFF) && (connected_mac[3] == 0xFF) && (connected_mac[4] == 0xFF) && (connected_mac[5] == 0xFF))
		{
			LOG_LEVEL("BLeConnectionStatus no valid mac c_type=%d\r\n", c_type);
			return;
		}
  }
	///bool matched = IsExistsBondedMAC(connected_mac);
	///if(!matched)
	///{
	///		LOG_LEVEL(F_NAME,"OnCheckBondedMAC no boned exists mac c_type=%d\r\n", c_type);
	///		return;	
	///}
	
	//LOG_LEVEL(F_NAME,"OnCheckBondedMAC ble connection type=%d\r\n", c_type);

	switch (c_type)
	{	
			case DEVICE_BLE_PAIR:
			case DEVICE_BLE_BONDED:
				FlashReadToBuff(0x1107c004,ble_status.mac,6);
	      PrintfBuffHex(__func__, __LINE__, "BLE", ble_status.mac, 6);
			case DEVICE_BLE_CONNECTED:
				StartToUnlock();
			break;
			case DEVICE_BLE_DISCONNECTED:
			  StartToLock();
			break;
			default:
			break;
	}
}

void BLE_connecttion_lock_polling(void)
{
	//uint32_t speed = app_carinfo_getSpeed();
	//PRINT("OnDelaySleepSystem %d\r\n", speed);
	if (ble_status.to_lock && GetTickCounter(&l_t_msg_wait_timer) > 10000)
	{
        StopTickCounter(&l_t_msg_wait_timer);
        ble_status.locked = true;
        ble_status.to_lock = false;	
        LOG_LEVEL("Start to power off system...\r\n");
        send_message(TASK_ID_SYSTEM, MSG_DEVICE_NORMAL_EVENT , CMD_MODSYSTEM_POWER_OFF,0);
	}
}

void StartToLock(void)
{
	if (!IsTickCounterStart(&l_t_msg_wait_timer))
	{
		//if(!IsAccOn())
		{
			ble_status.to_lock = true;
			ble_status.to_lock = true;
			StartTickCounter(&l_t_msg_wait_timer);
			LOG_LEVEL("Start to lock system...\r\n");
		}
	}
}

void StartToUnlock(void)
{
	if(IsTickCounterStart(&l_t_msg_wait_timer))
	{
		ble_status.to_lock = false;
		ble_status.locked = false;
		StopTickCounter(&l_t_msg_wait_timer);
	}
	send_message(TASK_ID_SYSTEM, MSG_DEVICE_NORMAL_EVENT , CMD_MODSYSTEM_POWER_ON,1);
	LOG_LEVEL("Start to unlock system...\r\n");
}


