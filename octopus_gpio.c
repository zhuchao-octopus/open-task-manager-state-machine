
/*******************************************************************************
 * INCLUDES
 */

#include "octopus_gpio.h"
#include "octopus_timer.h"
#include "octopus_msgqueue.h"
#include "octopus_task_manager.h"
#include "octopus_flash.h"
#include "octopus_log.h"
/*******************************************************************************
 * DEBUG SWITCH MACROS
*/

/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */

void GPIOInit(void);
void PollingGPIOStatus(uint8_t pin, GPIO_STATUS *gpio_status);
void PollingGPIOKeyStatus(uint8_t pin,GPIO_KEY_STATUS *key_status);
void ProcessKeyEvent(GPIO_KEY_STATUS *key_status);

static uint32_t           l_t_msg_wait_50_timer;
 
 /*******************************************************************************
 * GLOBAL VARIABLES
 */
GPIO_KEY_STATUS key_status_1 = {0,0,0,0,0,0};
GPIO_STATUS acc_status = {false,true,0,0};
GPIO_STATUS ddd_status = {false,true,0,0};
GPIO_STATUS zzd_status = {false,true,0,0};
GPIO_STATUS yzd_status = {false,true,0,0};
GPIO_STATUS skd_status = {false,true,0,0};

//static bool module_send_handler(ptl_frame_type_t frame_type, ptl_frame_cmd_t cmd, uint16_t param, ptl_proc_buff_t *buff);
//static bool module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void app_gpio_init_running(void)
{
    LOG_LEVEL(F_NAME,"app_gpio_init\r\n");
	  GPIOInit();
    //com_uart_ptl_register_module(MSGMODULE_SYSTEM, module_send_handler, module_receive_handler);
    OTMS(TASK_ID_GPIO, OTMS_S_INVALID);
}

void app_gpio_start_running(void)
{
    LOG_LEVEL(F_NAME,"app_gpio_start\r\n");
    OTMS(TASK_ID_GPIO, OTMS_S_ASSERT_RUN);
}

void app_gpio_assert_running(void)
{
    StartTimer(&l_t_msg_wait_50_timer);
    OTMS(TASK_ID_GPIO, OTMS_S_RUNNING);
}

void app_gpio_running(void)
{
   if(GetTimer(&l_t_msg_wait_50_timer) >= 20)
	 {
			PollingGPIOStatus(GPIO_ACC_PIN,&acc_status);
			PollingGPIOStatus(GPIO_DDD_PIN,&ddd_status);
			PollingGPIOStatus(GPIO_ZZD_PIN,&zzd_status);
			PollingGPIOStatus(GPIO_YZD_PIN,&yzd_status);
			PollingGPIOStatus(GPIO_SKD_PIN,&skd_status);
			PollingGPIOKeyStatus(GPIO_KEY_PIN,&key_status_1);
		 
      ProcessKeyEvent(&key_status_1);
			
			if(acc_status.changed)
			{
				LOG_LEVEL(F_NAME,"get acc status=%d\r\n",acc_status.offon);
				send_message(TASK_ID_BLE, MSG_DEVICE_GPIO_EVENT, GPIO_ACC_PIN, acc_status.offon);
				send_message(TASK_ID_SYSTEM, MSG_DEVICE_GPIO_EVENT, GPIO_ACC_PIN, acc_status.offon);
				acc_status.changed=false;
			}	
			
	  	if(ddd_status.changed)
			{
				LOG_LEVEL(F_NAME,"get ddd status=%d\r\n",ddd_status.offon);
				send_message(TASK_ID_CAR_INFOR, MSG_DEVICE_GPIO_EVENT, GPIO_DDD_PIN, ddd_status.offon);
				ddd_status.changed=false;
			}	
			
			if(zzd_status.changed)
			{
				LOG_LEVEL(F_NAME,"get zzd status=%d\r\n",zzd_status.offon);
				send_message(TASK_ID_CAR_INFOR, MSG_DEVICE_GPIO_EVENT, GPIO_ZZD_PIN, zzd_status.offon);
				zzd_status.changed=false;
			}	
			
			if(yzd_status.changed)
			{
				LOG_LEVEL(F_NAME,"get yzd status=%d\r\n",yzd_status.offon);
				send_message(TASK_ID_CAR_INFOR, MSG_DEVICE_GPIO_EVENT, GPIO_YZD_PIN, yzd_status.offon);
				yzd_status.changed=false;
			}	
			
			if(skd_status.changed)
			{
				LOG_LEVEL(F_NAME,"get skd status=%d\r\n",skd_status.offon);
				send_message(TASK_ID_CAR_INFOR, MSG_DEVICE_GPIO_EVENT, GPIO_SKD_PIN, skd_status.offon);
				skd_status.changed=false;
			}
			
		RestartTimer(&l_t_msg_wait_50_timer); 
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
void GPIOInit(void)
{

}
 
void PollingGPIOStatus(uint8_t pin, GPIO_STATUS *gpio_status)
{
	if(GPIO_PIN_READ(pin))
	{
		gpio_status->count1++;
		gpio_status->count2=0;
		if(gpio_status->count1 > GPIO_STATUS_REDUNDANCY)
		{
			if(!gpio_status->offon)
		  	gpio_status->changed=true;
			else
				gpio_status->changed=false;
			gpio_status->offon=true;
			gpio_status->count1=0;
		}
	}
	else
	{
		gpio_status->count1 = 0;
		gpio_status->count2++;
		if(gpio_status->count2 > GPIO_STATUS_REDUNDANCY)
		{
			if(gpio_status->offon)
		  	gpio_status->changed=true;
			else
				gpio_status->changed=false;
		  gpio_status->offon=false;
			gpio_status->count2=0;
		}
	}
}

void PollingGPIOKeyStatus(uint8_t pin,GPIO_KEY_STATUS *key_status)
{
	if(!GPIO_PIN_READ(pin))
	{
		if(key_status->count1 < GPIO_KEY_STATUS_MAX_REDUNDANCY)
		   key_status->count1++;
		
		if(key_status->count1 > GPIO_KEY_STATUS_REDUNDANCY*4 && key_status->pressed)
		{
			key_status->key = hal_get_gpio_key_mask_code(pin);//key_status->key | GetGPIOKeyMask(pin);
			key_status->pressed=true;
			key_status->pressed_l=true;
			key_status->released=false;
			key_status->dispatched=false;
			return;
		}
		
		if(key_status->count1 > GPIO_KEY_STATUS_REDUNDANCY && !key_status->pressed)
		{
			key_status->key = hal_get_gpio_key_mask_code(pin);//key_status->key | GetGPIOKeyMask(pin);
			key_status->pressed=true;
			key_status->pressed_l=false;
			key_status->released=false;
			key_status->dispatched=false;
		}	
	}
	else
	{
		key_status->count1 = 0;
		if(key_status->pressed)
			key_status->released=true;
		key_status->pressed=false;
		///key_status->key = key_status->key & (~GetGPIOKeyMask(pin));
	}
} 

void ProcessKeyEvent(GPIO_KEY_STATUS *key_status)
{
	if(!key_status->dispatched)	
	{	
		if(key_status->pressed_l)//long press
		{
			///LOG_LEVEL(F_NAME,"long press status key=%d %d\r\n",key_status->key,key_status->pressed_l);
			send_message(TASK_ID_KEY, MSG_DEVICE_KEY_EVENT , key_status->key, 2);
			key_status->dispatched=true;
		}
    else if(key_status->pressed)
		{
			///LOG_LEVEL(F_NAME,"press status key=%d %d\r\n",key_status->key,key_status->pressed_l);
			send_message(TASK_ID_KEY, MSG_DEVICE_KEY_EVENT , key_status->key, 1);
      key_status->dispatched=true;  
		}			
		
	  if(key_status->released)//short press
		{
			///LOG_LEVEL(F_NAME,"released status key=%d\r\n",key_status->key);
			send_message(TASK_ID_KEY, MSG_DEVICE_KEY_EVENT , key_status->key, 0);
			key_status->released=false;
			key_status->dispatched=true;
		}
	}	
}

bool IsAccOn(void)
{
	return GPIO_PIN_READ_ACC();
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


