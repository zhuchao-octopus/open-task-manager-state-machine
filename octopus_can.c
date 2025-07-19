// File: octopus_can.c
// Description: Implementation for CAN data dispatching and parsing logic
// Author: ak47
// Created: 2025-04-17
#include "octopus_platform.h"     // Include platform-specific header for hardware platform details
#include "octopus_carinfor.h"
#include "octopus_gpio.h"
#include "octopus_system.h"

#include "octopus_can.h"
#include "can/can_queue.h"
#include "can/can_message_rx.h"
#include "can/can_message_tx.h"
#include "can/can_message_l.h"

#include "can/can_function.h"

#ifdef TASK_MANAGER_STATE_MACHINE_CAN
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
extern carinfo_meter_t lt_meter;         // Local meter data structure
extern carinfo_indicator_t lt_indicator; // Local indicator data structure

static bool can_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool can_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);


/*******************************************************************************
 * Global Function Implementations
 ******************************************************************************/

/**
 * @brief Initializes the system for running.
 *
 * This function registers the system module with the communication layer
 * and transitions the system task to an invalid state.
 */
void task_can_init_running(void)
{
   LOG_LEVEL("task_can_init_running\r\n");
   OTMS(TASK_MODULE_CAN, OTMS_S_INVALID);
   ptl_register_module(MCU_TO_SOC_MOD_CAN, can_send_handler, can_receive_handler);
	
	 Can_Queue_Init(&CAN_rx_msg_queue);
	
	 can_function_init();
	 can_message_case_init();
}

void task_can_start_running(void)
{
	  LOG_LEVEL("task_can_start_running\r\n");
    OTMS(TASK_MODULE_CAN, OTMS_S_ASSERT_RUN);
}

void task_can_assert_running(void)
{
	  ptl_reqest_running(MCU_TO_SOC_MOD_CAN);
    OTMS(TASK_MODULE_CAN, OTMS_S_RUNNING);
}

void task_can_running(void)
{
		can_function_loop_rt();
		//can_ptl_loop_10ms();
}

void task_can_post_running(void)
{
   OTMS(TASK_MODULE_CAN, OTMS_S_ASSERT_RUN); 	
}

void task_can_stop_running(void)
{
	LOG_LEVEL("_stop_running\r\n");
    OTMS(TASK_MODULE_CAN, OTMS_S_INVALID);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static bool can_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
		return false;
}
static bool can_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
		return false;
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

// Internal function to parse a received CAN message
void parse_can_message(const CAN_Message_t* message)
{
    // Example: Check message ID and parse accordingly
	  LOG_BUFF_LEVEL((const uint8_t *)&message,sizeof(CAN_Message_t));
	  //LOG_BUFF_LEVEL((const uint8_t *)&message->Data,message->DLC);
	  CanQueue_Push(&CAN_rx_msg_queue, 0, message->StdId, message->Data, message->DLC);
   
    switch (message->StdId)
    {
    case 0x100:
        break;
    case 0x200:
        break;
    default:
        break;
    }
}
#endif

