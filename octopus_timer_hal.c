/**
 * ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 * C file for the Octopus Task Manager module.
 * Defines macros, includes required libraries, and declares functions.
 */

/*********************************************************************
 * INCLUDES
 */

#include "octopus_platform.h"
#include "octopus_log.h"
#include "octopus_sif.h"
#include "octopus_timer_hal.h"

//static uint8 timer_TaskID; 
//static uint8_t s_testCase = 0;
//static void TimerTest(uint8_t testCase);
#ifdef PLATFORM_CST_OSAL_RTOS
void hal_timer_interrupt_callback(uint8_t event);
#endif


#ifdef PLATFORM_CST_OSAL_RTOS
void hal_timer_init(uint8 timer_id)
{
	//timer_TaskID = task_id;
	//HalTimerSet(AP_TIMER_ID_6,2000000);	
	//LOG("when test this case,you can uncomment comment which in timer int function\n");
	HalTimerInit(hal_timer_interrupt_callback);
	HalTimerSet(AP_TIMER_ID_5,50);//50us for sif
	LOG_LEVEL("hal timer init\r\n");
}
 
void hal_timer_interrupt_callback(uint8_t event)
{
	switch(event)
	{
		case HAL_EVT_TIMER_5:
			//LOG("t5\n");
		  HalTimerMaskInt(AP_TIMER_ID_5,true);
			SIF_IO_IRQHandler();
		  HalTimerMaskInt(AP_TIMER_ID_5,false);
			break;
		case HAL_EVT_TIMER_6:
			LOG("t6\n");
			break;
		case HAL_EVT_WAKEUP: 
			LOG("wakeup\n");
			LOG("timer will disable when sleep,so if you want it work please init it when wakeup");
			break;
		case HAL_EVT_SLEEP:  
			LOG("sleep\n");
			break;		
		default:LOG("err ");
			break;
	}
}

#elif defined(PLATFORM_ITE_OPEN_RTOS)
void hal_timer_init(uint8_t timer_id)
{
 	
}

void hal_timer_interrupt_callback(uint8_t event)
{
	
}

#else
void hal_timer_init(uint8_t timer_id)
{	
}

void hal_timer_interrupt_callback(uint8_t event)
{	
}
#endif

