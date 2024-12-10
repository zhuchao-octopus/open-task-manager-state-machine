/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 */

/*******************************************************************************
 * INCLUDES
 */

#include "octopus_timer.h"
/*******************************************************************************
 * DEBUG SWITCH MACROS
 */


/*******************************************************************************
 * MACROS
 */
#define TICK_MAX                ((TimerType)-1)


/*******************************************************************************
 * TYPEDEFS
 */

/**
 * \defgroup GROUP_LOCAL_FUNCTIONS OS:TIMER:LOCAL_FUNCTIONS
 */


/*******************************************************************************
 * CONSTANTS
 */
/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */



/*******************************************************************************
 * GLOBAL VARIABLES
 */


/*******************************************************************************
 * STATIC VARIABLES
 */
static TimerType     OsTickCounter = 0;
//static TimerType 		 SystemClock_ms = 0;


/*******************************************************************************
 * EXTERNAL VARIABLES
 */


/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */

void StartTimer(TimerType* timer)
{
	assert(timer!=NULL);

	 OsTickCounter = GetSystemTickClock();

	if (OsTickCounter == 0U)
	{
		*timer = 1;
	}
	else
	{
		*timer = OsTickCounter;
	}
}

void StopTimer(TimerType* timer)
{
	assert(timer!=NULL);
	*timer = 0;
}

void RestartTimer(TimerType* timer)
{
	StartTimer(timer);
}

TimerType GetTimer(const TimerType* timer)
{
	TimerType diff;
	assert(timer!=NULL);

	 OsTickCounter = GetSystemTickClock();

	if(0 == *timer)
	{
		diff = 0U;
	}
	else
	{
		if (OsTickCounter >= *timer)
		{
			diff = OsTickCounter - *timer;
		}
		else
		{
			diff = (TICK_MAX - *timer) + OsTickCounter;
		}
	}
	return diff;
}

bool IsTimerStart(const TimerType *timer)
{
    return *timer != 0;
}

uint32_t GetSystemTickClock( void )
{
   return GET_SYSTEM_TICK_COUNT;
}

void SetSystemTickClock(uint32_t timer_tick)
{
   OsTickCounter = OsTickCounter + timer_tick;
}

void ResetSystemTickClock(void)
{
   OsTickCounter = 0;//TMOS_GetSystemClock() * 625 / 1000;
}
/*******************************************************************************
 * LOCAL FUNCTIONS IMPLEMENTATION
 */
