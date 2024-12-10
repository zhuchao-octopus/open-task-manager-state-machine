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


#if 1
void Date2tm(struct tm* pTM, const char* pData)
{
	struct tm timeInfo;
	char* tokenPtr = NULL;
	char dataTimeTest[40] = { 0 };
	char arrDate[20] = { 0 };
	char arrTime[20] = { 0 };
	// struct tm* ptmDate = NULL;

	if (NULL == pData || NULL == pTM)
		return;

	memset(&timeInfo, 0, sizeof(struct tm));
	// ptmDate = (struct tm*)pTM;
	strcpy(dataTimeTest, pData);
	dataTimeTest[39] = 0;

	tokenPtr = strtok(dataTimeTest, " ");
	if (tokenPtr) // 日期
	{
		strcpy(arrDate, tokenPtr);
		tokenPtr = strtok(NULL, " ");
		if (tokenPtr)
		{
			strcpy(arrTime, tokenPtr);
		}
	}

	// 日期
	tokenPtr = strtok(arrDate, ".");
	if (tokenPtr)
		timeInfo.tm_mday = atoi(tokenPtr);
	tokenPtr = strtok(NULL, ".");
	if (tokenPtr)
		timeInfo.tm_mon = atoi(tokenPtr);
	tokenPtr = strtok(NULL, ".");
	if (tokenPtr)
		timeInfo.tm_year = atoi(tokenPtr);

	// 时间
	tokenPtr = strtok(arrTime, ":");
	if (tokenPtr)
		timeInfo.tm_hour = atoi(tokenPtr);
	tokenPtr = strtok(NULL, ":");
	if (tokenPtr)
		timeInfo.tm_min = atoi(tokenPtr);
	tokenPtr = strtok(NULL, ":");
	if (tokenPtr)
		timeInfo.tm_sec = atoi(tokenPtr);

	*pTM = timeInfo;
}
#endif

//------ end add ECTiny Sample ------//


 
