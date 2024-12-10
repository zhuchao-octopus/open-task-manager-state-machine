/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 *
 */
#ifndef __OCTOPUS_TASK_MANAGER_TIMER_H__
#define __OCTOPUS_TASK_MANAGER_TIMER_H__

/*******************************************************************************
 * INCLUDES
 */
 
#include "octopus_platform.h"

#ifdef __cplusplus
extern "C"{
#endif


/**
 * \defgroup  APP:SUB_TYPE
 * @{
 */


/*******************************************************************************
 * DEBUG SWITCH MACROS
 */


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * TYPEDEFS
 */
/* Kernel Timer.
 * If Tick is 1ms per Tick,
 * then counter max time is (0xFFFFFFFF)ms = 49.71 days.
 */
typedef uint32_t TimerType;

/*******************************************************************************
 * CONSTANTS
 */


/*******************************************************************************
 * GLOBAL VARIABLES DECLEAR
 */

/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */

extern void StartTimer(TimerType* timer);
extern void StopTimer(TimerType* timer);
extern void RestartTimer(TimerType* timer);

extern TimerType GetTimer(const TimerType* timer);

extern bool IsTimerStart(const TimerType *timer);
extern uint32_t GetSystemTickClock( void );

extern void SetSystemTickClock(uint32_t timer_tick);

extern void ResetSystemTickClock(void);

/**
 * end of group
 * @}
 */

#ifdef __cplusplus
}
#endif


#endif
