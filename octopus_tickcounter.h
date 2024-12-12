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
 * uint32_t bit if Tick is 1ms per Tick,
 * then counter max time is (0xFFFFFFFF)ms = 49.71 days.
 */

/*******************************************************************************
 * CONSTANTS
 */


/*******************************************************************************
 * GLOBAL VARIABLES DECLEAR
 */

/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */

void StartTickCounter(uint32_t* timer);
void StopTickCounter(uint32_t* timer);
void RestartTickCounter(uint32_t* timer);

bool IsTickCounterStart(const uint32_t *timer);

uint32_t GetTickCounter(const uint32_t* timer);
uint32_t GetSystemTickClock( void );

void SetSystemTickClock(uint32_t timer_tick);
void ResetSystemTickClock(void);

/**
 * end of group
 * @}
 */

#ifdef __cplusplus
}
#endif


#endif
