/**
 * ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 * Header file for the Octopus Task Manager module.
 * Defines macros, includes required libraries, and declares functions.
 */

#ifndef __OCTOPUS_TASK_MANAGER_TIMER_HAL_H__
#define __OCTOPUS_TASK_MANAGER_TIMER_HAL_H__

#include "octopus_platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define TIMER_1000_MS_EVT                 0x0001

/*********************************************************************
 * FUNCTIONS
 */

void hal_timer_init(uint8_t timer_id);
void hal_timer_interrupt_callback(uint8_t evt);


#ifdef __cplusplus
}
#endif

#endif /* HEARTRATE_H */
