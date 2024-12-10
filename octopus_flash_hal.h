/**
 * ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 * Header file for the Octopus Task Manager module.
 * Defines macros, includes required libraries, and declares functions.
 */

#ifndef __OCTOPUS_TASK_MANAGER_FLASH_HAL_H__
#define __OCTOPUS_TASK_MANAGER_FLASH_HAL_H__

#include "octopus_platform.h"


#ifdef __cplusplus
extern "C"
{
#endif

#define USR_FLASH_TEST			0x0001

    

/*********************************************************************
 * FUNCTIONS
 */
void hal_flash_init(uint8_t task_id);
void hal_flash_read_to_buff(uint32_t addr, uint8_t *buf, uint32_t len);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HEARTRATE_H */
