/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 *
 */
#ifndef __OCTOPUS_TASK_MANAGER_FLASH_H__
#define __OCTOPUS_TASK_MANAGER_FLASH_H__

/*******************************************************************************
 * INCLUDES
 */
 
 #include "octopus_platform.h"
#include "octopus_flash_hal.h" 


#ifdef __cplusplus
extern "C"{
#endif



/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */

extern void PrintfBuffHex(const char *fun, int line, char *str, uint8_t *dat, int len);
extern void FlashReadToBuff(uint32_t addr, uint8_t *buf, uint32_t len);
extern void FlashWriteBuffTo(uint32_t addr, uint8_t *buf, uint32_t len);


#ifdef __cplusplus
}
#endif


#endif


