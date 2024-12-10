/**
 * ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 * Header file for the Octopus Task Manager module.
 * Defines macros, includes required libraries, and declares functions.
 */

#ifndef __TASK_OCTOPUS_BLE_HAL_H__
#define __TASK_OCTOPUS_BLE_HAL_H__

/*******************************************************************************
 * INCLUDES
 */
 
#include "octopus_platform.h"

#ifdef __cplusplus
extern "C"{
#endif

/*******************************************************************************
 * MACROS
 */
#define DEVICE_BLE_PAIR				  			0x0008
#define DEVICE_BLE_BONDED			  			0x0010
#define DEVICE_BLE_CONNECTED					0x0020
#define DEVICE_BLE_DISCONNECTED				0x0040
#define DEVICE_TIMER_EVENT						0x0080
#define DEVICE_PIN_KEY_PRESS					0x0100
////////////////////////////////////////////////////////////////////////////////


/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */
//#ifdef PLATFORM_CST_OSAL_RTOS
uint8_t hal_set_pairing_mode_onoff(bool ono_ff,uint8_t current_pairing_mod);
void hal_enable_bLe_pair_mode(void);
void hal_disable_bLe_pair_mode(void);
//#endif
#ifdef __cplusplus
}
#endif


#endif
