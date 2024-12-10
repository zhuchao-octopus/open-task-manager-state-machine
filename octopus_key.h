/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 *
 */
#ifndef __OCTOPUS_TASK_MANAGER_KEY_H__
#define __OCTOPUS_TASK_MANAGER_KEY_H__

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

#define OCTOPUS_KEY_0 0
#define OCTOPUS_KEY_1 1
#define OCTOPUS_KEY_2 2
#define OCTOPUS_KEY_3 3
#define OCTOPUS_KEY_4 4
#define OCTOPUS_KEY_5 5
#define OCTOPUS_KEY_6 6
#define OCTOPUS_KEY_7 7
#define OCTOPUS_KEY_8 8
#define OCTOPUS_KEY_9 9
#define OCTOPUS_KEY_10 10
#define OCTOPUS_KEY_11 11
#define OCTOPUS_KEY_12 12
#define OCTOPUS_KEY_13 13
#define OCTOPUS_KEY_14 14
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * TYPEDEFS
 */

#define KEY_CODE_IDLE               (0x00)      
#define KEY_CODE_MENU               (0x01)     
#define KEY_CODE_UP                 (0x02)      
#define KEY_CODE_DOWN               (0x03)      
#define KEY_CODE_OK                 (0x04)      
#define KEY_CODE_LEFT               (0x05)      
#define KEY_CODE_RIGHT              (0x06)      
#define KEY_CODE_BACK               (0x07)     
#define KEY_CODE_INFO               (0x08)      
#define KEY_CODE_ILL                (0x09)      
#define KEY_CODE_POWER              (0x0A)      


#define KEY_STATE_RELEASE           (0x00)      
#define KEY_STATE_PRESSED           (0x01)      
#define KEY_STATE_LONG_PRESSED      (0x02)      
#define KEY_STATE_DOUBLE_PRESSED    (0x03)      
/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */

void app_key_init_running(void);
void app_key_start_running(void);
void app_key_assert_running(void);
void app_key_running(void);
void app_key_post_running(void);
void app_key_stop_running(void);


#ifdef __cplusplus
}
#endif


#endif
