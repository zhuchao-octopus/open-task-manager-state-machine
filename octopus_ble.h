/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 *
 */
#ifndef __OCTOPUS_TASK_MANAGER_BLE_H__
#define __OCTOPUS_TASK_MANAGER_BLE_H__

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

////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * TYPEDEFS
 */
typedef struct
{
  uint8_t mode;
  bool locked;
  bool to_lock;
  uint8_t mac[8];
}BLE_STATUS;
/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */

void app_ble_init_running(void);
void app_ble_start_running(void);
void app_ble_assert_running(void);
void app_ble_running(void);
void app_ble_post_running(void);
void app_ble_stop_running(void);


#ifdef __cplusplus
}
#endif


#endif
