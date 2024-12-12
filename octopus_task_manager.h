/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 */
 
#ifndef __OCTOPUS_TASK_MANAGER_H__
#define __OCTOPUS_TASK_MANAGER_H__

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"
#include "octopus_system.h"
#include "octopus_gpio.h"
#include "octopus_carinfor.h"
#include "octopus_ble.h"
#include "octopus_key.h"
#include "octopus_uart_ptl.h"

#ifdef __cplusplus
extern "C"{
#endif

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */


/*******************************************************************************
 * MACROS
 */

/* OTMS states */
#define OTMS_S_INIT                (0x00U)
#define OTMS_S_START               (0x01U)
#define OTMS_S_ASSERT_RUN          (0x02U)          /* check running condition */
#define OTMS_S_RUNNING             (0x03U)
#define OTMS_S_POST_RUN            (0x04U)          /* for doing something before sleep */
#define OTMS_S_STOP                (0x05U)
#define OTMS_S_INVALID             (0xFFU)

#define OTMS_S_COUNT               6U

/*******************************************************************************
 * TYPEDEFS
 */
 
typedef int32_t otms_id_t;
typedef uint8_t otms_state_t;
typedef void (*otms_state_func_t) (void);

typedef struct
{
    otms_state_t state_limit;
    const otms_state_func_t func[OTMS_S_COUNT];
}otms_t;    /* task State Machine */


typedef enum{
	TASK_ID_SYSTEM=0,
	TASK_ID_GPIO,
	TASK_ID_PTL,
	TASK_ID_CAR_INFOR,
	TASK_ID_BLE,
	TASK_ID_KEY,
  TASK_ID_MAX_NUM
}TaskModule_t;

/*******************************************************************************
 * CONSTANTS
 */

const otms_t *otms_get_config(void);

const static otms_t lat_otms_config[TASK_ID_MAX_NUM] =
{
		[TASK_ID_SYSTEM] = {
        .state_limit = OTMS_S_INVALID,
        .func = {
            [OTMS_S_INIT] = app_system_init_running,
            [OTMS_S_START] = app_system_start_running,
            [OTMS_S_ASSERT_RUN] = app_system_assert_running,
            [OTMS_S_RUNNING] = app_system_running,
            [OTMS_S_POST_RUN] = app_system_post_running,
            [OTMS_S_STOP] = app_system_stop_running,
        },
    },
			
		[TASK_ID_GPIO] = {
        .state_limit = OTMS_S_INVALID,
        .func = {
            [OTMS_S_INIT] = app_gpio_init_running,
            [OTMS_S_START] = app_gpio_start_running,
            [OTMS_S_ASSERT_RUN] = app_gpio_assert_running,
            [OTMS_S_RUNNING] = app_gpio_running,
            [OTMS_S_POST_RUN] = app_gpio_post_running,
            [OTMS_S_STOP] = app_gpio_stop_running,
        },
    },
		
  [TASK_ID_PTL] = {
        .state_limit = OTMS_S_INVALID,
        .func = {
            [OTMS_S_INIT] = ptl_init_running,
            [OTMS_S_START] = ptl_start_running,
            [OTMS_S_ASSERT_RUN] = ptl_assert_running,
            [OTMS_S_RUNNING] = ptl_running,
            [OTMS_S_POST_RUN] = ptl_post_running,
            [OTMS_S_STOP] = ptl_stop_running,
        },
    },
	
	[TASK_ID_CAR_INFOR] = {
        .state_limit = OTMS_S_INVALID,
        .func = {
            [OTMS_S_INIT] = app_carinfo_init_running,
            [OTMS_S_START] = app_carinfo_start_running,
            [OTMS_S_ASSERT_RUN] = app_carinfo_assert_running,
            [OTMS_S_RUNNING] = app_carinfo_running,
            [OTMS_S_POST_RUN] = app_carinfo_post_running,
            [OTMS_S_STOP] = app_carinfo_stop_running,
        },
    },
	
	[TASK_ID_BLE] = {
        .state_limit = OTMS_S_INVALID,
        .func = {
            [OTMS_S_INIT] = app_ble_init_running,
            [OTMS_S_START] = app_ble_start_running,
            [OTMS_S_ASSERT_RUN] = app_ble_assert_running,
            [OTMS_S_RUNNING] = app_ble_running,
            [OTMS_S_POST_RUN] = app_ble_post_running,
            [OTMS_S_STOP] = app_ble_stop_running,
        },
    },
	
	[TASK_ID_KEY] = {
        .state_limit = OTMS_S_INVALID,
        .func = {
            [OTMS_S_INIT] = app_key_init_running,
            [OTMS_S_START] = app_key_start_running,
            [OTMS_S_ASSERT_RUN] = app_key_assert_running,
            [OTMS_S_RUNNING] = app_key_running,
            [OTMS_S_POST_RUN] = app_key_post_running,
            [OTMS_S_STOP] = app_key_stop_running,
        },
    },
};
/*******************************************************************************
 * GLOBAL VARIABLES DECLEAR
 */
/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */

void task_manager_init(void);
void task_manager_start(void);
void task_manager_stop(void);
void task_manager_run(void);
void otms_on_enter_run(void);
void otms_on_exit_post_run(void);

void otms_set_state(otms_id_t task_id, otms_state_t state);
otms_state_t otms_get_state(otms_id_t task_id);

/* trigger the task go to next state */
#define OTMS(task_id,state)   otms_set_state(task_id,state)//otms_set_state(TASK_##task_id,state)

/* get current OTMS state */
#define GET_OTMS_STATE(task_id)         otms_get_state(task_id)


#ifdef __cplusplus
}
#endif


#endif
