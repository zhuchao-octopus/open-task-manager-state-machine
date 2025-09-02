/*******************************************************************************
 * @file        octopus_task_manager.c
 * @brief       Task manager module for managing multiple tasks and their states
 * octopus task  state machine (otsm)
 * octopus task manager system (otms)
 * This file implements a task manager that controls the lifecycle of tasks
 * in a system. Tasks can transition between various predefined states,
 * such as INIT, START, RUN, and STOP. Each task has its own state and
 * corresponding state-specific function handlers. The task manager provides
 * APIs for initializing, starting, stopping, and running tasks.
 *
 * @version  1.0.0
 * @date     2024-12-09
 * @author   Octopus Team
 *
 * @note        This module assumes a fixed number of tasks (`TASK_MODULE_MAX_NUM`)
 *              and relies on a configuration (`otms_t`) for task-specific
 *              state handling.
 ******************************************************************************/

/******************************************************************************
 * INCLUDES
 */
#include "octopus_configuration.h"
#include "octopus_task_manager.h" // Include task manager for scheduling tasks
#include "octopus_system.h"
#include "octopus_gpio.h"
#include "octopus_key.h"
#include "octopus_update_mcu.h"
#include "octopus_uart_ptl.h" // Include UART protocol header
#include "octopus_uart_upf.h" // Include UART protocol header



/*******************************************************************************
 * CONSTANTS
 ******************************************************************************/
/** Static configuration for all tasks in the OTMS. */
const otms_t task_module_config_table[TASK_MODULE_MAX_NUM] = {

#ifdef TASK_MANAGER_STATE_MACHINE_SYSTEM
    [TASK_MODULE_SYSTEM] = {
        .func = {
            [OTMS_S_INIT] = task_system_init_running,
            [OTMS_S_START] = task_system_start_running,
            [OTMS_S_ASSERT_RUN] = task_system_assert_running,
            [OTMS_S_RUNNING] = task_system_running,
            [OTMS_S_POST_RUN] = task_system_post_running,
            [OTMS_S_STOP] = task_system_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_GPIO
    [TASK_MODULE_GPIO] = {
        .func = {
            [OTMS_S_INIT] = task_gpio_init_running,
            [OTMS_S_START] = task_gpio_start_running,
            [OTMS_S_ASSERT_RUN] = task_gpio_assert_running,
            [OTMS_S_RUNNING] = task_gpio_running,
            [OTMS_S_POST_RUN] = task_gpio_post_running,
            [OTMS_S_STOP] = task_gpio_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_PTL
    [TASK_MODULE_PTL_1] = {
        .func = {
            [OTMS_S_INIT] = task_ptl_init_running,
            [OTMS_S_START] = task_ptl_start_running,
            [OTMS_S_ASSERT_RUN] = task_ptl_assert_running,
            [OTMS_S_RUNNING] = task_ptl_running,
            [OTMS_S_POST_RUN] = task_ptl_post_running,
            [OTMS_S_STOP] = task_ptl_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_UPF
    [TASK_MODULE_UPF] = {
        .func = {
            [OTMS_S_INIT] = task_upf_init_running,
            [OTMS_S_START] = task_upf_start_running,
            [OTMS_S_ASSERT_RUN] = task_upf_assert_running,
            [OTMS_S_RUNNING] = task_upf_running,
            [OTMS_S_POST_RUN] = task_upf_post_running,
            [OTMS_S_STOP] = task_upf_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_IPC
    [TASK_MODULE_IPC] = {
        .func = {
            [OTMS_S_INIT] = task_ipc_init_running,
            [OTMS_S_START] = task_ipc_start_running,
            [OTMS_S_ASSERT_RUN] = task_ipc_assert_running,
            [OTMS_S_RUNNING] = task_ipc_running,
            [OTMS_S_POST_RUN] = task_ipc_post_running,
            [OTMS_S_STOP] = task_ipc_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
    [TASK_MODULE_CAR_INFOR] = {
        .func = {
            [OTMS_S_INIT] = task_vehicle_init_running,
            [OTMS_S_START] = task_vehicle_start_running,
            [OTMS_S_ASSERT_RUN] = task_vehicle_assert_running,
            [OTMS_S_RUNNING] = task_vehicle_running,
            [OTMS_S_POST_RUN] = task_vehicle_post_running,
            [OTMS_S_STOP] = task_vehicle_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_CAN
    [TASK_MODULE_CAN] = {
        .func = {
            [OTMS_S_INIT] = task_can_init_running,
            [OTMS_S_START] = task_can_start_running,
            [OTMS_S_ASSERT_RUN] = task_can_assert_running,
            [OTMS_S_RUNNING] = task_can_running,
            [OTMS_S_POST_RUN] = task_can_post_running,
            [OTMS_S_STOP] = task_can_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_KEY
    [TASK_MODULE_KEY] = {
        .func = {
            [OTMS_S_INIT] = task_key_init_running,
            [OTMS_S_START] = task_key_start_running,
            [OTMS_S_ASSERT_RUN] = task_key_assert_running,
            [OTMS_S_RUNNING] = task_key_running,
            [OTMS_S_POST_RUN] = task_key_post_running,
            [OTMS_S_STOP] = task_key_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_BLE
    [TASK_MODULE_BLE] = {
        .func = {
            [OTMS_S_INIT] = task_ble_init_running,
            [OTMS_S_START] = task_ble_start_running,
            [OTMS_S_ASSERT_RUN] = task_ble_assert_running,
            [OTMS_S_RUNNING] = task_ble_running,
            [OTMS_S_POST_RUN] = task_ble_post_running,
            [OTMS_S_STOP] = task_ble_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_4G
    [TASK_MODULE_4G] = {
        .func = {
            [OTMS_S_INIT] = task_4g_init_running,
            [OTMS_S_START] = task_4g_start_running,
            [OTMS_S_ASSERT_RUN] = task_4g_assert_running,
            [OTMS_S_RUNNING] = task_4g_running,
            [OTMS_S_POST_RUN] = task_4g_post_running,
            [OTMS_S_STOP] = task_4g_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_BT_MUSIC
    [TASK_MODULE_BT] = {
        .func = {
            [OTMS_S_INIT] = task_bt_init_running,
            [OTMS_S_START] = task_bt_start_running,
            [OTMS_S_ASSERT_RUN] = task_bt_assert_running,
            [OTMS_S_RUNNING] = task_bt_running,
            [OTMS_S_POST_RUN] = task_bt_post_running,
            [OTMS_S_STOP] = task_bt_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_BAFANG
    [TASK_MODULE_BAFANG] = {
        .func = {
            [OTMS_S_INIT] = task_bfang_ptl_init_running,
            [OTMS_S_START] = task_bfang_ptl_start_running,
            [OTMS_S_ASSERT_RUN] = task_bfang_ptl_assert_running,
            [OTMS_S_RUNNING] = task_bfang_ptl_running,
            [OTMS_S_POST_RUN] = task_bfang_ptl_post_running,
            [OTMS_S_STOP] = task_bfang_ptl_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_LING_HUI_LIION2
    [TASK_MODULE_LING_HUI_LIION2] = {
        .func = {
            [OTMS_S_INIT] = task_lhl2_ptl_init_running,
            [OTMS_S_START] = task_lhl2_ptl_start_running,
            [OTMS_S_ASSERT_RUN] = task_lhl2_ptl_assert_running,
            [OTMS_S_RUNNING] = task_lhl2_ptl_running,
            [OTMS_S_POST_RUN] = task_lhl2_ptl_post_running,
            [OTMS_S_STOP] = task_lhl2_ptl_stop_running,
        },
    },
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_UPDATE
    [TASK_MODULE_UPDATE_MCU] = {
        .func = {
            [OTMS_S_INIT] = task_update_init_running,
            [OTMS_S_START] = task_update_start_running,
            [OTMS_S_ASSERT_RUN] = task_update_assert_running,
            [OTMS_S_RUNNING] = task_update_running,
            [OTMS_S_POST_RUN] = task_update_post_running,
            [OTMS_S_STOP] = task_update_stop_running,
        },
    },
#endif
};

upf_module_info_t upf_module_info[_UPF_MODULE_MAX_];

