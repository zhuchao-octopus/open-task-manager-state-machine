/*******************************************************************************
 * @file        octopus_task_manager.c
 * @brief       Task manager module for managing multiple tasks and their states
 *
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
#include "octopus_task_manager.h" // Include task manager for scheduling tasks
#include "octopus_system.h"
#include "octopus_gpio.h"
#include "octopus_key.h"

#include "octopus_carinfor.h"
#include "octopus_ble.h"
#include "octopus_4g.h"
#include "octopus_bt.h"

#include "octopus_update_mcu.h"
#include "octopus_ipc.h"
#include "octopus_bafang.h"
#include "octopus_uart_ptl_1.h" // Include UART protocol header
#include "octopus_uart_ptl_2.h" // Include UART protocol header
/*******************************************************************************
 * MACROS
 ******************************************************************************/

/* Add any necessary macros here */

/*******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/* Define any custom types here */

/*******************************************************************************
 * CONSTANTS
 ******************************************************************************/

/** Static configuration for all tasks in the OTMS. */
const static otms_t task_module_config_table[TASK_MODULE_MAX_NUM] = {

    [TASK_MODULE_PTL_1] = {
        .func = {
            [OTMS_S_INIT] = ptl_init_running,
            [OTMS_S_START] = ptl_start_running,
            [OTMS_S_ASSERT_RUN] = ptl_assert_running,
            [OTMS_S_RUNNING] = ptl_running,
            [OTMS_S_POST_RUN] = ptl_post_running,
            [OTMS_S_STOP] = ptl_stop_running,
        },
    },

#ifdef TASK_MANAGER_STATE_MACHINE_PTL2
    [TASK_MODULE_PTL_2] = {
        .func = {
            [OTMS_S_INIT] = ptl_2_init_running,
            [OTMS_S_START] = ptl_2_start_running,
            [OTMS_S_ASSERT_RUN] = ptl_2_assert_running,
            [OTMS_S_RUNNING] = ptl_2_running,
            [OTMS_S_POST_RUN] = ptl_2_post_running,
            [OTMS_S_STOP] = ptl_2_stop_running,
        },
    },
#endif

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
		
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
    [TASK_MODULE_CAR_INFOR] = {
        .func = {
            [OTMS_S_INIT] = task_carinfo_init_running,
            [OTMS_S_START] = task_carinfo_start_running,
            [OTMS_S_ASSERT_RUN] = task_carinfo_assert_running,
            [OTMS_S_RUNNING] = task_carinfo_running,
            [OTMS_S_POST_RUN] = task_carinfo_post_running,
            [OTMS_S_STOP] = task_carinfo_stop_running,
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

#ifdef TASK_MANAGER_STATE_MACHINE_BT
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

#ifdef TASK_MANAGER_STATE_MACHINE_BAFANG
    [TASK_MODULE_PTL_BAFANG] = {
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

#ifdef TASK_MANAGER_STATE_MACHINE_IPC_SOCKET
    [TASK_MODULE_IPC_SOCKET] = {
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
};
/* Add any constants here */

/*******************************************************************************
 * LOCAL FUNCTION DECLARATIONS
 ******************************************************************************/

/**
 * @brief Executes the state-specific function for a given task.
 *
 * @param TASK_MODULE ID of the task to execute.
 * @param state   State to execute for the task.
 */
inline static void otms_exec_state(otms_id_t TASK_MODULE, otms_state_t state);

/*******************************************************************************
 * GLOBAL VARIABLES
 ******************************************************************************/

/* Add any global variables here */

/*******************************************************************************
 * STATIC VARIABLES
 ******************************************************************************/

/**
 * @brief Array to store the current state of each task.
 */
static otms_state_t otms_task_state[TASK_MODULE_MAX_NUM];

/*******************************************************************************
 * EXTERNAL VARIABLES
 ******************************************************************************/

/* Add any external variables here */

/*******************************************************************************
 * FUNCTION IMPLEMENTATIONS
 ******************************************************************************/

/**
 * @brief Gets the task manager configuration.
 *
 * @return Pointer to the task manager configuration.
 */
const otms_t *otms_get_config(void)
{
    return task_module_config_table;
}

/**
 * @brief Initializes the task manager and sets all tasks to the INIT state.
 */
void task_manager_init(void)
{
    otms_id_t i;
    for (i = 0; i < TASK_MODULE_MAX_NUM; i++)
    {
        otms_exec_state(i, OTMS_S_INIT); // Set each task to the INIT state.
    }
}

/**
 * @brief Starts all tasks by transitioning them to the START state.
 */
void task_manager_start(void)
{
    otms_id_t i;
    for (i = 0; i < TASK_MODULE_MAX_NUM; i++)
    {
        otms_exec_state(i, OTMS_S_START); // Set each task to the START state.
    }
}

void task_manager_start_module(TaskModule_t TaskModule)
{
    otms_id_t i;
    for (i = 0; i < TASK_MODULE_MAX_NUM; i++)
    {
        if (i == TaskModule)
            otms_exec_state(i, OTMS_S_START); // Set each task to the START state.
    }
}
/**
 * @brief Stops all tasks by transitioning them to the STOP state.
 */
void task_manager_stop(void)
{
    otms_id_t i;

    for (i = 0; i < TASK_MODULE_MAX_NUM; i++)
    {
        otms_exec_state(i, OTMS_S_STOP); // Set each task to the STOP state.
    }
}

void task_manager_stop_except_1(TaskModule_t task_module1)
{
    otms_id_t i;

    for (i = 0; i < TASK_MODULE_MAX_NUM; i++)
    {
        if (i != task_module1)
            otms_exec_state(i, OTMS_S_STOP); // Set each task to the STOP state.
    }
}

void task_manager_stop_except_2(TaskModule_t task_module1, TaskModule_t task_module2)
{
    otms_id_t i;

    for (i = 0; i < TASK_MODULE_MAX_NUM; i++)
    {
        if (i != task_module1 && i != task_module2)
            otms_exec_state(i, OTMS_S_STOP); // Set each task to the STOP state.
    }
}

/**
 * @brief Runs the current state-specific function for all tasks.
 */
void task_manager_run(void)
{
    otms_id_t i;

    for (i = 0; i < TASK_MODULE_MAX_NUM; i++)
    {
        otms_exec_state(i, otms_task_state[i]); // Execute the current state for each task.
    }
}

/**
 * @brief Sets the state of a specific task.
 *
 * @param task_module ID of the task.
 * @param state   New state to set for the task.
 */
void otms_set_state(otms_id_t task_module, otms_state_t state)
{
    if (task_module < TASK_MODULE_MAX_NUM)
    {
        otms_task_state[task_module] = state; // Update the task state.
    }
    else
    {
        MY_ASSERT(0); // Invalid task ID.
    }
}

/**
 * @brief Gets the current state of a specific task.
 *
 * @param task_module ID of the task.
 * @return Current state of the task.
 */
otms_state_t otms_get_state(otms_id_t task_module)
{
    otms_state_t state;

    if (task_module < TASK_MODULE_MAX_NUM)
    {
        state = otms_task_state[task_module];
    }
    else
    {
        state = OTMS_S_INVALID; // Invalid state.
        MY_ASSERT(0);           // Invalid task ID.
    }
    return state;
}

/**
 * @brief Handles transitions when entering the RUN state.
 */
void otms_on_enter_run(void)
{
    otms_id_t i = 0;

    for (i = 0; i < TASK_MODULE_MAX_NUM; i++)
    {
        if (otms_get_state(i) > OTMS_S_POST_RUN)
        {
            otms_set_state(i, OTMS_S_START); // Reset tasks to the START state.
        }
    }
}

/**
 * @brief Handles transitions when exiting the POST_RUN state.
 */
void otms_on_exit_post_run(void)
{
    otms_id_t i = 0;

    for (i = 0; i < TASK_MODULE_MAX_NUM; i++)
    {
        otms_set_state(i, OTMS_S_STOP); // Transition tasks to the STOP state.
    }
}

/**
 * @brief Executes the state-specific function for a given task.
 *
 * @param task_module ID of the task to execute.
 * @param state   State to execute for the task.
 */
static void otms_exec_state(otms_id_t task_module, otms_state_t state)
{
    const otms_t *cfg = otms_get_config();

    if ((task_module < TASK_MODULE_MAX_NUM) && (NULL != cfg) && (OTMS_S_COUNT > state) && (OTMS_S_INVALID != state))
    {
        if (NULL != cfg[task_module].func[state])
        {
            cfg[task_module].func[state](); // Call the state-specific function.
        }
    }
}
