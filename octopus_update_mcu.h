/*******************************************************************************
 * @file octopus_task_manager_update_mcu.h
 * @brief Header file for MCU update task management in the Octopus platform.
 *
 * This file defines the interface for managing the MCU update task,
 * including state transitions and associated functionality.
 *
 * @version  1.0.0
 * @date 2024-12-12
 * @author   Octopus Team
 *******************************************************************************/
#ifndef ___OCTOPUS_TASK_MANAGER_UPDATE_MCU_H___
#define ___OCTOPUS_TASK_MANAGER_UPDATE_MCU_H___

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * \defgroup APP_SETTING Application: MCU Update Task Manager
 * @{
 */

/*******************************************************************************
 * DEBUG SWITCH MACROS
 *
 * Define macros to enable or disable debugging features for the MCU update
 * task manager. These macros are generally defined for debugging purposes
 * during development.
 */
#ifdef TASK_MANAGER_STATE_MACHINE_UPDATE
    /*******************************************************************************
     * MACROS
     *
     * Define any general macros required for the MCU update task manager.
     * These can include constants, helper macros, or flags.
     */

    /*******************************************************************************
     * TYPEDEFS
     *
     * Define data types specific to the MCU update task manager. These can include
     * enumerations, structures, or aliases for existing types.
     */

    /*******************************************************************************
     * CONSTANTS
     *
     * Define any constant values used in the MCU update task manager.
     * These may include default settings or configuration parameters.
     */

    /*******************************************************************************
     * GLOBAL VARIABLES DECLARATION
     *
     * Declare any global variables that are required for the MCU update
     * task manager. These variables will be defined in the implementation file.
     */

    /*******************************************************************************
     * GLOBAL FUNCTIONS DECLARATION
     *
     * Declare the functions used to manage the lifecycle of the MCU update task.
     */

    /**
     * @brief Initialize the MCU update task.
     *
     * This function is called during system initialization to prepare
     * the MCU update task for execution.
     */
    void app_update_mcu_init_running(void);

    /**
     * @brief Start the MCU update task.
     *
     * This function transitions the MCU update task to the start state and
     * performs any necessary initialization or resource allocation.
     */
    void app_update_mcu_start_running(void);

    /**
     * @brief Assert the running state of the MCU update task.
     *
     * This function checks the conditions for the MCU update task to run and
     * ensures that all prerequisites are met.
     */
    void app_update_mcu_assert_running(void);

    /**
     * @brief Execute the running state of the MCU update task.
     *
     * This function performs the main operations of the MCU update task
     * while it is in the running state.
     */
    void app_update_mcu_running(void);

    /**
     * @brief Perform post-run operations for the MCU update task.
     *
     * This function handles any required cleanup or preparations for
     * transitioning out of the running state.
     */
    void app_update_mcu_post_running(void);

    /**
     * @brief Stop the MCU update task.
     *
     * This function transitions the MCU update task to the stopped state
     * and releases any allocated resources.
     */
    void app_update_mcu_stop_running(void);

    /** @} end of group APP_SETTING */

#ifdef __cplusplus
}
#endif

#endif

#endif // ___OCTOPUS_TASK_MANAGER_UPDATE_MCU_H___
