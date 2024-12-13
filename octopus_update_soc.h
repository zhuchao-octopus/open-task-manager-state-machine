/*******************************************************************************
 * @file    octopus_task_manager_update.h
 * @brief   Header file for the Octopus Task Manager Update module.
 *          This file declares the functions and types required for managing
 *          MCU firmware updates.
 *
 * @details This module provides functionality for initializing, starting,
 *          and monitoring the MCU firmware update process. It defines the
 *          update states and provides interfaces to query the update status
 *          and progress.
 *
 * @version  1.0.0
 * @date     2024-12-09
 * @author   Octopus Team

 ******************************************************************************/

#ifndef ___OCTOPUS_TASK_MANAGER_UPDATE_SOC_H___
#define ___OCTOPUS_TASK_MANAGER_UPDATE_SOC_H___

/*******************************************************************************
 * INCLUDE FILES
 * Include standard libraries and platform-specific headers.
 ******************************************************************************/
#include "octopus_platform.h"          // Platform-specific definitions
#include "octopus_system.h" 

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup APP_SETTING APP:SETTING
 * @brief    Application settings and configurations.
 * @{
 */

/*******************************************************************************
 * DEBUG SWITCH MACROS
 * Define macros to enable or disable debug functionality.
 ******************************************************************************/
#ifdef TASK_MANAGER_STATE_MACHINE_UPDATE
/*******************************************************************************
 * MACROS
 * Define commonly used macros for this module.
 ******************************************************************************/

/*******************************************************************************
 * TYPEDEFS
 * Define types used in the MCU update process.
 ******************************************************************************/

/*******************************************************************************
 * CONSTANTS
 * Define any module-specific constants.
 ******************************************************************************/

/*******************************************************************************
 * GLOBAL VARIABLES DECLARATION
 * Declare external variables used across the module.
 ******************************************************************************/

/*******************************************************************************
 * GLOBAL FUNCTIONS DECLARATION
 * Declare the public functions provided by this module.
 ******************************************************************************/

/**
 * @brief Initialize the update process.
 */
void app_update_soc_init_running(void);

/**
 * @brief Start the update process.
 */
void app_update_soc_start_running(void);

/**
 * @brief Assert and verify the update process is running.
 */
void app_update_soc_assert_running(void);

/**
 * @brief Handle the main logic for the update process.
 */
void app_update_soc_running(void);

/**
 * @brief Perform post-update operations.
 */
void app_update_soc_post_running(void);

/**
 * @brief Stop the update process.
 */
void app_update_soc_stop_running(void);

/**
 * @brief Confirm the update process.
 */
void app_update_confirm(void);

/**
 * @brief Get the time of the last confirmation.
 * @return The timestamp of the last confirmation.
 */
uint32_t app_update_get_confirm_time(void);

/**
 * @brief Get the total number of firmware lines.
 * @return The total number of lines in the firmware.
 */
uint32_t app_update_get_fw_total_line(void);

/**
 * @brief Get the current firmware line being processed.
 * @return The current line number.
 */
uint32_t app_update_get_fw_curr_line(void);

/**
 * @brief Get the error code from the update process.
 * @return The error code indicating the reason for failure.
 */
#ifdef TASK_MANAGER_STATE_MACHINE_SOC
uint32_t app_update_get_error_code(void);
#endif
/**
 * @brief Get the current status of the update process.
 * @return The current state of the MCU update.
 */
mcu_update_state_t app_update_get_status(void);

/**
 * @}
 * End of group APP_SETTING
 */

#ifdef __cplusplus
}
#endif

#endif

#endif // ___OCTOPUS_TASK_MANAGER_UPDATE_H___

