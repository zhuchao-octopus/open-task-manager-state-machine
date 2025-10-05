/*******************************************************************************
 * @file     octopus_task_manager_system.h
 * @brief    Provides system management functions for task manager, including
 *           version information, state machine management, and MCU update states.
 *
 * This header file defines the necessary API to manage the system state
 * within the Octopus Task Manager framework. It includes management of the
 * main control state machine, MCU update states, and platform-specific state
 * transitions for the system's MPU (Microprocessor Unit) and boot sequence.
 *
 * The file also includes macros for versioning and the system's state
 * definitions to aid in the initialization, startup, and state transitions of
 * the task manager. Additionally, the system status for various stages of
 * application execution is provided.
 *
 * @note     This file assumes the use of platform-specific configurations
 *           for the task manager's operation and MCU update procedures.
 *
 * @version  1.0.0
 * @date     2024-12-09
 * @author   Octopus Team
 *******************************************************************************/

#ifndef __OCTOPUS_TASK_MANAGER_SYSTEM_H__
#define __OCTOPUS_TASK_MANAGER_SYSTEM_H__

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "octopus_base.h" //  Base include file for the Octopus project.

/*******************************************************************************
 * MACROS
 ******************************************************************************/

/**
 * @brief Version information.
 */

/**
 * @brief System MPU (Microprocessor Unit) state definitions.
 */
#define SYSTEM_MPU_STATE_INIT (0x00)         /**< Initial state. */
#define SYSTEM_MPU_STATE_ENTER_PLAY (0x01)   /**< Entering playback animation. */
#define SYSTEM_MPU_STATE_ENTER_FINISH (0x02) /**< Playback animation complete. */
#define SYSTEM_MPU_STATE_COMPLETED (0x03)    /**< Loading completed. */
#define SYSTEM_MPU_STATE_LEAVE_PLAY (0x04)   /**< Exiting playback animation. */
#define SYSTEM_MPU_STATE_LEAVE_FINISH (0x05) /**< Exit animation complete. */

/*******************************************************************************
 * TYPEDEFS
 ******************************************************************************/

/**
 * @brief Mainboard (MB) state enumeration.
 */
typedef enum MB_POWER_STATE
{
    MB_POWER_ST_INIT = 0, /**< Initialization state. */
    MB_POWER_ST_BOOTING,  /**< Booting state. */

    MB_POWER_ST_LOWPOWER, /**< Low-power state. */
    MB_POWER_ST_STANDBY,  /**< Standby state. */

    MB_POWER_ST_ON, /**< Fully operational state. */
    MB_POWER_ST_STOP,
    MB_POWER_ST_SHUTDOWN, /**< Shutdown process. */
    MB_POWER_ST_OFF,      /**< Power-off state. */

} main_board_state_t;

typedef enum MCU_POWER_STATE
{
    MCU_POWER_ST_INIT = 0, /**< Initialization state. */
    MCU_POWER_ST_BOOTING,  /**< Booting state. */

    MCU_POWER_ST_LOWPOWER, /**< Low-power state. */
    MCU_POWER_ST_STANDBY,  /**< Standby state. */

    MCU_POWER_ST_ON, /**< Fully operational state. */
    MCU_POWER_ST_STOP,
    MCU_POWER_ST_SHUTDOWN, /**< Shutdown process. */
    MCU_POWER_ST_OFF,      /**< Power-off state. */

} mcu_state_t;

/*******************************************************************************
 * GLOBAL FUNCTIONS DECLARATION
 ******************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif
    /**
     * @brief Initialize the application system.
     */
    void task_system_init_running(void);

    /**
     * @brief Start running the application system.
     */
    void task_system_start_running(void);

    /**
     * @brief Assert and maintain the running state of the application system.
     */
    void task_system_assert_running(void);

    /**
     * @brief Execute the main running logic of the application system.
     */
    void task_system_running(void);

    /**
     * @brief Post-run procedures for the application system.
     */
    void task_system_post_running(void);

    /**
     * @brief Stop the application system.
     */
    void task_system_stop_running(void);

    /**
     * @brief Get the current MPU (Microprocessor Unit) status.
     * @return Current MPU status.
     */
    uint8_t system_get_mpu_status(void);

    /**
     * @brief Perform a synchronization request with the application layer.
     */
    void system_synchronize_with_app(void);

    /**
     * @brief Perform a synchronization request with the MCU (Microcontroller Unit).
     */
    void system_synchronize_with_mcu(void);

    void system_power_onoff(bool onoff);
    void system_reboot_soc(void);
    void system_set_mcu_status(mcu_state_t mcu_state);

    mcu_state_t system_get_mcu_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_TASK_MANAGER_SYSTEM_H__ */
