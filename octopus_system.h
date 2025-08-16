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
#include "octopus_platform.h"
#include "driver_audio.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * MACROS
 ******************************************************************************/

/*******************************************************************************
* TYPEDEFS
******************************************************************************/
		typedef enum POWER_MANAGER_STATE {
				POWER_STATE_START_INIT = 0,     /**< Initialization state. */
				POWER_STATE_BOOTING,            /**< Booting process state. */

				POWER_STATE_ACC_ON,             /**< ACC is ON (engine/key ON). */
				POWER_STATE_ACC_OFF,            /**< ACC is OFF. */
				POWER_STATE_ACC_OFF_WAITING,        /**< Waiting for ACC stabilization. */
				POWER_STATE_ACC_OFF_WAITING_HOST,    /**< Waiting for host command (from SoC). */
			
				POWER_STATE_ENTER_LOWPOWER,     /**< Entering low-power mode. */
				POWER_STATE_ENTER_STANDBY,      /**< Entering standby (sleep) mode. */

				POWER_STATE_POWER_ON,           /**< System fully operational. */
				POWER_STATE_POWER_OFF,          /**< System shutting down or off. */
			
				POWER_STATE_NORMAL_RUNNING,     /**< System is running normally. */
        POWER_STATE_STANDBY,
			  POWER_STATE_SHUTDOWN,
		} mb_power_manager_state_t;

		typedef enum {
				MODULE_OFF = 0,  /**< Module is turned OFF. */
				MODULE_ON        /**< Module is turned ON. */
		} module_state_t;
				
		typedef struct {
			 // --- Power Management ---
			mb_power_manager_state_t power_state;              /**< Current power state. */

			// --- Peripheral Status ---
			module_state_t lcd_backlight;           /**< LCD backlight state. */
			module_state_t audio_amp;               /**< Audio amplifier state. */
			module_state_t gps_module;              /**< GPS module enable state. */
			module_state_t antenna_power;           /**< Antenna power state. */
			module_state_t led_indicator;           /**< LED status indicator. */	
		
			// --- Input Signal States ---
			uint8_t acc_signal;                     /**< ACC signal (1 = ON). */
			uint8_t brake_signal;                   /**< Brake signal. */
			uint8_t ill_signal;                     /**< Illumination signal. */
			uint8_t reverse_signal;                 /**< Reverse gear signal. */
			uint8_t tel_mute_signal;                /**< Telephone mute signal. */

			// --- Communication Status ---
			uint8_t uart_soc_connected;            /**< UART to SoC connected (1 = OK). */
			uint8_t ota_update_in_progress;        /**< OTA update flag. */

			// --- Runtime Info ---
			bool system_need_reset;
			bool host_is_sleeping;
			bool host_is_charging;
			bool force_kill_host;
			uint32_t acc_wait_time;
			uint32_t host_wait_time;
			uint32_t low_power_wait_timer;
			uint32_t uptime_ms;                    /**< System uptime in milliseconds. */
		} system_state_t;

		typedef struct {
			uint32_t magic_num;
			uint8_t radio_area;
			uint8_t brake_mode;
			uint8_t led_always_on;
			uint8_t led_r_level;
			uint8_t led_g_level;
			uint8_t led_b_level;
			uint8_t vcom_value;
			uint32_t acc_wait_timeout;
		} user_mata_infor_t;
    /*******************************************************************************
     * GLOBAL FUNCTIONS DECLARATION
     ******************************************************************************/

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
     * @brief Perform a handshake with the application layer.
     */
    void system_handshake_with_app(void);

    /**
     * @brief Perform a handshake with the MCU (Microcontroller Unit).
     */
    void system_handshake_with_mcu(void);

    /**
     * @brief Get the current mainboard state.
     * @return Current mainboard state.
     */
    mb_power_manager_state_t system_get_mb_state(void);

    void system_set_mb_state(mb_power_manager_state_t status);
    void system_power_on_off(bool onoff);
    void system_power_manager_init(void);
#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_TASK_MANAGER_SYSTEM_H__ */
