/*******************************************************************************
 * @file     octopus_system.c
 * @brief    Implements the system control logic for managing states,
 *           message handling, and UART communication in an embedded application.
 *
 * This source file is responsible for initializing and controlling the system's
 * state machine. It handles incoming and outgoing messages through a UART
 * interface, processes system-level events, and manages the power state
 * and handshake procedures with both the MCU and external applications.
 *
 * The code uses a modular design to interface with other system components,
 * ensuring flexibility and scalability.
 *
 * @version  1.0.0
 * @date     2024-12-11
 * @author   Octopus Team
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "octopus_gpio.h"
#include "octopus_system.h"
#include "octopus_flash.h"
#include "octopus_uart_hal.h"
/*******************************************************************************
 * Debug Switch Macros
 * Define debug levels or other switches as required.
 ******************************************************************************/

/*******************************************************************************
 * MACROS
 * The following macros define key IDs and their respective actions.
 */

/*******************************************************************************
 * Local Function Declarations
 * Declare static functions used only within this file.
 ******************************************************************************/
static bool system_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool system_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

void system_power_on_off(bool onoff);
void system_gpio_power_onoff(bool onoff);
void system_power_onoff_auto(void);
/*******************************************************************************
 * Global Variables
 * Define variables accessible across multiple files if needed.
 ******************************************************************************/

/*******************************************************************************
 * Local Variables
 * Define static variables used only within this file.
 ******************************************************************************/
static mb_state_t lt_mb_state;         // Current state of the system
static uint8_t l_u8_mpu_status = 0;    // Tracks the status of the MPU
static uint8_t l_u8_power_off_req = 0; // Tracks if a power-off request is pending
static uint32_t l_t_msg_wait_10_timer; // Timer for 10 ms message waiting period
static uint32_t l_t_msg_lowpower_wait_timer;
static uint32_t l_t_msg_booting_wait_timer;
/*******************************************************************************
 * Global Function Implementations
 ******************************************************************************/

/**
 * @brief Initializes the system for running.
 *
 * This function registers the system module with the communication layer
 * and transitions the system task to an invalid state.
 */
void task_system_init_running(void)
{
    LOG_LEVEL("task_system_init_running\r\n");
    OTMS(TASK_MODULE_SYSTEM, OTMS_S_INVALID);

    // ptl_register_module(P2M_MOD_DEBUG, debug_send_handler, debug_receive_handler);
#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    ptl_register_module(MCU_TO_SOC_MOD_SYSTEM, system_send_handler, system_receive_handler);
#elif defined(TASK_MANAGER_STATE_MACHINE_SOC)
    ptl_register_module(SOC_TO_MCU_MOD_SYSTEM, system_send_handler, system_receive_handler);
#else
    ptl_register_module(MCU_TO_SOC_MOD_SYSTEM, system_send_handler, system_receive_handler);
#endif
}

/**
 * @brief Starts the system.
 *
 * This function transitions the system task to the "assert run" state.
 */
void task_system_start_running(void)
{
    LOG_LEVEL("task_system_start_running\r\n");
    OTMS(TASK_MODULE_SYSTEM, OTMS_S_ASSERT_RUN);
}

/**
 * @brief Asserts the system is running.
 *
 * This function starts timers, requests the system to start running,
 * and transitions the system task to the running state.
 */
void task_system_assert_running(void)
{
    StartTickCounter(&l_t_msg_wait_10_timer);

#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    ptl_reqest_running(MCU_TO_SOC_MOD_SYSTEM);
#elif defined(TASK_MANAGER_STATE_MACHINE_SOC)
    ptl_reqest_running(SOC_TO_MCU_MOD_SYSTEM);
#endif

    OTMS(TASK_MODULE_SYSTEM, OTMS_S_RUNNING);
}

/**
 * @brief Main system running state handler.
 *
 * Processes system messages and handles events like power on/off and device events.
 */
void task_system_running(void)
{
    if (GetTickCounter(&l_t_msg_wait_10_timer) < 5)
        return;
    StartTickCounter(&l_t_msg_wait_10_timer);

    #ifdef TASK_MANAGER_STATE_MACHINE_MCU
    if (lt_mb_state == MB_POWER_ST_BOOTING)
    {
        if (GetTickCounter(&l_t_msg_booting_wait_timer) > 3000)
        {
            StopTickCounter(&l_t_msg_booting_wait_timer);
            lt_mb_state = MB_POWER_ST_ON;
        }
    }
    else if (lt_mb_state == MB_POWER_ST_LOWPOWER)
    {
        if (GetTickCounter(&l_t_msg_lowpower_wait_timer) > 8000)
        {
            task_manager_stop();
            enter_sleep_mode();
            lt_mb_state = MB_POWER_ST_BOOTING;
            StartTickCounter(&l_t_msg_booting_wait_timer);
            task_manager_start();
            system_gpio_power_onoff(true);
            StopTickCounter(&l_t_msg_lowpower_wait_timer);
        }
    }
   #endif

    Msg_t *msg = get_message(TASK_MODULE_SYSTEM);
    if (msg->msg_id == NO_MSG)
        return;

    switch (msg->msg_id)
    {
    case MSG_OTSM_DEVICE_NORMAL_EVENT:
        break;

    case MSG_OTSM_DEVICE_ACC_EVENT:
        break;

    case MSG_OTSM_DEVICE_POWER_EVENT:
        LOG_LEVEL("Got Event MSG_DEVICE_POWER_EVENT\r\n");
        if (msg->param1 == CMD_MODSYSTEM_POWER_ON)
            system_gpio_power_onoff(true);
        else if (msg->param1 == CMD_MODSYSTEM_POWER_OFF)
            system_gpio_power_onoff(false);
        else
            system_power_onoff_auto();
        break;

    case MSG_OTSM_DEVICE_BLE_EVENT:
        if (msg->param1 == MSG_OTSM_CMD_BLE_PAIR_ON)
        {
            LOG_LEVEL("MSG_OTSM_DEVICE_BLE_EVENT notify ble to enable pair mode\r\n");
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_SYSTEM, msg->param1, msg->param2);
        }
		else 
		{
		    send_message(TASK_MODULE_PTL_1, SOC_TO_MCU_MOD_SYSTEM, msg->param1, msg->param2);
		}
        break;
    }
}

void task_system_post_running(void)
{
}

void task_system_stop_running(void)
{
    OTMS(TASK_MODULE_SYSTEM, OTMS_S_INVALID);
}

/*******************************************************************************
 * FUNCTION: system_send_handler
 *
 * DESCRIPTION:
 * Handles the sending of system-related commands based on the frame type and parameters.
 * This function processes commands for system, setup, and app modules and sends appropriate responses.
 *
 * PARAMETERS:
 * - frame_type: Type of the frame (MCU_TO_SOC_MOD_SYSTEM, SOC_TO_MCU_MOD_SYSTEM, MCU_TO_SOC_MOD_SETUP).
 * - param1: Command identifier.
 * - param2: Additional parameter for the command.
 * - buff: Pointer to the buffer where the frame should be built.
 *
 * RETURNS:
 * - true if the command was processed successfully, false otherwise.
 ******************************************************************************/
bool system_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    MY_ASSERT(buff);      // Ensure the buffer is valid
    uint8_t tmp[8] = {0}; // Temporary buffer for command parameters

    // Handle commands for MCU_TO_SOC_MOD_SYSTEM frame type
    if (MCU_TO_SOC_MOD_SYSTEM == frame_type)
    {
        switch (param1)
        {
        case CMD_MODSYSTEM_HANDSHAKE:
            tmp[0] = 0;
            tmp[1] = 0;
            LOG_LEVEL("system handshake frame_type=%02x param1=%02x param2=%02x\n", frame_type, tmp[0], tmp[1]);
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, tmp, 2, buff);
            return true;

        case CMD_MODSYSTEM_ACC_STATE:
            // Acknowledgement, no additional action needed
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_ACC_STATE, tmp, 1, buff);
            return true;

        case CMD_MODSYSTEM_POWER_ON:
            // Acknowledgement, no additional action needed
            tmp[0] = 0;
            tmp[1] = 0;
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_POWER_ON, tmp, 2, buff);
            return true;
        case CMD_MODSYSTEM_POWER_OFF:
            // Acknowledgement, no additional action needed
            tmp[0] = 0;
            tmp[1] = 0;
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_POWER_OFF, tmp, 2, buff);
            return true;

        case CMD_MODSETUP_UPDATE_TIME:
            // Update time command, sending date and time values
            tmp[0] = 23; // year
            tmp[1] = 3;  // month
            tmp[2] = 25; // day
            tmp[3] = 8;  // hour
            tmp[4] = 55; // minute
            tmp[5] = 0;  // second
            ptl_build_frame(MCU_TO_SOC_MOD_SETUP, CMD_MODSETUP_UPDATE_TIME, tmp, 6, buff);
            return true;

        case MSG_OTSM_CMD_BLE_PAIR_ON:
            tmp[0] = 0;
            tmp[1] = 0;
            LOG_LEVEL("MSG_OTSM_CMD_BLE_PAIR_ON \r\n");
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, (ptl_frame_cmd_t)MSG_OTSM_CMD_BLE_PAIR_ON, tmp, 2, buff);
            hal_com_uart_send_buffer_3(buff->buff, buff->size);
            return false;

        default:
            break;
        }
    }
    // Handle commands for SOC_TO_MCU_MOD_SYSTEM frame type
    else if (SOC_TO_MCU_MOD_SYSTEM == frame_type)
    {
        switch (param1)
        {
        case CMD_MODSYSTEM_HANDSHAKE:
            tmp[0] = 0;
            tmp[1] = 0;
            LOG_LEVEL("system handshake param1=%02x param2=%02x\n", tmp[0], tmp[1]);
            ptl_build_frame(SOC_TO_MCU_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, tmp, 2, buff);
            return true;

        case CMD_MODSYSTEM_APP_STATE:
            tmp[0] = l_u8_mpu_status; // Send MPU status
            tmp[1] = 0x01;            // Additional status byte
            ptl_build_frame(SOC_TO_MCU_MOD_SYSTEM, CMD_MODSYSTEM_APP_STATE, tmp, 2, buff);
            return true;
        case MSG_OTSM_CMD_BLE_CONNECTED:
        case MSG_OTSM_CMD_BLE_DISCONNECTED:
            tmp[0] = param1; // Send MPU status
            tmp[1] = param2; // Additional status byte
            ptl_build_frame(SOC_TO_MCU_MOD_SYSTEM, (ptl_frame_cmd_t)param1, tmp, 2, buff);
            return true;
        default:
            break;
        }
    }
    return false; // Command not processed
}

/*******************************************************************************
 * FUNCTION: system_receive_handler
 *
 * DESCRIPTION:
 * Handles the reception of system-related commands based on the payload and sends appropriate responses.
 *
 * PARAMETERS:
 * - payload: Pointer to the received payload data.
 * - ackbuff: Pointer to the buffer where the acknowledgment frame will be built.
 *
 * RETURNS:
 * - true if the command was processed successfully, false otherwise.
 ******************************************************************************/
bool system_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    MY_ASSERT(payload); // Ensure payload is valid
    MY_ASSERT(ackbuff); // Ensure acknowledgment buffer is valid
    uint8_t tmp;        // Temporary variable for holding command data

    // Handle received commands for MCU_TO_SOC_MOD_SYSTEM frame type
    if (MCU_TO_SOC_MOD_SYSTEM == payload->frame_type)
    {
        switch (payload->frame_cmd)
        {
        case CMD_MODSYSTEM_HANDSHAKE:
            LOG_LEVEL("system handshake from mcu payload->frame_type=%02x\r\n", payload->frame_type);
            // ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, (uint8_t *)VER_STR, sizeof(VER_STR), ackbuff);
            return false;

        case CMD_MODSYSTEM_ACC_STATE:
            LOG_LEVEL("CMD_MODSYSTEM_ACC_STATE\r\n");
            // ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, (uint8_t *)VER_STR, sizeof(VER_STR), ackbuff);
            return false;

        case CMD_MODSYSTEM_APP_STATE:
            LOG_LEVEL("CMD_MODSYSTEM_APP_STATE l_u8_mpu_status = %d\r\n", payload->data[0]);
            tmp = 0x01;
            ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_APP_STATE, &tmp, 1, ackbuff);
            return true;

        case CMD_MODSYSTEM_POWER_ON:
            LOG_LEVEL("got CMD_MODSYSTEM_POWER_ON from mcu\r\n");
            system_power_on_off(true);
            return false;
        case CMD_MODSYSTEM_POWER_OFF:
            LOG_LEVEL("got CMD_MODSYSTEM_POWER_OFF from mcu\r\n");
            system_power_on_off(false);
            return false;
        case CMD_MODSETUP_UPDATE_TIME:
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_SETUP, CMD_MODSETUP_UPDATE_TIME, &tmp, 1, ackbuff);
            return false;

        case CMD_MODSETUP_SET_TIME:
            tmp = 0x01;
            ptl_build_frame(MCU_TO_SOC_MOD_SETUP, CMD_MODSETUP_SET_TIME, &tmp, 1, ackbuff);
            return true;

        case CMD_MODSETUP_KEY:
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_SETUP, CMD_MODSETUP_KEY, &tmp, 1, ackbuff);
            return false;
        case MSG_OTSM_CMD_BLE_PAIR_ON:
        case MSG_OTSM_CMD_BLE_PAIR_OFF:
            send_message(TASK_MODULE_BLE, MSG_OTSM_DEVICE_BLE_EVENT, MSG_OTSM_CMD_BLE_PAIR_ON, 0);
            return false;
        default:
            break;
        }
    }

    if (SOC_TO_MCU_MOD_SYSTEM == payload->frame_type)
    {
        switch (payload->frame_cmd)
        {
        case CMD_MODSYSTEM_HANDSHAKE:

            LOG_LEVEL("system handshake from soc payload->frame_type=%02x\r\n", payload->frame_type);
            // ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, (uint8_t *)VER_STR, sizeof(VER_STR), ackbuff);
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_CARINFOR, CMD_MOD_CARINFOR_INDICATOR, 0); // after got handshake then send indicate respond

            return false;
        case MSG_OTSM_CMD_BLE_CONNECTED:
            if (!is_power_on())
            {
                LOG_LEVEL("system got MSG_OTSM_CMD_BLE_CONNECTED prameter=%02x\r\n", payload->data[0]);
                system_gpio_power_onoff(true);
            }
            /// if (is_power_on())
            ///{
            ///     //tmp = 0x01;
            ///     //ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_POWER_ON, &tmp, 1, ackbuff);
            ///     //return true;
            /// }
            break;
        case MSG_OTSM_CMD_BLE_DISCONNECTED:

            LOG_LEVEL("system got MSG_OTSM_CMD_BLE_DISCONNECTED prameter=%02x\r\n", payload->data[1]);
            if (payload->data[1] == CMD_MODSYSTEM_POWER_OFF)
            {
                system_gpio_power_onoff(false);
            }
            /// if (!is_power_on())
            ///{
            ///     //tmp = 0x01;
            ///     //ptl_build_frame(MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_POWER_OFF, &tmp, 1, ackbuff);
            ///     //return true;
            /// }
            break;
        default:
            break;
        }
    }

    return false; // Command not processed
}

/*******************************************************************************
 * FUNCTION: system_handshake_with_mcu
 *
 * DESCRIPTION:
 * Initiates a handshake with the MCU by sending the handshake command.
 ******************************************************************************/
void system_handshake_with_mcu(void)
{
    LOG_LEVEL("system send handshake data to xxx\r\n");
    send_message(TASK_MODULE_PTL_1, SOC_TO_MCU_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, 0);
}

/*******************************************************************************
 * FUNCTION: system_handshake_with_app
 *
 * DESCRIPTION:
 * Initiates a handshake with the app by sending the handshake command.
 ******************************************************************************/
void system_handshake_with_app(void)
{
    LOG_LEVEL("system send handshake data to xxx\r\n");
    send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_HANDSHAKE, 0);
}

/*******************************************************************************
 * FUNCTION: system_set_mpu_status
 *
 * DESCRIPTION:
 * Sets the MPU status and sends an application state update message.
 *
 * PARAMETERS:
 * - status: The status value to set for the MPU.
 ******************************************************************************/
void system_set_mpu_status(uint8_t status)
{
    LOG_LEVEL("system set mpu status=%d \r\n", status);
    l_u8_mpu_status = status;
    send_message(TASK_MODULE_SYSTEM, CMD_MODSYSTEM_APP_STATE, l_u8_mpu_status, l_u8_mpu_status);
}

/*******************************************************************************
 * FUNCTION: system_get_mpu_status
 *
 * DESCRIPTION:
 * Retrieves the current MPU status.
 *
 * RETURNS:
 * - The current MPU status value.
 ******************************************************************************/
uint8_t system_get_mpu_status(void)
{
    return l_u8_mpu_status;
}

/*******************************************************************************
 * FUNCTION: system_get_power_off_req
 *
 * DESCRIPTION:
 * Checks if a power off request has been made.
 *
 * RETURNS:
 * - true if power off is requested, false otherwise.
 ******************************************************************************/
bool system_get_power_off_req(void)
{
    return l_u8_power_off_req;
}

/*******************************************************************************
 * FUNCTION: system_get_mb_state
 *
 * DESCRIPTION:
 * Retrieves the current ModBus state.
 *
 * RETURNS:
 * - The current ModBus state value.
 ******************************************************************************/
mb_state_t system_get_mb_state(void)
{
    return lt_mb_state;
}

void system_set_mb_state(mb_state_t status)
{
    lt_mb_state = status;
}

void system_reboot_system(void)
{
    LOG_LEVEL("System is rebooting...\n");
    int result = -1;
#ifdef PLATFORM_LINUX_RISC
    // Execute the reboot command
    result = system("reboot");
#endif
    // Check if the command executed successfully
    if (result == -1)
    {
        LOG_LEVEL("Failed to execute reboot command");
    }
    else
    {
        LOG_LEVEL("Reboot command executed successfully.\n");
    }
}

/**
 * @brief Handles the system power on/off state.
 *
 * Logs the power state and updates GPIO pins accordingly.
 *
 * @param onoff Boolean indicating power state (true for on, false for off).
 */
void system_power_on_off(bool onoff)
{
    system_reboot_system();
}

void system_gpio_power_onoff(bool onoff)
{
    if (!onoff)
    {
        LOG_LEVEL("power down f133 soc\r\n");
        send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_POWER_OFF, 0);
        flash_save_carinfor_meter();
        power_on_off(false);
        if (!is_power_on())
        {
            LOG_LEVEL("power down f133 soc succesfully\r\n");
            lt_mb_state = MB_POWER_ST_LOWPOWER;
            StartTickCounter(&l_t_msg_lowpower_wait_timer);
        }
    }
    else
    {
        LOG_LEVEL("power on f133 soc\r\n");
        lt_mb_state = MB_POWER_ST_ON;
        send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_SYSTEM, CMD_MODSYSTEM_POWER_ON, 0);
        power_on_off(true);
        if (is_power_on())
        {
            LOG_LEVEL("power on f133 soc succesfully\r\n");
        }
    }
}

void system_power_onoff_auto(void)
{
    if (is_power_on())
        system_gpio_power_onoff(false);
    else
        system_gpio_power_onoff(true);
}
