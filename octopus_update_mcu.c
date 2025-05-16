/*******************************************************************************
 * @file    octopus_mcu_update.c
 * @brief   MCU update management and reboot state machine implementation.
 *
 * This file contains the implementation for managing MCU firmware updates,
 * handling reboot sequences, and interacting with various hardware modules.
 *
 * @details
 * - Provides initialization and control functions for the MCU update task.
 * - Implements a state machine for managing reboot sequences.
 * - Handles communication with other system modules for update and reboot processes.
 * - Offers support for flash memory operations during firmware updates.
 *
 * @version  1.0.0
 * @date     2024-12-09
 * @author   Octopus Team
 *
 * @note    Ensure all dependent modules are correctly initialized before using
 *          the functions in this file.
 */

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"	// Include platform-specific hardware details
#include "octopus_update_mcu.h" // Include header for MCU update management
#include "octopus_flash.h"		// Include flash memory handling utilities

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * TYPEDEFS
 */
// Define enumeration for MCU reboot state machine states
typedef enum
{
	MCU_UPDATE_STATE_IDLE,
	MCU_UPDATE_STATE_CHECK,
	MCU_UPDATE_STATE_INIT,
	MCU_UPDATE_STATE_ERASE,
	MCU_UPDATE_STATE_RECEIVING,
	MCU_UPDATE_STATE_UPDATING,
	MCU_UPDATE_STATE_EXIT,
	MCU_UPDATE_STATE_COMPLETE,
	MCU_UPDATE_STATE_ERROR,
} mcu_update_state_t;

// Define a buffer structure used for flash programming during firmware updates
typedef struct
{
	uint32_t addr;	  // Address in flash memory to write data
	uint8_t buff[48]; // Data buffer for temporary storage
	uint8_t length;	  // Number of valid bytes in the buffer
	uint8_t state;
	uint8_t error_count;
} program_buf_t;

static program_buf_t lt_mcu_program_buf;

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * STATIC VARIABLES
 */
// Declare static variables for managing reboot process

/*******************************************************************************
 * EXTERNAL VARIABLES
 */

/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
// Function declarations
static bool update_module_send_handler(ptl_frame_type_t module, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff); // Handle outgoing messages
static bool update_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);						  // Handle incoming messages
static void mcu_update_state_proc(void);

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */

/**
 * @brief Initialize the MCU update module and register communication handlers.
 */
void app_update_mcu_init_running(void)
{
	ptl_register_module(MCU_TO_SOC_MOD_UPDATE, update_module_send_handler, update_module_receive_handler); // Register handlers for module communication
	OTMS(TASK_ID_UPDATE_MCU, OTMS_S_INVALID);															   // Set the task state to invalid (not running)
	LOG_LEVEL("OTMS app_update_mcu_init_running\r\n");
}

/**
 * @brief Start the MCU update task by setting it to the assert run state.
 */
void app_update_mcu_start_running(void)
{
	OTMS(TASK_ID_UPDATE_MCU, OTMS_S_ASSERT_RUN); // Assert task state to running
	LOG_LEVEL("OTMS app_update_mcu_start_running\r\n");
	lt_mcu_program_buf.state = MCU_UPDATE_STATE_IDLE;
}

/**
 * @brief Assert the MCU update task to a running state, ensuring it is active.
 */
void app_update_mcu_assert_running(void)
{
	ptl_reqest_running(MCU_TO_SOC_MOD_UPDATE); // Request module to be active
	OTMS(TASK_ID_UPDATE_MCU, OTMS_S_RUNNING);  // Set task state to running
}

/**
 * @brief Main task function for managing the MCU update process.
 */
void app_update_mcu_running(void)
{
	mcu_update_state_proc(); // Call the reboot state machine processing function
}

/**
 * @brief Post-task actions after the MCU update task completes.
 */
void app_update_mcu_post_running(void)
{
	ptl_release_running(MCU_TO_SOC_MOD_UPDATE);	 // Release the active module
	OTMS(TASK_ID_UPDATE_MCU, OTMS_S_ASSERT_RUN); // Reset task state to assert run
}

/**
 * @brief Stop the MCU update task by invalidating its state.
 */
void app_update_mcu_stop_running(void)
{
	OTMS(TASK_ID_UPDATE_MCU, OTMS_S_INVALID); // Set the task state to invalid
}

/*******************************************************************************
 * LOCAL FUNCTIONS IMPLEMENTATION
 */
// Function to handle sending update module requests based on the module type and parameters
bool update_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
	MY_ASSERT(buff);
	uint8_t tmp[2] = {0};
	// PRINT("update_module_send_handler  MOD %02x  CMD %02x PRARM %04x\n", module, cmd, param);
	if (MCU_TO_SOC_MOD_UPDATE == frame_type)
	{
		switch (param1)
		{
		case CMD_MODUPDATE_UPDATE_FW_STATE:
		{
			tmp[0] = param1;
			tmp[1] = lt_mcu_program_buf.state; // ack success
			ptl_build_frame(MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_UPDATE_FW_STATE, tmp, 2, buff);
			return true;
		}
		case CMD_MODUPDATE_SEND_FW_DATA:
		{
			// bootloader_iap_mode = 1;
			tmp[0] = param1; // ack success
			tmp[1] = param2; // ack success
			ptl_build_frame(MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_SEND_FW_DATA, tmp, 2, buff);
			return true;
		}
		default:
			break;
		}
	}
	return false;
}

void MCU_Print_Program_Data(uint32_t address, uint8_t *buff, uint8_t length)
{
	LOG_LEVEL("MCU_UPDATE addr:%08x count:%02d ", address, length);
	LOG_BUFF(buff, length);
}

void MCU_Print_Receive_Data(uint32_t address, uint8_t *buff, uint8_t length)
{
	LOG_LEVEL("MCU_RECEIV addr:%08x count:%02d ", address, length);
	LOG_BUFF(buff, length);
}
// Function to handle receiving update module requests, process the payload, and send acknowledgment.
bool update_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
	MY_ASSERT(payload);
	MY_ASSERT(ackbuff);
	uint8_t tmp[2] = {0};
	if (SOC_TO_MCU_MOD_UPDATE == payload->frame_type)
	{

		switch (payload->cmd)
		{
		case CMD_MODUPDATE_CHECK_FW_STATE:
		case CMD_MODUPDATE_UPDATE_FW_STATE: // second
		{
			LOG_LEVEL("CMD_MODUPDATE_UPDATE_FW_STATE \n");
			tmp[0] = lt_mcu_program_buf.state;
			tmp[1] = lt_mcu_program_buf.state;
			ptl_build_frame(MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_CHECK_FW_STATE, tmp, 2, ackbuff);
			return true;
		}
		case CMD_MODUPDATE_ENTER_FW_UPDATE: // first
		{
			LOG_LEVEL("BootLoaer CMD_MODUPDATE_ENTER_FW_UPDATE \n");
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_INIT;
			// tmp = lt_mcu_program_buf.state;
			// ptl_build_frame(MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_ENTER_FW_UPDATE, &tmp, 1, ackbuff);
			return false;
		}
		case CMD_MODUPDATE_EXIT_FW_UPDATE:
		{
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_EXIT;
			// tmp[0] = lt_mcu_program_buf.state;
			// tmp[1] = lt_mcu_program_buf.state;
			// ptl_build_frame(MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_EXIT_FW_UPDATE, tmp, 2, ackbuff);
			LOG_LEVEL("\nCMD_MODUPDATE_EXIT_FW_UPDATE \n");
			return true;
		}
		case CMD_MODUPDATE_SEND_FW_DATA:
		{
			lt_mcu_program_buf.addr = MK_DWORD(MK_WORD(payload->data[0], payload->data[1]), MK_WORD(payload->data[2], payload->data[3]));
			lt_mcu_program_buf.length = payload->data[4];

			for (int i = 0; i < lt_mcu_program_buf.length; i++)
			{
				lt_mcu_program_buf.buff[i] = payload->data[5 + i];
			}

			MCU_Print_Receive_Data(lt_mcu_program_buf.addr, lt_mcu_program_buf.buff, lt_mcu_program_buf.length);

			if (lt_mcu_program_buf.state == MCU_UPDATE_STATE_RECEIVING)
				lt_mcu_program_buf.state = MCU_UPDATE_STATE_UPDATING;
			else
				lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERROR;
			return false;
		}
		case CMD_MODUPDATE_REBOOT:
		{
			LOG_LEVEL("CMD_MODUPDATE_REBOOT\n");
			return true;
		}
		default:
			break;
		}
	}
	return false;
}

// Function to process the MCU reboot state machine. It handles different stages of the reboot process.
static void mcu_update_state_proc(void)
{

	switch (lt_mcu_program_buf.state)
	{
	case MCU_UPDATE_STATE_IDLE:
	case MCU_UPDATE_STATE_CHECK:
		break;
	case MCU_UPDATE_STATE_INIT:
	{
		LOG_LEVEL("MCU_UPDATE_STATE_INIT \n");
		// send_message(TASK_ID_PTL_1, MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_UPDATE_FW_STATE, 0x01);
		lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERASE;
		break;
	}
	case MCU_UPDATE_STATE_ERASE:
	{
		send_message(TASK_ID_PTL_1, MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_UPDATE_FW_STATE, 0x02);
		uint32_t erase_count = Flash_erase_user_app_arear();

		if (erase_count == MAIN_APP_BLOCK_COUNT)
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_RECEIVING;
		else
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERROR;

		break;
	}
	case MCU_UPDATE_STATE_RECEIVING:

		break;
	case MCU_UPDATE_STATE_UPDATING:
	{
		// LOG_LEVEL("MCU_UPDATE_STATE_UPDATING \n");
		lt_mcu_program_buf.error_count = 0;
		uint32_t writed_count = 16;
	RETRY_PROGRAM:
#if 0
				__disable_irq();
				writed_count = FlashWriteBuffTo(lt_mcu_program_buf.addr, lt_mcu_program_buf.buff,lt_mcu_program_buf.length);
				__enable_irq();
#endif
		MCU_Print_Program_Data(lt_mcu_program_buf.addr, lt_mcu_program_buf.buff, lt_mcu_program_buf.length);

		if (writed_count == lt_mcu_program_buf.length)
		{
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_RECEIVING;
			send_message(TASK_ID_PTL_1, MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_SEND_FW_DATA, lt_mcu_program_buf.state);
			break;
		}

		lt_mcu_program_buf.error_count++;
		if (lt_mcu_program_buf.error_count >= 3)
		{
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERROR;
			break;
		}

		goto RETRY_PROGRAM;
	}

	case MCU_UPDATE_STATE_EXIT:
	case MCU_UPDATE_STATE_COMPLETE:
	{
		LOG_LEVEL("MCU_UPDATE_STATE_COMPLETE \n");
		break;
	}

	case MCU_UPDATE_STATE_ERROR:
	{
		LOG_LEVEL("MCU_UPDATE_ST_FAILED \n");
		// send_message(TASK_ID_PTL_1, MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_SEND_FW_DATA, 0);
		break;
	}
	}
}
