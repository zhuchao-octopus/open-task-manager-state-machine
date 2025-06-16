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
#include "octopus_system.h"
/*******************************************************************************
 * DEBUG SWITCH MACROS
 */

/*******************************************************************************
 * MACROS
 */
#ifdef TASK_MANAGER_STATE_MACHINE_UPDATE
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
	MCU_UPDATE_STATE_COMPLETE,
	MCU_UPDATE_STATE_EXIT,
	MCU_UPDATE_STATE_ERROR,
} mcu_update_state_t;

// Define a buffer structure used for flash programming during firmware updates
typedef struct
{
	uint32_t bank_slot;        // Indicates the firmware bank or partition being updated (e.g., bank A or B)
	uint32_t bank_address;
	uint32_t total_crc_32;     // Expected total CRC32 checksum for the complete firmware image (used for integrity verification)
	uint32_t total_length;     // Total expected length (in bytes) of the firmware image being programmed
	
	uint32_t r_address;        // Current flash memory address where the next chunk of data will be written
	uint8_t  r_buff[64];       // Temporary buffer for holding a chunk of data (up to 64 bytes) before writing to flash
	uint8_t  r_length;         // Number of valid data bytes currently stored in 'buff'
	
	uint8_t  state;            // State indicator for the programming process (e.g., idle, receiving, writing, error)
	uint8_t  error_count;      // Counts how many errors (e.g., CRC mismatch, timeout) have occurred during programming
	uint16_t f_count;    			 // Total number of data frame packets received successfully

} program_buf_t;

static program_buf_t lt_mcu_program_buf;

mcu_update_pragress_t mcu_pragress_status;
/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */
char update_path_filename[25]={0};
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
static bool update_send_handler(ptl_frame_type_t module, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff); // Handle outgoing messages
static bool update_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);						  // Handle incoming messages
static void mcu_update_state_proc(void);

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */

/**
 * @brief Initialize the MCU update module and register communication handlers.
 */
void task_update_init_running(void)
{
	ptl_register_module(MCU_TO_SOC_MOD_UPDATE, update_send_handler, update_receive_handler); // Register handlers for module communication
	OTMS(TASK_MODULE_UPDATE_MCU, OTMS_S_INVALID);											 // Set the task state to invalid (not running)
	LOG_LEVEL("task_update_mcu_init_running\r\n");
}

/**
 * @brief Start the MCU update task by setting it to the assert run state.
 */
void task_update_start_running(void)
{
	OTMS(TASK_MODULE_UPDATE_MCU, OTMS_S_ASSERT_RUN); // Assert task state to running
	LOG_LEVEL("task_update_mcu_start_running\r\n");
	lt_mcu_program_buf.state = MCU_UPDATE_STATE_IDLE;
	lt_mcu_program_buf.f_count = 0;
}

/**
 * @brief Assert the MCU update task to a running state, ensuring it is active.
 */
void task_update_assert_running(void)
{
	ptl_reqest_running(MCU_TO_SOC_MOD_UPDATE); // Request module to be active
	OTMS(TASK_MODULE_UPDATE_MCU, OTMS_S_RUNNING);  // Set task state to running
}

/**
 * @brief Main task function for managing the MCU update process.
 */
void task_update_running(void)
{
	mcu_update_state_proc(); // Call the reboot state machine processing function
}

/**
 * @brief Post-task actions after the MCU update task completes.
 */
void task_update_post_running(void)
{
	ptl_release_running(MCU_TO_SOC_MOD_UPDATE);	 		// Release the active module
	OTMS(TASK_MODULE_UPDATE_MCU, OTMS_S_ASSERT_RUN);// Reset task state to assert run
}

/**
 * @brief Stop the MCU update task by invalidating its state.
 */
void task_update_stop_running(void)
{
	LOG_LEVEL("_stop_running\r\n");
	OTMS(TASK_MODULE_UPDATE_MCU, OTMS_S_INVALID); 	// Set the task state to invalid
}

/*******************************************************************************
 * LOCAL FUNCTIONS IMPLEMENTATION
 */

void MCU_Print_Program_Data(uint32_t address, uint8_t *buff, uint8_t length)
{
	LOG_LEVEL("MCU_UPDATE addr:%08x length:%02d, ", address, length);
	LOG_BUFF(buff, length);
}

void MCU_Print_Receive_Data(uint32_t address, uint8_t *buff, uint8_t length)
{
	LOG_LEVEL("MCU_RECEIV addr:%08x length:%02d, ", address, length);
	LOG_BUFF(buff, length);
}

bool update_and_verify_dest_bank(uint32_t slot_addr)
{
	if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_LOADER)
		lt_mcu_program_buf.bank_slot = BANK_SLOT_A;
	else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_A)
		lt_mcu_program_buf.bank_slot = BANK_SLOT_B;
	else if (FLASH_BANK_CONFIG_MODE_SLOT == BANK_SLOT_B)
		lt_mcu_program_buf.bank_slot = BANK_SLOT_A;
	else lt_mcu_program_buf.bank_slot = BANK_SLOT_INVALID;
	
	switch(lt_mcu_program_buf.bank_slot)
	{
		case BANK_SLOT_A:
			if(slot_addr != app_meta_data.slot_a_addr) return false;
		break;
		case BANK_SLOT_B:
		  if(slot_addr != app_meta_data.slot_b_addr) return false;
		break;
    default:return false;		
	}
	return true;
}

// Function to handle sending update module requests based on the module type and parameters
bool update_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
	MY_ASSERT(buff);
	uint8_t buffer[8] = {0};
	// PRINT("update_module_send_handler  MOD %02x  CMD %02x PRARM %04x\n", module, cmd, param);
	if (MCU_TO_SOC_MOD_UPDATE == frame_type)
	{
		switch (param1)
		{
		case CMD_MODUPDATE_UPDATE_FW_STATE:
		{
			buffer[0] = lt_mcu_program_buf.state;
			buffer[1] = lt_mcu_program_buf.state; // ack success
			ptl_build_frame(MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_UPDATE_FW_STATE, buffer, 2, buff);
			return true;
		}
		case CMD_MODUPDATE_REQUEST_FW_DATA://request data from
		{
			// bootloader_iap_mode = 1;
			buffer[0] = lt_mcu_program_buf.state; // ack success
			buffer[1] = MK_MSB(lt_mcu_program_buf.f_count); // ack success
			buffer[2] = MK_LSB(lt_mcu_program_buf.f_count);
			ptl_build_frame(MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_REQUEST_FW_DATA, buffer, 3, buff);
			return true;
		}

		default:
			break;
		}
	}
	
	else if (SOC_TO_MCU_MOD_SYSTEM == frame_type)
	{
		switch (param1)
		{
			case CMD_MODUPDATE_ENTER_FW_UPDATE:
			{
				mcu_pragress_status.file_info = parse_firmware_file(update_path_filename);
				lt_mcu_program_buf.bank_address = mcu_pragress_status.file_info.reset_handler & 0xFFFF0000;
				lt_mcu_program_buf.total_length = mcu_pragress_status.file_info.file_size;
				if((mcu_pragress_status.file_info.file_type == FILE_TYPE_UNKNOWN || lt_mcu_program_buf.total_length == 0) &&  
				(lt_mcu_program_buf.bank_address != MAIN_APP_SLOT_A_START_ADDR || lt_mcu_program_buf.bank_address != MAIN_APP_SLOT_B_START_ADDR)		
				)
				{
					LOG_LEVEL("CMD_MODUPDATE_UPDATE_FW_STATE error file_type=%d total_length=%d bank_address=%08x\r\n",mcu_pragress_status.file_info.file_type,
					mcu_pragress_status.file_info.file_size,mcu_pragress_status.file_info.reset_handler);
					return false;
				}
				
				buffer[0] = (uint8_t)(lt_mcu_program_buf.bank_address & 0xFF);
				buffer[1] = (uint8_t)((lt_mcu_program_buf.bank_address >> 8) & 0xFF);
				buffer[2] = (uint8_t)((lt_mcu_program_buf.bank_address >> 16) & 0xFF);
				buffer[3] = (uint8_t)((lt_mcu_program_buf.bank_address >> 24) & 0xFF);

				buffer[4] = (uint8_t)(lt_mcu_program_buf.total_length & 0xFF);
				buffer[5] = (uint8_t)((lt_mcu_program_buf.total_length >> 8) & 0xFF);
				buffer[6] = (uint8_t)((lt_mcu_program_buf.total_length >> 16) & 0xFF);
				buffer[7] = (uint8_t)((lt_mcu_program_buf.total_length >> 24) & 0xFF);
				ptl_build_frame(SOC_TO_MCU_MOD_SYSTEM, CMD_MODUPDATE_ENTER_FW_UPDATE, buffer, 8, buff);
				return true;
			}
			default:
				break;
		}
	}
	return false;
}
// Function to handle receiving update module requests, process the payload, and send acknowledgment.
bool update_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
	MY_ASSERT(payload);
	MY_ASSERT(ackbuff);
	uint8_t buffer[20] = {0};
	if (SOC_TO_MCU_MOD_UPDATE == payload->frame_type)
	{

		switch (payload->frame_cmd)
		{
		case CMD_MODUPDATE_CHECK_FW_STATE:
		case CMD_MODUPDATE_UPDATE_FW_STATE: // second
		{
			LOG_LEVEL("CMD_MODUPDATE_UPDATE_FW_STATE \n");
			buffer[0] = lt_mcu_program_buf.state;
			buffer[1] = lt_mcu_program_buf.state;
			ptl_build_frame(MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_CHECK_FW_STATE, buffer, 2, ackbuff);
			return true;
		}
		case CMD_MODUPDATE_ENTER_FW_UPDATE: // first
		{
			//LOG_LEVEL("CMD_MODUPDATE_ENTER_FW_UPDATE \n");
			//uint32_t address = BYTES_TO_UINT32_LE(&payload->data[0]);
			lt_mcu_program_buf.bank_address = BYTES_TO_UINT32_LE(&payload->data[0]);
			lt_mcu_program_buf.total_length = BYTES_TO_UINT32_LE(&payload->data[4]);
			lt_mcu_program_buf.total_crc_32 = 0;
			lt_mcu_program_buf.error_count = 0;
			lt_mcu_program_buf.r_length = 0;
			lt_mcu_program_buf.f_count = 0;
			lt_mcu_program_buf.bank_slot = BANK_SLOT_INVALID;
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_IDLE;

			if(lt_mcu_program_buf.total_length == 0)
			{
				LOG_LEVEL("CMD_MODUPDATE_ENTER_FW_UPDATE faild update bank:%d address:%08x total_length=%d\r\n",lt_mcu_program_buf.bank_slot,lt_mcu_program_buf.bank_address,lt_mcu_program_buf.total_length);	
				return false;
			}
			if(update_and_verify_dest_bank(lt_mcu_program_buf.bank_address))
			{
				LOG_LEVEL("CMD_MODUPDATE_ENTER_FW_UPDATE update bank:%d address:%08x\r\n",lt_mcu_program_buf.bank_slot,lt_mcu_program_buf.bank_address);
				lt_mcu_program_buf.state = MCU_UPDATE_STATE_INIT;
			}
			else
			{
				lt_mcu_program_buf.state = MCU_UPDATE_STATE_IDLE;
				LOG_LEVEL("CMD_MODUPDATE_ENTER_FW_UPDATE failed update bank:%d address:%08x\r\n",lt_mcu_program_buf.bank_slot,lt_mcu_program_buf.bank_address);
			}
		
			return false;
		}
		case CMD_MODUPDATE_EXIT_FW_UPDATE:
		{		
			// tmp[0] = lt_mcu_program_buf.state;
			// tmp[1] = lt_mcu_program_buf.state;
			// ptl_build_frame(MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_EXIT_FW_UPDATE, tmp, 2, ackbuff);
			lt_mcu_program_buf.total_crc_32 = BYTES_TO_UINT32_LE(&payload->data[0]);
			uint32_t total_length = BYTES_TO_UINT32_LE(&payload->data[4]);
			
			if(total_length == lt_mcu_program_buf.total_length)	
				lt_mcu_program_buf.state = MCU_UPDATE_STATE_EXIT;
			else
				lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERROR;
			
			LOG_LEVEL("CMD_MODUPDATE_EXIT_FW_UPDATE total_crc_32=%08x total_length=%d\r\n",lt_mcu_program_buf.total_crc_32,total_length);
			return false;
		}
		case CMD_MODUPDATE_SEND_FW_DATA:
		{
			lt_mcu_program_buf.r_address = (payload->data[3] << 24) | (payload->data[2] << 16) | (payload->data[1] << 8) | payload->data[0];
			//MK_DWORD(MK_WORD(payload->data[0], payload->data[1]), MK_WORD(payload->data[2], payload->data[3]));
			lt_mcu_program_buf.r_length = payload->data_len-4;//payload->data[4];

			for (int i = 0; i < lt_mcu_program_buf.r_length; i++)
			{
				lt_mcu_program_buf.r_buff[i] = payload->data[4 + i];
			}

			//MCU_Print_Receive_Data(lt_mcu_program_buf.addr, lt_mcu_program_buf.buff, lt_mcu_program_buf.length);

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
	
	/*else if (MCU_TO_SOC_MOD_SYSTEM == payload->frame_type)
	{
		switch (payload->frame_cmd)
		{
			case CMD_MODUPDATE_SEND_FW_DATA:
			{
				
				buffer[0] = (uint8_t)(lt_mcu_program_buf.bank_address & 0xFF);
				buffer[1] = (uint8_t)((lt_mcu_program_buf.bank_address >> 8) & 0xFF);
				buffer[2] = (uint8_t)((lt_mcu_program_buf.bank_address >> 16) & 0xFF);
				buffer[3] = (uint8_t)((lt_mcu_program_buf.bank_address >> 24) & 0xFF);
				
				if(lt_mcu_program_buf.r_length + 4 <= sizeof(lt_mcu_program_buf.r_buff))
				{
				  for(uint16_t i = 0; i < lt_mcu_program_buf.r_length; i++)
					{
						 buffer[i+4] = lt_mcu_program_buf.r_buff[i];	
					}
			  }
				else
				{
					LOG_LEVEL("CMD_MODUPDATE_UPDATE_FW_STATE error lt_mcu_program_buf.r_length=%d\r\n",lt_mcu_program_buf.r_length);
				}
				ptl_build_frame(SOC_TO_MCU_MOD_SYSTEM, CMD_MODUPDATE_SEND_FW_DATA, buffer, 20, ackbuff);
				return true;
			}
			case CMD_MODUPDATE_EXIT_FW_UPDATE:
			{
				return false;	
			}
			default:
				break;
		}
	}*/
	return false;
}

// Function to process the MCU reboot state machine. It handles different stages of the reboot process.
static void mcu_update_state_proc(void)
{
	//static uint16_t writed_total_count;
	uint16_t erase_count = 0;
	uint16_t writed_count = 16;
	uint32_t crc_32;
	switch (lt_mcu_program_buf.state)
	{
	case MCU_UPDATE_STATE_IDLE:
		break;
	case MCU_UPDATE_STATE_CHECK:
		break;
	case MCU_UPDATE_STATE_INIT:
		LOG_LEVEL("MCU_UPDATE_STATE_INIT FLASH_BANK_CONFIG_MODE_SLOT:\r\n",FLASH_BANK_CONFIG_MODE_SLOT);
		if (lt_mcu_program_buf.bank_slot == BANK_SLOT_INVALID)
		{
			//lt_mcu_program_buf.bank_slot = BANK_SLOT_INVALID;
			LOG_LEVEL("MCU_UPDATE_STATE_INIT error! FLASH_BANK_CONFIG_MODE_SLOT:\r\n",FLASH_BANK_CONFIG_MODE_SLOT);
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_IDLE;
			break;
		}			
		
		lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERASE;
		lt_mcu_program_buf.f_count = 0;
		task_manager_stop_except(TASK_MODULE_UPDATE_MCU);
		task_manager_start_module(TASK_MODULE_PTL_1);
		task_manager_start_module(TASK_MODULE_IPC_SOCKET);
		
		break;
	case MCU_UPDATE_STATE_ERASE:
		LOG_LEVEL("MCU_UPDATE_STATE_ERASE...\n");
		if (lt_mcu_program_buf.bank_slot == BANK_SLOT_A)
		{
			app_meta_data.slot_a_crc = 0;
			app_meta_data.slot_a_size = 0;
			app_meta_data.slot_a_version = 0;
			app_meta_data.app_state_flags &= ~APP_FLAG_VALID_A;
		}
		else if (lt_mcu_program_buf.bank_slot == BANK_SLOT_B)
		{
			app_meta_data.slot_b_crc = 0;
			app_meta_data.slot_b_size = 0;
			app_meta_data.slot_b_version = 0;
			app_meta_data.app_state_flags &= ~APP_FLAG_VALID_B;			
		} 
		else
		{
			lt_mcu_program_buf.bank_slot = BANK_SLOT_INVALID;
			LOG_LEVEL("MCU_UPDATE_STATE_INIT error! bank_slot:\r\n",lt_mcu_program_buf.bank_slot);
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_IDLE;
			break;
		}		
    		
		erase_count = flash_erase_user_app_arear();
		LOG_LEVEL("MCU_UPDATE_STATE_ERASE... %d / %d\n",erase_count,MAIN_APP_BLOCK_COUNT);
		if (erase_count == MAIN_APP_BLOCK_COUNT)
		{
			flash_save_app_meter_infor();
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_RECEIVING;
			lt_mcu_program_buf.f_count = 0;//start receiving data
			send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_REQUEST_FW_DATA, lt_mcu_program_buf.state);
		}
		else
		{
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERROR;
		}  			
		break;		
	case MCU_UPDATE_STATE_RECEIVING:
   
		break;
	case MCU_UPDATE_STATE_UPDATING:
		// LOG_LEVEL("MCU_UPDATE_STATE_UPDATING \n");
		lt_mcu_program_buf.error_count = 0;
	
		RETRY_PROGRAM:
		writed_count = FlashWriteBuffTo(lt_mcu_program_buf.r_address, lt_mcu_program_buf.r_buff,lt_mcu_program_buf.r_length);
		MCU_Print_Program_Data(lt_mcu_program_buf.r_address, lt_mcu_program_buf.r_buff, lt_mcu_program_buf.r_length);
	
		if (writed_count == lt_mcu_program_buf.r_length)
		{
			lt_mcu_program_buf.f_count++;
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_RECEIVING;//continue receive data
			send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_UPDATE, CMD_MODUPDATE_REQUEST_FW_DATA, lt_mcu_program_buf.state);
			break;
		}

		lt_mcu_program_buf.error_count++;
		LOG_LEVEL("FlashWriteBuffTo failed  writed_count=%d\n",writed_count);
		if (lt_mcu_program_buf.error_count >= 3)
		{
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERROR;
			break;
		}

		goto RETRY_PROGRAM;

	case MCU_UPDATE_STATE_EXIT:
		LOG_LEVEL("MCU_UPDATE_STATE_EXIT \n");
	 
	  if (lt_mcu_program_buf.bank_slot == BANK_SLOT_A)
		{
			DISABLE_IRQ;
			crc_32 = flash_calculate_crc_32((uint8_t *)app_meta_data.slot_a_addr, lt_mcu_program_buf.total_length);
			ENABLE_IRQ;
		}
	  else if(lt_mcu_program_buf.bank_slot == BANK_SLOT_B)
		{
			DISABLE_IRQ;
			crc_32 = flash_calculate_crc_32((uint8_t *)app_meta_data.slot_b_addr, lt_mcu_program_buf.total_length);
			ENABLE_IRQ;
		}
	  else
		{
			LOG_LEVEL("lt_mcu_program_buf.bank_slot error! bank_slot=%d\n",lt_mcu_program_buf.bank_slot);
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERROR;	
     		 break;			
		}
	  
	  if(crc_32 == lt_mcu_program_buf.total_crc_32)
	  {
		  if (lt_mcu_program_buf.bank_slot == BANK_SLOT_A)
			{
				app_meta_data.slot_a_crc = crc_32; 	
				app_meta_data.app_state_flags |= APP_FLAG_VALID_A;	
				app_meta_data.slot_a_size = lt_mcu_program_buf.total_length;
				SET_FLAG(app_meta_data.app_state_flags,APP_FLAG_SLOT_A_UPGRADED);				
			}     	
			if (lt_mcu_program_buf.bank_slot == BANK_SLOT_B)
			{
				app_meta_data.slot_b_crc = crc_32;
				app_meta_data.app_state_flags |= APP_FLAG_VALID_B;
				app_meta_data.slot_b_size = lt_mcu_program_buf.total_length;
				SET_FLAG(app_meta_data.app_state_flags,APP_FLAG_SLOT_B_UPGRADED);
			}		
			lt_mcu_program_buf.state = MCU_UPDATE_STATE_COMPLETE;	
	  }
	  else	
	  {
			if (lt_mcu_program_buf.bank_slot == BANK_SLOT_A)
			{
				app_meta_data.slot_a_crc = crc_32; 	
				app_meta_data.app_state_flags &= ~APP_FLAG_VALID_A;		
				app_meta_data.slot_a_size = lt_mcu_program_buf.total_length;			
			} 
			
			if (lt_mcu_program_buf.bank_slot == BANK_SLOT_B)
			{
				app_meta_data.slot_b_crc = crc_32;
				app_meta_data.app_state_flags &= ~APP_FLAG_VALID_B;
				app_meta_data.slot_b_size = lt_mcu_program_buf.total_length;
			}				
		  lt_mcu_program_buf.state = MCU_UPDATE_STATE_ERROR;		
	  }
	  
		break;	
		
	case MCU_UPDATE_STATE_COMPLETE:
		
	 if (lt_mcu_program_buf.bank_slot == BANK_SLOT_A)
		{
			LOG_LEVEL("MCU_UPDATE_STATE_COMPLETE app_meta_data.slot_a_crc=%08x,received crc_32=%08x\n",app_meta_data.slot_a_crc,lt_mcu_program_buf.total_crc_32);
			//app_meta_data.slot_a_version = build_version_code();
			flash_save_app_meter_infor(); 
			JumpToApplication(app_meta_data.slot_a_addr);
		}
	 else if (lt_mcu_program_buf.bank_slot == BANK_SLOT_B)
		{
			LOG_LEVEL("MCU_UPDATE_STATE_COMPLETE app_meta_data.slot_b_crc=%08x,received crc_32=%08x\n",app_meta_data.slot_b_crc,lt_mcu_program_buf.total_crc_32);
			///app_meta_data.slot_b_version = build_version_code();
			flash_save_app_meter_infor(); 
			JumpToApplication(app_meta_data.slot_b_addr);
		}
		else
		{
			LOG_LEVEL("lt_mcu_program_buf.bank_slot error! bank_slot=%d\n",lt_mcu_program_buf.bank_slot);
		}
		lt_mcu_program_buf.state = MCU_UPDATE_STATE_IDLE;
		break;
	
	case MCU_UPDATE_STATE_ERROR:  
	  if (lt_mcu_program_buf.bank_slot == BANK_SLOT_A)
	    LOG_LEVEL("MCU_UPDATE_STATE_ERROR app_meta_data.slot_a_crc=%08x,received crc_32=%08x\n",app_meta_data.slot_a_crc,lt_mcu_program_buf.total_crc_32);
		
	  else if (lt_mcu_program_buf.bank_slot == BANK_SLOT_B)
		 LOG_LEVEL("MCU_UPDATE_STATE_ERROR app_meta_data.slot_b_crc=%08x,received crc_32=%08x\n",app_meta_data.slot_b_crc,lt_mcu_program_buf.total_crc_32);
		
	  else
		 LOG_LEVEL("lt_mcu_program_buf.bank_slot error! bank_slot=%d\n",lt_mcu_program_buf.bank_slot);
		
	  flash_save_app_meter_infor(); 
	  lt_mcu_program_buf.state = MCU_UPDATE_STATE_IDLE;
		break;
	}
	
}

#endif
