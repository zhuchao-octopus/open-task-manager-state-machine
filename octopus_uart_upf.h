/*******************************************************************************
 * @file octopus_task_manager_ptl.h
 * @brief Header file for managing Octopus protocol tasks.
 *
 * This file defines the structure, constants, and function prototypes for handling
 * communication tasks between the MCU (Microcontroller Unit) and the APP (Application)
 * within the Octopus platform. It provides the necessary protocol definitions to
 * ensure smooth task management and communication across the platform.
 *
 * The Octopus protocol facilitates seamless interaction between hardware and software,
 * enabling efficient data exchange and task synchronization. This header file serves
 * as the interface for the task manager, providing the necessary tools to integrate
 * protocol handling into the Octopus platform.
 *
 * @version 1.0.0
 * @author   Octopus Team
 * @date     2024-12-12
 *******************************************************************************/

#ifndef __OCTOPUS_TASK_MANAGER_upf_H__
#define __OCTOPUS_TASK_MANAGER_upf_H__
/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"
#include "octopus_cfifo.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * MACROS
 *******************************************************************************/
#define UPF_FIFO_MAX_SIZE 255
#define UPF_FRAME_MAX_SIZE 255 ///< Maximum frame size

	typedef enum
	{
		SETTING_WHEEL_16_Inch,
		SETTING_WHEEL_18_Inch,
		SETTING_WHEEL_20_Inch,
		SETTING_WHEEL_22_Inch,
		SETTING_WHEEL_24_Inch,
		SETTING_WHEEL_26_Inch,
		SETTING_WHEEL_27_Inch,
		SETTING_WHEEL_27_5_Inch,
		SETTING_WHEEL_28_Inch,
		SETTING_WHEEL_29_Inch,
	} SETTING_WHEEL;

	typedef enum
	{
		SETTING_MAX_PAS_3_LEVEL = 3,
		SETTING_MAX_PAS_5_LEVEL = 5,
		SETTING_MAX_PAS_9_LEVEL = 9,
	} SETTING_MAX_PAS;

	/*******************************************************************************
	 * ENUMERATIONS
	 *******************************************************************************/
	typedef enum
	{
		// #ifdef TASK_MANAGER_STATE_MACHINE_BAFANG
		UPF_MODULE_BAFANG, ///< Protocol for Bafang
// #endif
#ifdef TASK_MANAGER_STATE_MACHINE_4G
		UPF_MODULE_LOT4G,
#endif
#ifdef TASK_MANAGER_STATE_MACHINE_BT_MUSIC
		UPF_MODULE_BT,
#endif
#ifdef TASK_MANAGER_STATE_MACHINE_LING_HUI_LIION2
		UPF_MODULE_LING_HUI_LIION2,
#endif

		UPF_MODULE_MAX,
	} UPF_MODEL;

	typedef UPF_MODEL upf_module_t;

	typedef struct
	{
		uint16_t size;
		uint8_t buffer[PTL_FRAME_MAX_SIZE];
	} upf_proc_buff_t;

	typedef bool (*upf_module_receive_handler_t)(upf_proc_buff_t *buffer);

	typedef struct
	{
		upf_module_t module;
		upf_proc_buff_t upf_proc_buff;
		upf_module_receive_handler_t receive_handler;
		cFifo_t *upf_usart_rx_fifo;
		uint8_t upf_usart_rx_fifo_buff[cFifo_ObjSize(UPF_FIFO_MAX_SIZE)];
	} upf_module_info_t;

	void task_upf_init_running(void);
	void task_upf_start_running(void);
	void task_upf_assert_running(void);
	void task_upf_running(void);
	void task_upf_post_running(void);
	void task_upf_stop_running(void);

	bool upf_is_com_error(void);

	uint8_t upf_get_fifo_data(cFifo_t *a_ptFifo, uint8_t *buffer, uint16_t length);

	void upf_register_module(upf_module_t module, upf_module_receive_handler_t receive_handler);
	void upf_receive_callback(upf_module_t upf_module, const uint8_t *buffer, uint16_t length);
	void upf_send_buffer(upf_module_t upf_module, const uint8_t *buffer, size_t size);
	void upf_print_registered_module(void);

#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_TASK_MANAGER_PTL_H__ */
