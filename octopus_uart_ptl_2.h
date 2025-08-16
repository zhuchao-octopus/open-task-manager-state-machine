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

#ifndef __OCTOPUS_TASK_MANAGER_PTL_2_H__
#define __OCTOPUS_TASK_MANAGER_PTL_2_H__
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
#define PTL2_FIFO_MAX_SIZE 255
#define PTL2_FRAME_MAX_SIZE 255 ///< Maximum frame size

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
		PTL2_MODULE_BAFANG, ///< Protocol for Bafang
// #endif
#ifdef TASK_MANAGER_STATE_MACHINE_4G
		PTL2_MODULE_LOT4G,
#endif
#ifdef TASK_MANAGER_STATE_MACHINE_BT_MUSIC
		PTL2_MODULE_BT,
#endif
#ifdef TASK_MANAGER_STATE_MACHINE_LING_HUI_LIION2
		PTL2_MODULE_LING_HUI_LIION2,
#endif

		PTL2_MODULE_MAX,
	} PTL2_MODEL;

	// #define  SETTING_PTL_BEGIN SETTING_PTL_BAFANG
	// #define  SETTING_PTL_END   (SETTING_PTL_MAX-1)

/* ============================== UART PTL ============================== */
#define PTL_FRAME_MAX_SIZE 255

	typedef PTL2_MODEL ptl_2_module_t;

	typedef struct
	{
		uint16_t size;
		uint8_t buffer[PTL_FRAME_MAX_SIZE];
	} ptl_2_proc_buff_t;

	typedef bool (*ptl_2_module_receive_handler_t)(ptl_2_proc_buff_t *buffer);

	typedef struct
	{
		ptl_2_module_t module;
		ptl_2_proc_buff_t ptl_2_proc_buff;
		ptl_2_module_receive_handler_t receive_handler;
		cFifo_t *ptl_2_usart_rx_fifo;
		uint8_t ptl_2_usart_rx_fifo_buff[cFifo_ObjSize(PTL2_FIFO_MAX_SIZE)];
	} ptl_2_module_info_t;

	void task_ptl_2_init_running(void);
	void task_ptl_2_start_running(void);
	void task_ptl_2_assert_running(void);
	void task_ptl_2_running(void);
	void task_ptl_2_post_running(void);
	void task_ptl_2_stop_running(void);

	bool ptl_2_is_com_error(void);

	uint8_t ptl_2_get_fifo_data(cFifo_t *a_ptFifo, uint8_t *buffer, uint16_t length);

	void ptl_2_register_module(ptl_2_module_t module, ptl_2_module_receive_handler_t receive_handler);
	void ptl_2_receive_callback(ptl_2_module_t ptl_2_module, const uint8_t *buffer, uint16_t length);
	void ptl_2_send_buffer(ptl_2_module_t ptl_2_module, const uint8_t *buffer, size_t size);
	void print_ptl2_registered_module(void);

#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_TASK_MANAGER_PTL_H__ */
