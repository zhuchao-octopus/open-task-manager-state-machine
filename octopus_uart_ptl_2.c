/*******************************************************************************
 * @file octopus_task_manager_ptl.c
 * @brief C file for managing Octopus protocol tasks.
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

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h"   // Include platform-specific header for hardware platform details
#include "octopus_uart_ptl_2.h" // Include UART protocol header
#include "octopus_uart_hal.h"   // Include UART hardware abstraction layer header

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */
// #define TEST_LOG_DEBUG_PTL_RX_FRAME // Enable debugging for receiving frames
// #define TEST_LOG_DEBUG_PTL_TX_FRAME // Enable debugging for transmitting frames

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * LOCAL FUNCTIONS DECLARATION
 */

// Declare function prototypes for various tasks and processing functions
#ifdef TASK_MANAGER_STATE_MACHINE_PTL2

static void ptl_2_proc_valid_frame(void); // Process the valid frame
static void ptl_2_rx_event_message_handler(void);

/*******************************************************************************
 * STATIC VARIABLES
 */
static uint32_t l_t_ptl_rx_main_timer;
// static uint32_t l_t_ptl_tx_main_timer;
// static uint32_t l_t_ptl_error_detect_timer;
// static bool lb_com_error = false;

static ptl_2_module_info_t ptl_2_module_info[PTL2_MODULE_MAX];
static uint8_t ptl_2_next_empty_module = 0;

/*******************************************************************************
 * GLOBAL FUNCTIONS IMPLEMENTATION
 */

void ptl_2_register_module(ptl_2_module_t module, ptl_2_module_receive_handler_t receive_handler)
{
    if (ptl_2_next_empty_module < PTL2_MODULE_MAX)
    {
        LOG_LEVEL("ptl_2_register_module %d\r\n", module);

        ptl_2_module_info[ptl_2_next_empty_module].module = module;
        ptl_2_module_info[ptl_2_next_empty_module].receive_handler = receive_handler;

        cFifo_Init(&ptl_2_module_info[ptl_2_next_empty_module].ptl_2_usart_rx_fifo,
                   ptl_2_module_info[ptl_2_next_empty_module].ptl_2_usart_rx_fifo_buff,
                   sizeof(ptl_2_module_info[ptl_2_next_empty_module].ptl_2_usart_rx_fifo_buff));

        ptl_2_next_empty_module++;
    }
}

ptl_2_module_info_t *ptl_2_get_module(ptl_2_module_t module)
{
    ptl_2_module_info_t *module_info = NULL;

    for (uint8_t i = 0; i < ptl_2_next_empty_module; i++)
    {
        if (ptl_2_module_info[i].module == module)
        {
            module_info = &ptl_2_module_info[i];
            break;
        }
    }

    return module_info;
}

void print_ptl2_registered_module(void)
{
    // module_info_t *module_info = NULL;
    for (uint8_t i = 0; i < ptl_2_next_empty_module; i++)
    {
        LOG_LEVEL("registered ptl_2_module_info[%d]=%02x \r\n", i, ptl_2_module_info[i].module);
    }
}

// Initialize UART communication for the task
void task_ptl_2_init_running(void)
{
    LOG_LEVEL("ptl2_init_running\r\n");
    OTMS(TASK_MODULE_PTL_2, OTMS_S_INVALID);
}

// Start the UART communication for the task
void task_ptl_2_start_running(void)
{
    LOG_LEVEL("ptl2_start_running\r\n");
    OTMS(TASK_MODULE_PTL_2, OTMS_S_ASSERT_RUN);
}

// Assert that UART communication is running
void task_ptl_2_assert_running(void)
{
    StartTickCounter(&l_t_ptl_rx_main_timer);
    // StartTickCounter(&l_t_ptl_tx_main_timer);
    // StartTickCounter(&l_t_ptl_error_detect_timer);
    OTMS(TASK_MODULE_PTL_2, OTMS_S_RUNNING);
}

// Main running function for UART communication
void task_ptl_2_running(void)
{
    // if(GetTickCounter(&l_t_ptl_rx_main_timer) < 10)
    //	return;
    // StartTickCounter(&l_t_ptl_rx_main_timer);
    ptl_2_rx_event_message_handler();
    ptl_2_proc_valid_frame();
}

// Post-running function for UART communication
void task_ptl_2_post_running(void)
{
    OTMS(TASK_MODULE_PTL_2, OTMS_S_ASSERT_RUN);
}

// Stop the UART communication task
void task_ptl_2_stop_running(void)
{
    LOG_LEVEL("_stop_running\r\n");
    OTMS(TASK_MODULE_PTL_2, OTMS_S_INVALID);
}

void ptl_2_receive_callback(ptl_2_module_t ptl_2_module, const uint8_t *buffer, uint16_t length)
{
#ifdef TEST_LOG_DEBUG_UART_RX_DATA
    LOG_BUFF_LEVEL(buffer, length);
#endif
#if 1
    ptl_2_module_info_t *ptl_2_module_infor = ptl_2_get_module(ptl_2_module);
    if (ptl_2_module_infor != NULL)
    {
        for (uint8_t j = 0; j < length; j++)
        {
            cFifo_Push(ptl_2_module_infor->ptl_2_usart_rx_fifo, buffer[j]);
        }
    }
#endif
}

uint8_t ptl_2_get_fifo_data(cFifo_t *a_ptFifo, uint8_t *buffer, uint16_t length)
{
    uint8_t data = 0;  // Variable to hold each byte read from the FIFO
    uint8_t index = 0; // Index to track how many bytes have been stored in the buffer

    // Get the current size of the FIFO (number of available bytes to read)
    // uint8_t datasize = cFifo_DataSize(ptl_1_usart_rx_fifo);
    // if(datasize <= 0) return index;
    // Loop to read data from FIFO until we either fill the buffer or run out of data
    while (1)
    {
        // If we haven't reached the desired length and there's still data in the FIFO
        if (index < length)
        {
            // Try to pop a byte from the FIFO
            if (true == cFifo_Pop(a_ptFifo, &data))
            {
                // Store the byte in the provided buffer
                buffer[index] = data;
                index++; // Increment the index to store the next byte
            }
            else
            {
                // If no more data is available in the FIFO, exit the loop
                break;
            }
        }
        else
        {
            // If we've already read the desired number of bytes, exit the loop
            break;
        }
    }

    // Return the number of bytes actually read from the FIFO
    return index;
}

/**
 * Handler for receiving UART data byte-by-byte.
 * Calls the UART reception handler to store the received data.
 */
void ptl_2_rx_event_message_handler(void)
{
    uint8_t count = 0;
    // while (1)
    for (uint8_t i = 0; i < ptl_2_next_empty_module; i++) // handle all modules
    {
#if 1
        ptl_2_module_info_t *module_info = &ptl_2_module_info[i];
        if (module_info->ptl_2_proc_buff.size >= PTL2_FRAME_MAX_SIZE)
        { // ptl_2_proc_buff is full must to be processed first before get from fifo
            continue;
        }
#endif
#ifdef TASK_MANAGER_STATE_MACHINE_BT_MUSIC
        if (module_info->module == PTL2_MODULE_BT)
        {
            // For AT command module: Read until we get a '\n' or fill the buffer
            while (cFifo_HasLine(module_info->ptl_2_usart_rx_fifo))
            {
                if (module_info->ptl_2_proc_buff.size >= PTL2_FRAME_MAX_SIZE / 2)
                    break; // wait for processing
                uint8_t byte = 0;
                count = ptl_2_get_fifo_data(module_info->ptl_2_usart_rx_fifo, &byte, 1);
                if (count > 0)
                {
                    module_info->ptl_2_proc_buff.buffer[module_info->ptl_2_proc_buff.size++] = byte;
                    // Stop when reaching end of an AT command line
                    if (byte == '\n')
                    {
                        module_info->ptl_2_proc_buff.buffer[module_info->ptl_2_proc_buff.size++] = '\0';
                        // LOG_LEVEL("%s",proc_buffer->buffer);
                    }
                }
            }
        }
        else
#endif
        {
            count = ptl_2_get_fifo_data(
                module_info->ptl_2_usart_rx_fifo,
                &module_info->ptl_2_proc_buff.buffer[module_info->ptl_2_proc_buff.size],
                PTL2_FRAME_MAX_SIZE - module_info->ptl_2_proc_buff.size);

            if (count > 0)
                module_info->ptl_2_proc_buff.size = module_info->ptl_2_proc_buff.size + count;
        }

    } // for
}

/**
 * Processes the valid frame after it has been extracted from the buffer.
 * The payload is passed to the appropriate module handler.
 */
void ptl_2_proc_valid_frame(void)
{
    for (uint8_t i = 0; i < ptl_2_next_empty_module; i++) // handle all modules
    {
        ptl_2_module_info_t *module_info = &ptl_2_module_info[i];

        if (module_info->receive_handler == NULL)
        {
            LOG_LEVEL("l_t_module_info[i].receive_handler is null.");
            continue;
        }
        if (module_info->ptl_2_proc_buff.size == 0)
            continue;

        bool res = module_info->receive_handler(&(module_info->ptl_2_proc_buff));
        if (res)
        {
#ifdef TEST_LOG_DEBUG_PTL_RX_FRAME
            if (module_info->module == PTL2_MODULE_BAFANG)
            {
                LOG_LEVEL("ptl_2_proc_valid_frame data[]=");
                LOG_BUFF(module_info->ptl_2_proc_buff.buffer, module_info->ptl_2_proc_buff.size);
            }
#endif
            module_info->ptl_2_proc_buff.size = 0;
        }
        else
        {
#if defined(TASK_MANAGER_STATE_MACHINE_LING_HUI_LIION2)
            if (module_info->module == PTL2_MODULE_LING_HUI_LIION2)
            {
                if (module_info->ptl_2_proc_buff.size > 25)
                    module_info->ptl_2_proc_buff.size = 0;
            }
#elif defined(TASK_MANAGER_STATE_MACHINE_BAFANG)
            if (module_info->module == PTL2_MODULE_BAFANG)
            {
                if (module_info->ptl_2_proc_buff.size > 10)
                    module_info->ptl_2_proc_buff.size = 0;
            }
#endif
        }
    }
}

void ptl_2_send_buffer(ptl_2_module_t ptl_2_module, const uint8_t *buffer, size_t size)
{
    if (buffer == NULL || size == 0)
    {
        return;
    }

    switch (ptl_2_module)
    {

#ifdef TASK_MANAGER_STATE_MACHINE_BAFANG
    case PTL2_MODULE_BAFANG:
        LPUART_Send_Buffer(buffer, size);
        break;
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_LING_HUI_LIION2
    case PTL2_MODULE_LING_HUI_LIION2:
        LPUART_Send_Buffer(buffer, size);
        break;
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_4G
    case PTL2_MODULE_LOT4G:
        UART4_Send_Buffer(buffer, size);
        break;
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_BT_MUSIC
    case PTL2_MODULE_BT:
        UART1_Send_Buffer(buffer, size);
        break;
#endif
    default:
        break;
    }
}

#endif
