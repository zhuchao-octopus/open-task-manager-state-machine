/**
 * @file octopus_task_manager_sif.h
 * @brief Header file for managing the SIF (Serial Interface) communication within the Octopus Task Manager.
 *
 * This file provides function declarations and macros for handling the SIF communication,
 * including initializing, deinitializing, sending, and receiving data. The macros control
 * the GPIO pins used for sending and receiving data. It also includes external function declarations
 * for the SIF interrupt handler and data read operations.
 *
 * The SIF communication is based on GPIO pins configured to send and receive data bits,
 * with support for custom initialization, data handling, and state checking.
 *
 * @note The macros for GPIO pin assignments (e.g., `SIF_REV_DATA_PIN`) should be adjusted based
 * on the actual project configuration.
 *
 * @ingroup APP:SUB_TYPE
 * @version  1.0.0
 * @date     2024-12-11
 * @author   Octopus Team
 */

#ifndef __OCTOPUS_TASK_MANAGER_SIF_H__
#define __OCTOPUS_TASK_MANAGER_SIF_H__

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_base.h" //  Base include file for the Octopus project.

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */
#ifdef TASK_MANAGER_STATE_MACHINE_SIF

/*********************************************************************
 * CONSTANTS
 * Constants used for the SIF communication.
 */

/********************************************************************
 * MACROS
 * The following macros control the GPIO pins for sending and receiving data.
 * These should be adjusted according to the specific hardware configuration.
 */

// Data receive GPIO pin, defined to read data from the SIF communication interface
#define SIF_RECEIVE_DATA_BIT() hal_gpio_read(GPIO_SIF_GROUP, GPIO_SIF_PIN)

// Set the send data GPIO pin to low for transmission
#define SIF_SEND_DATA_BIT_LOW() hal_gpio_write(GPIO_SIF_GROUP, GPIO_SIF_PIN, BIT_RESET)

// Set the send data GPIO pin to high for transmission
#define SIF_SEND_DATA_BIT_HIGH() hal_gpio_write(GPIO_SIF_GROUP, GPIO_SIF_PIN, BIT_SET)

#ifdef __cplusplus
extern "C"
{
#endif
    /*******************************************************************************
     * TYPEDEFS
     * Place for defining types related to SIF communication, if needed.
     */

    /*******************************************************************************
     * GLOBAL VARIABLES
     * Declaration for global variables related to SIF communication, if necessary.
     */

    /*******************************************************************************
     * EXTERNAL FUNCTION DECLARATIONS
     * The following functions are externally declared for managing the SIF interface.
     */

    /**
     * @brief Interrupt handler for SIF IO events.
     *
     * This function handles interrupts generated during SIF communication, allowing the system
     * to respond to data reception or transmission events.
     */
    void SIF_IO_IRQHandler(void);

    /**
     * @brief Checks if the SIF interface is idle.
     *
     * This function checks if the SIF interface is currently in an idle state, indicating that
     * no data transmission or reception is ongoing.
     *
     * @return true if the SIF interface is idle, false otherwise.
     */
    bool sif_is_idle(void);

    /*******************************************************************************
     * LOCAL FUNCTION DECLARATIONS
     * The following functions are local to the SIF module and handle its initialization,
     * deinitialization, and data operations.
     */

    /**
     * @brief Initializes the SIF interface.
     *
     * This function sets up the necessary GPIO pins and configurations to initialize the SIF
     * communication interface.
     */
    void sif_init(void);

    /**
     * @brief Checks if the SIF interface is initialized.
     *
     * This function returns the status of the SIF interface initialization.
     *
     * @return true if the SIF interface is initialized, false otherwise.
     */
    bool sif_is_init(void);

    /**
     * @brief Reads data from the SIF interface.
     *
     * This function reads a specified number of bytes of data from the SIF interface into a buffer.
     *
     * @param data A pointer to the buffer where the received data will be stored.
     * @param maxlen The maximum number of bytes to read.
     * @return The number of bytes successfully read from the interface.
     */
    uint8_t sif_read_data(uint8_t *data, uint8_t maxlen);
    void sif_delay_50_us(uint32_t delay_us);
    /*******************************************************************************
    *******************************************************************************/
    void otsm_sif_init(void);

    extern volatile uint32_t system_timer_tick_50us;
#ifdef __cplusplus
}
#endif

#endif

#endif /* __OCTOPUS_TASK_MANAGER_SIF_H__ */
