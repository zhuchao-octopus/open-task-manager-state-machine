/*
 * octopus_i2c.h
 *
 * @version 1.0
 * @date    2025-09-18
 * @author  Octopus Team
 *
 * @brief
 * This header file declares the public interfaces for the I2C task module.
 * It provides lifecycle functions for managing I2C communication in an
 * embedded system environment.
 *
 * Typical usage:
 * - Call `task_i2c_init_running()` once during system initialization.
 * - Call `task_i2c_start_running()` when the I2C service should begin.
 * - Periodically call `task_i2c_running()` from the main loop or RTOS task.
 * - Optionally call `task_i2c_post_running()` for deferred actions.
 * - Call `task_i2c_stop_running()` to gracefully stop I2C operations.
 *
 * This module is designed to integrate with a task state machine framework.
 */

#ifndef __OCTOPUS_I2C_H_
#define __OCTOPUS_I2C_H_

#include "octopus_base.h" //  Base include file for the Octopus project.

/**
 * @brief I2C communication error codes
 */
typedef enum
{
	I2C_ERROR_NONE = 0,			///< No error, last communication successful
	I2C_ERROR_NO_ACK = 1,		///< No ACK received after sending a byte
	I2C_ERROR_READ_TIMEOUT = 2, ///< Timeout during reading SDA
	I2C_ERROR_BUS_LOCKED = 3,	///< Bus stuck in busy state (not yet implemented)
} i2c_error_t;

#ifdef __cplusplus
extern "C"
{
#endif

	/*******************************************************************************
	 * GLOBAL FUNCTIONS DECLARATION
	 *******************************************************************************/

	/**
	 * @brief Initialize I2C functionality.
	 *
	 * This function sets up internal data structures, initializes the I2C hardware,
	 * and prepares the message queue for operation.
	 */
	void task_i2c_init_running(void);

	/**
	 * @brief Start I2C operations.
	 *
	 * Enables interrupts, DMA, or background tasks associated with I2C communication.
	 * Should be called after initialization and when I2C needs to become active.
	 */
	void task_i2c_start_running(void);

	/**
	 * @brief Assert and verify the I2C module is running correctly.
	 *
	 * This function can be used as a sanity check to ensure the I2C module is still
	 * operating as expected (e.g., queue not stuck, bus not hung).
	 */
	void task_i2c_assert_running(void);

	/**
	 * @brief Handle the main logic for I2C operations.
	 *
	 * This function is expected to be called periodically in the main loop or a
	 * dedicated RTOS task. It processes messages in the I2C TX/RX queue and
	 * performs data transfers.
	 */
	void task_i2c_running(void);

	/**
	 * @brief Perform post-processing for I2C operations.
	 *
	 * This function can handle cleanup or deferred actions after I2C transfers,
	 * such as callback execution or logging.
	 */
	void task_i2c_post_running(void);

	/**
	 * @brief Stop I2C operations.
	 *
	 * Gracefully shuts down I2C communication, disables interrupts, and clears
	 * internal states. Should be called before system shutdown or reset.
	 */
	void task_i2c_stop_running(void);

	void i2c_send_message(uint8_t channel, uint8_t dev_address, uint8_t reg_address, const uint8_t *data, uint8_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* __OCTOPUS_I2C_H_ */
