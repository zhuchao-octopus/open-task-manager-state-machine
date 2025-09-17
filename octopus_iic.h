/**
 * @file    octopus_iic.h
 * @brief   Software I2C driver header for GPIO-based I2C bit-banging implementation.
 *          This header provides the interface and macros used to implement
 *          a lightweight I2C protocol on platforms lacking hardware I2C controllers.
 *
 * @author  Macky
 * @version 1.1
 * @date    2025-07-07
 *
 * @details
 * This file defines GPIO macros for SDA/SCL control, the software-based I2C API,
 * and error tracking mechanisms. It is designed to work with the GPIO HAL wrapper layer
 * provided in `octopus_gpio.h`.
 *
 * Features:
 * - Software-based I2C start/stop conditions
 * - Byte send/receive operations
 * - ACK/NACK handling
 * - Error detection (no ACK, timeout, bus lock)
 * - Optional logging support (in .c file)
 */

#ifndef __OCTOPUS_IIC_H__
#define __OCTOPUS_IIC_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "octopus_platform.h" ///< Platform-specific settings (e.g. clock, GPIO defs)
#include "octopus_gpio.h"     ///< HAL GPIO API used for I2C bit operations

// -----------------------------------------------------------------------------
// GPIO Macros for Software I2C Bit Manipulation
// -----------------------------------------------------------------------------
#define IIC_GPIO_GROUP 0
/**
 * @brief Set SCL line to high (logic 1)
 */
#define iic_scl_high() hal_gpio_write(IIC_GPIO_GROUP, GPIO_I2C_SCL_PIN, BIT_SET)

/**
 * @brief Set SCL line to low (logic 0)
 */
#define iic_scl_low() hal_gpio_write(IIC_GPIO_GROUP, GPIO_I2C_SCL_PIN, BIT_RESET)

/**
 * @brief Set SDA line to high (logic 1)
 */
#define iic_sda_high() hal_gpio_write(IIC_GPIO_GROUP, GPIO_I2C_SDA_PIN, BIT_SET)

/**
 * @brief Set SDA line to low (logic 0)
 */
#define iic_sda_low() hal_gpio_write(IIC_GPIO_GROUP, GPIO_I2C_SDA_PIN, BIT_RESET)

/**
 * @brief Read current level of SDA line
 * @return uint8_t Logic level (0 or 1)
 */
#define read_sda_() hal_gpio_read(IIC_GPIO_GROUP, GPIO_I2C_SDA_PIN)

/**
 * @brief Configure SDA as output (push-pull)
 */
#define iic_sda_out() hal_gpio_set_mode(IIC_GPIO_GROUP, GPIO_I2C_SDA_PIN, GPIO_MODE_OUTPUT)

/**
 * @brief Configure SDA as input (floating or pull-up)
 */
#define iic_sda_in() hal_gpio_set_mode(IIC_GPIO_GROUP, GPIO_I2C_SDA_PIN, GPIO_MODE_INPUT)

/**
 * @brief Configure SDA as output (push-pull)
 */
#define iic_scl_out() hal_gpio_set_mode(IIC_GPIO_GROUP, GPIO_I2C_SCL_PIN, GPIO_MODE_OUTPUT)

/**
 * @brief Configure SDA as input (floating or pull-up)
 */
#define iic_scl_in() hal_gpio_set_mode(IIC_GPIO_GROUP, GPIO_I2C_SCL_PIN, GPIO_MODE_INPUT)

    // -----------------------------------------------------------------------------
    // Error Type Definitions for I2C Communication
    // -----------------------------------------------------------------------------

    /**
     * @brief I2C communication error codes
     */
    typedef enum
    {
        I2C_ERROR_NONE = 0,         ///< No error, last communication successful
        I2C_ERROR_NO_ACK = 1,       ///< No ACK received after sending a byte
        I2C_ERROR_READ_TIMEOUT = 2, ///< Timeout during reading SDA
        I2C_ERROR_BUS_LOCKED = 3,   ///< Bus stuck in busy state (not yet implemented)
    } i2c_error_t;

    // -----------------------------------------------------------------------------
    // Software I2C API Declarations
    // -----------------------------------------------------------------------------

    /**
     * @brief Initialize I2C GPIO lines and optional DWT delay unit
     */
    void i2c_init(void);

    /**
     * @brief Generate I2C start condition (SDA goes low while SCL is high)
     */
    void i2c_start(void);

    /**
     * @brief Generate I2C stop condition (SDA goes high while SCL is high)
     */
    void i2c_stop(void);

    /**
     * @brief Wait for ACK after sending a byte
     * @retval 0 if ACK received
     * @retval 1 if no ACK (NACK)
     */
    uint8_t i2c_wait_ack(void);

    /**
     * @brief Send ACK bit (used when master wants to continue reading)
     */
    void i2c_send_ack(void);

    /**
     * @brief Send NACK bit (used when master wants to stop reading)
     */
    void i2c_send_nack(void);

    /**
     * @brief Send a byte over I2C bus (MSB first)
     * @param data Byte to send
     */
    void i2c_send_byte(uint8_t data);

    /**
     * @brief Read a byte from I2C bus
     * @param ack If 1, send ACK after receiving; if 0, send NACK
     * @return Received byte
     */
    uint8_t i2c_read_byte(uint8_t ack);

    /**
     * @brief Retrieve last I2C error status
     * @return i2c_error_t Last recorded error
     */
    i2c_error_t i2c_get_last_error(void);

    /**
     * @brief Clear I2C error state (usually after a successful transaction)
     */
    void i2c_clear_error(void);

#ifdef __cplusplus
}
#endif

#endif // __OCTOPUS_IIC_H__
