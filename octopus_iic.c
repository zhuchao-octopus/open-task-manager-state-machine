/**
 * @file    octopus_iic.c
 * @brief   Software I2C (bit-banging) driver implementation for GPIO-based I2C communication.
 * @author  Macky
 * @version 1.0
 * @date    2025-07-07
 *
 * This file provides basic software I2C functions using GPIO for platforms
 * without hardware I2C support or for communicating with simple I2C peripherals.
 *
 * Supports:
 * - I2C start/stop condition
 * - Send/receive one byte
 * - ACK/NAK handling
 * - Configurable via HAL GPIO wrapper
 */

#include "octopus_iic.h"

#ifdef TASK_MANAGER_STATE_MACHINE_I2C
// Static variable to hold the last error encountered
static volatile i2c_error_t i2c_last_error = I2C_ERROR_NONE;

// Function to get the last I2C error
i2c_error_t i2c_get_last_error(void)
{
    return i2c_last_error;
}

// Function to clear the last I2C error
void i2c_clear_error(void)
{
    i2c_last_error = I2C_ERROR_NONE;
}

// --- I2C Functions ---
// Initialize the I2C GPIO pins (SDA and SCL)
void i2c_init(void)
{
    // Initialize the GPIO pins for I2C communication
    iic_scl_out();
    iic_sda_out();

    iic_scl_high();         // Set SCL to high initially
    iic_sda_high();         // Set SDA to high initially
    platform_delay_us__(5); // Small delay to ensure lines are stable

    i2c_clear_error(); // Clear any previous errors
}

// Start I2C communication (SDA goes low while SCL is high)
void i2c_start(void)
{
    iic_sda_out();
    iic_sda_high(); // Make sure SDA is high
    iic_scl_high(); // Ensure SCL is high
    platform_delay_us__(5);

    iic_sda_low(); // Pull SDA low
    platform_delay_us__(5);

    iic_scl_low(); // Pull SCL low to start communication
    platform_delay_us__(5);
}

// Stop I2C communication (SDA goes high while SCL is high)
void i2c_stop(void)
{
    iic_sda_out();
    iic_scl_low(); // Pull SCL low first
    iic_sda_low(); // Then pull SDA low
    platform_delay_us__(5);

    iic_scl_high(); // Pull SCL high to end communication
    platform_delay_us__(5);

    iic_sda_high(); // Pull SDA high to signal stop
    platform_delay_us__(5);
}

// Wait for ACK after sending a byte
uint8_t i2c_wait_ack(void)
{
    uint8_t retry = 0;

    iic_sda_in();   // Set SDA to input to read ACK
    iic_sda_high(); // Release SDA line
    platform_delay_us__(1);

    iic_scl_high(); // Clock the ACK
    platform_delay_us__(5);

    while (read_sda_())
    { // Wait for SDA to go low (ACK)
        retry++;
        if (retry > 200)
        { // Timeout after 200 retries (indicating no ACK)
            i2c_last_error = I2C_ERROR_NO_ACK;
            i2c_stop(); // Stop communication in case of no ACK
            return 1;   // No ACK received (NACK)
        }
    }

    iic_scl_low();                   // Pull SCL low after receiving ACK
    i2c_last_error = I2C_ERROR_NONE; // ACK received, clear error
    return 0;                        // ACK received successfully
}

// Send an ACK bit to continue communication
void i2c_send_ack(void)
{
    iic_sda_out(); // Set SDA to output
    iic_sda_low(); // Pull SDA low for ACK
    platform_delay_us__(2);

    iic_scl_high(); // Clock the ACK bit
    platform_delay_us__(5);
    iic_scl_low(); // Pull SCL low after ACK
}

// Send a NACK bit to stop communication
void i2c_send_nack(void)
{
    iic_sda_out();  // Set SDA to output
    iic_sda_high(); // Pull SDA high for NACK
    platform_delay_us__(2);

    iic_scl_high(); // Clock the NACK bit
    platform_delay_us__(5);
    iic_scl_low(); // Pull SCL low after NACK
}

// Send a byte over I2C
void i2c_send_byte(uint8_t data)
{
    uint8_t i;

    iic_sda_out(); // Set SDA to output
    for (i = 0; i < 8; i++)
    {
        iic_scl_low(); // Pull SCL low to start bit transfer
        platform_delay_us__(2);

        if (data & 0x80)
        { // Send the most significant bit first
            iic_sda_high();
        }
        else
        {
            iic_sda_low();
        }
        data <<= 1; // Shift data to send the next bit

        platform_delay_us__(2);
        iic_scl_high(); // Clock the bit
        platform_delay_us__(5);
    }

    iic_scl_low(); // Pull SCL low after byte is sent
}

// Read a byte from I2C
uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i, data = 0;
    uint16_t timeout;

    iic_sda_in(); // Set SDA to input to receive data
    for (i = 0; i < 8; i++)
    {
        iic_scl_low(); // Pull SCL low to start bit reception
        platform_delay_us__(2);

        iic_scl_high(); // Clock the bit into SDA
        data <<= 1;
        if (read_sda_())
        { // Read the bit from SDA
            data |= 0x01;
        }

        platform_delay_us__(3);
    }

    iic_scl_low(); // Pull SCL low after byte is read

    // Send ACK or NACK depending on input argument
    if (ack)
    {
        i2c_send_ack();
    }
    else
    {
        i2c_send_nack();
    }

    if (timeout >= 1000)
    {
        i2c_last_error = I2C_ERROR_READ_TIMEOUT;
    }
    else
    {
        i2c_last_error = I2C_ERROR_NONE;
    }

    return data; // Return the received byte
}

#endif
