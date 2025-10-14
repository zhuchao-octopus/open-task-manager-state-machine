/*
 * octopus_i2c.c
 *
 * @version 1.0
 * @date    2025-09-18
 * @author  Octopus Team
 *
 * @brief
 * This source file implements the I2C task module. It integrates with the
 * system task framework and provides lifecycle management functions for
 * I2C communication.
 */
/*******************************************************************************
 * INCLUDES
 */
#include "octopus_iic.h"
#include "octopus_task_manager.h" // Task Manager: handles scheduling and execution of system tasks
#include "octopus_tickcounter.h"  // Tick Counter: provides timing and delay utilities
#include "octopus_message.h"      // Message IDs: defines identifiers for inter-task communication
#include "octopus_msgqueue.h"     // Message Queue: API for sending/receiving messages between tasks
#include "octopus_iic_queue.h"    // For I2C message queue
#include "octopus_bsp_hk32l08x.h"
#include "octopus_gpio.h"

#ifdef TASK_MANAGER_STATE_MACHINE_I2C
/* =======================================================================
 * MODULE INTERNAL STATE
 * ======================================================================= */

/** Global I2C TX message queue */
extern I2cQueue_t i2c_tx_msg_queue;

/** Flag indicating whether I2C task is active */
// static volatile uint8_t i2c_task_running = 0;

/* =======================================================================
 * PUBLIC FUNCTION IMPLEMENTATIONS
 * ======================================================================= */

void task_i2c_init_running(void)
{
    I2c_Queue_Init(&i2c_tx_msg_queue);
    LOG_LEVEL("task_i2c_init_running\r\n");
    OTMS(TASK_MODULE_IIC, OTMS_S_INVALID);
}

void task_i2c_start_running(void)
{
    LOG_LEVEL("task_i2c_start_running\r\n");
    OTMS(TASK_MODULE_IIC, OTMS_S_ASSERT_RUN);
}

void task_i2c_assert_running(void)
{
    OTMS(TASK_MODULE_IIC, OTMS_S_RUNNING);
}

void task_i2c_running(void)
{
    I2c_QueueMsg_t *msg = I2c_GetMsg();
    if (msg != NULL)
    {
        // TODO: Perform actual I2C transfer using msg->dev_address,
        // msg->reg_address, msg->data, msg->data_len
        // I2C_Master_Transmit(msg->dev_address, msg->reg_address, msg->data, msg->data_len);
        if (msg->oparation == 0)
        {
            switch (msg->channel)
            {
            case 1:
                hal_iic1_write(msg->dev_address, msg->reg_address, msg->data, msg->data_len);
                break;
            case 2:
                hal_iic2_write(msg->dev_address, msg->reg_address, msg->data, msg->data_len);
                break;
            case 3:
                hal_iic3_write(msg->dev_address, msg->reg_address, msg->data, msg->data_len);
                break;
            case 4:
                hal_iic4_write(msg->dev_address, msg->reg_address, msg->data, msg->data_len);
                break;
            case 5:
                hal_iic5_write(msg->dev_address, msg->reg_address, msg->data, msg->data_len);
                break;
            case 6:
                hal_iic6_write(msg->dev_address, msg->reg_address, msg->data, msg->data_len);
                break;
            }
        }
    }
}

void task_i2c_post_running(void)
{
    OTMS(TASK_MODULE_IIC, OTMS_S_ASSERT_RUN);
}

void task_i2c_stop_running(void)
{
    LOG_LEVEL("_stop_running\r\n");
    OTMS(TASK_MODULE_IIC, OTMS_S_INVALID);
}

void i2c_send_message(uint8_t channel, uint8_t dev_address, uint8_t reg_address, const uint8_t *data, uint8_t data_len)
{
    I2c_Queue_Push(&i2c_tx_msg_queue, channel, 0, dev_address, reg_address, data, data_len);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
#define iic_scl_out()
#define iic_sda_out()
#define iic_scl_high()
#define iic_sda_high()
#define iic_sda_low()
#define iic_scl_low()
#define iic_sda_in()
#define iic_read_sda_() gpio_is_high(0, 0)

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

    iic_scl_high();       // Set SCL to high initially
    iic_sda_high();       // Set SDA to high initially
    platform_delay_us(5); // Small delay to ensure lines are stable

    i2c_clear_error(); // Clear any previous errors
}

// Start I2C communication (SDA goes low while SCL is high)
void i2c_start(void)
{
    iic_sda_out();
    iic_sda_high(); // Make sure SDA is high
    iic_scl_high(); // Ensure SCL is high
    platform_delay_us(5);

    iic_sda_low(); // Pull SDA low
    platform_delay_us(5);

    iic_scl_low(); // Pull SCL low to start communication
    platform_delay_us(5);
}

// Stop I2C communication (SDA goes high while SCL is high)
void i2c_stop(void)
{
    iic_sda_out();
    iic_scl_low(); // Pull SCL low first
    iic_sda_low(); // Then pull SDA low
    platform_delay_us(5);

    iic_scl_high(); // Pull SCL high to end communication
    platform_delay_us(5);

    iic_sda_high(); // Pull SDA high to signal stop
    platform_delay_us(5);
}

// Wait for ACK after sending a byte
uint8_t i2c_wait_ack(void)
{
    uint8_t retry = 0;

    iic_sda_in();   // Set SDA to input to read ACK
    iic_sda_high(); // Release SDA line
    platform_delay_us(1);

    iic_scl_high(); // Clock the ACK
    platform_delay_us(5);

    while (iic_read_sda_())
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
    platform_delay_us(2);

    iic_scl_high(); // Clock the ACK bit
    platform_delay_us(5);
    iic_scl_low(); // Pull SCL low after ACK
}

// Send a NACK bit to stop communication
void i2c_send_nack(void)
{
    iic_sda_out();  // Set SDA to output
    iic_sda_high(); // Pull SDA high for NACK
    platform_delay_us(2);

    iic_scl_high(); // Clock the NACK bit
    platform_delay_us(5);
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
        platform_delay_us(2);

        if (data & 0x80)
        { // Send the most significant bit first
            iic_sda_high();
        }
        else
        {
            iic_sda_low();
        }
        data <<= 1; // Shift data to send the next bit

        platform_delay_us(2);
        iic_scl_high(); // Clock the bit
        platform_delay_us(5);
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
        platform_delay_us(2);

        iic_scl_high(); // Clock the bit into SDA
        data <<= 1;
        if (iic_read_sda_())
        { // Read the bit from SDA
            data |= 0x01;
        }

        platform_delay_us(3);
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

