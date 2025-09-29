/*
 * octopus_iic_queue.c
 *
 * @version 1.0
 * @date    2025-05-15
 * @author  Octopus Team
 *
 * Description:
 *   This source file implements a fixed-size circular queue to store I²C messages.
 *   It is typically used in embedded systems where I²C messages are received via
 *   interrupt (ISR) and need to be processed asynchronously in the main loop or
 *   RTOS task thread.
 */

#include "octopus_iic_queue.h"

#if 1 // def TASK_MANAGER_STATE_MACHINE_I2C

/* Global TX queue instance */
I2cQueue_t i2c_tx_msg_queue;

/**
 * @brief Initializes the I²C queue by resetting the head and tail pointers.
 */
void I2c_Queue_Init(I2cQueue_t *queue)
{
    queue->head = 0;
    queue->tail = 0;
}

/**
 * @brief Calculates the number of messages currently in the queue.
 *
 * @return Number of messages in the queue.
 */
uint16_t I2c_Queue_Length(I2cQueue_t *queue)
{
    return (queue->tail + I2C_MSG_QUEUE_SIZE - queue->head) & I2C_MSG_QUEUE_SIZE_MASK;
}

/**
 * @brief Pushes an I²C message into the queue if there is available space.
 *
 * @param queue Pointer to the I²C queue structure.
 * @param dev_address I²C device address.
 * @param reg_address I²C register address.
 * @param data Pointer to the data payload.
 * @param data_len Length of the payload.
 * @return 0 if push was successful, 1 if the queue is full.
 */
uint8_t I2c_Queue_Push(I2cQueue_t *queue, uint8_t channel, uint8_t oparation, uint8_t dev_address, uint8_t reg_address, const uint8_t *data, uint8_t data_len)
{
    if (I2c_Queue_Length(queue) < I2C_MSG_QUEUE_SIZE)
    {
        uint8_t i;
        queue->msg[queue->tail].channel = channel;
        queue->msg[queue->tail].oparation = oparation;
        queue->msg[queue->tail].dev_address = dev_address;
        queue->msg[queue->tail].reg_address = reg_address;
        queue->msg[queue->tail].data_len = data_len;

        for (i = 0; i < data_len; i++)
        {
            queue->msg[queue->tail].data[i] = data[i];
        }

        queue->tail = (queue->tail + 1) & I2C_MSG_QUEUE_SIZE_MASK;
        return 0; // Success
    }

    return 1; // Queue full
}

/**
 * @brief Pops the oldest I²C message from the queue if available.
 *
 * @param msg Pointer to store the popped message.
 * @return 1 if a message was popped successfully, 0 if the queue was empty.
 */
uint8_t I2c_Queue_Pop(I2cQueue_t *queue, I2c_QueueMsg_t *msg)
{
    if (I2c_Queue_Length(queue) > 0)
    {
        *msg = queue->msg[queue->head];
        queue->head = (queue->head + 1) & I2C_MSG_QUEUE_SIZE_MASK;
        return 1;
    }
    return 0;
}

/**
 * @brief Retrieves one I²C message from the global TX queue.
 * @return Pointer to message if available, NULL otherwise.
 */
I2c_QueueMsg_t *I2c_GetMsg(void)
{
    static I2c_QueueMsg_t s_msg;
    if (I2c_Queue_Pop(&i2c_tx_msg_queue, &s_msg))
    {
        return &s_msg;
    }
    return NULL;
}

/**
 * @brief Returns the current number of messages in the global TX queue.
 */
uint16_t I2c_GetMsgQueueSize(void)
{
    return I2c_Queue_Length(&i2c_tx_msg_queue);
}

#endif
