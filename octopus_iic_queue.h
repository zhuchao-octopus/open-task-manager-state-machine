/*
 * octopus_iic.h
 *
 * @version 1.0
 * @date    2025-05-15
 * @author  Octopus Team
 *
 * Description:
 *   This header file defines the data structures and function interfaces for a circular
 *   message queue used to temporarily store I²C messages.
 *
 *   The queue is mainly designed to decouple I²C message transmission/reception
 *   (often happening in interrupt context or ISR) from their processing in the main loop
 *   or in another RTOS task/thread.
 *
 * Features:
 *   - Ring buffer implementation with head/tail indices
 *   - Non-blocking push and pop operations
 *   - Efficient modulo operation using bitmask (queue size must be a power of 2)
 *   - Designed for embedded real-time environments (interrupt + task separation)
 *
 * Typical Usage:
 *   - In ISR: Call I2c_Queue_Push() to push a received I²C message
 *   - In main loop / task: Call I2c_Queue_Pop() to retrieve and process messages
 */

#ifndef __OCTOPUS_IIC_QUEUE_H__
#define __OCTOPUS_IIC_QUEUE_H__

#include "octopus_base.h" // Base include file for the Octopus project.

/* ========================================================================================
 *        Configuration Macros
 * ======================================================================================== */

/**
 * @brief Size of the I²C message queue (must be a power of 2).
 *        Larger size increases buffering capability but uses more RAM.
 */
#define I2C_MSG_QUEUE_SIZE (128U)

/**
 * @brief Mask used for efficient index wrapping (equivalent to modulo).
 *        Works only if I2C_MSG_QUEUE_SIZE is a power of 2.
 */
// I2C_MSG_QUEUE_SIZE_MASK 主要起到 取模运算（wrap-around） 的作用
// 用来让环形队列的 head 和 tail 指针在到达末尾后自动回到队列开头

#define I2C_MSG_QUEUE_SIZE_MASK (I2C_MSG_QUEUE_SIZE - 1)

/* ========================================================================================
 *        Data Structures
 * ======================================================================================== */

#pragma pack(push, 1)

/**
 * @brief Structure representing a single I²C message stored in the queue.
 */
typedef struct
{
    uint8_t channel;
    uint8_t oparation;
    uint8_t dev_address; /**< I²C device address */
    uint8_t reg_address; /**< I²C register address */
    uint8_t data_len;    /**< Length of valid data in @ref data */
    uint8_t data[128];   /**< Data payload (up to 255 bytes) */
} I2c_QueueMsg_t;

/**
 * @brief Structure representing the circular I²C message queue.
 */
typedef struct
{
    uint16_t head;                          /**< Index of the next message to be popped */
    uint16_t tail;                          /**< Index of the next free slot for pushing */
    I2c_QueueMsg_t msg[I2C_MSG_QUEUE_SIZE]; /**< Internal buffer to store I²C messages */
} I2cQueue_t;

#pragma pack(pop)

/* ========================================================================================
 *        Function Prototypes
 * ======================================================================================== */

/**
 * @brief Initializes the I²C message queue.
 * @param queue Pointer to the I²C queue instance to be initialized.
 */
void I2c_Queue_Init(I2cQueue_t *queue);

/**
 * @brief Returns the current number of messages stored in the queue.
 * @param queue Pointer to the I²C queue instance.
 * @return Number of messages currently in the queue.
 */
uint16_t I2c_Queue_Length(I2cQueue_t *queue);

/**
 * @brief Attempts to push a new I²C message into the queue.
 * @param queue Pointer to the I²C queue instance.
 * @param dev_address I²C device address.
 * @param reg_address I²C register address.
 * @param data Pointer to the I²C data payload.
 * @param data_len Length of the payload.
 * @return 0 if push was successful, 1 if the queue is full.
 */
uint8_t I2c_Queue_Push(I2cQueue_t *queue, uint8_t channel, uint8_t oparation, uint8_t dev_address, uint8_t reg_address, const uint8_t *data, uint8_t data_len);

/**
 * @brief Attempts to pop the oldest I²C message from the queue.
 * @param queue Pointer to the I²C queue instance.
 * @param msg Output pointer to receive the popped message.
 * @return 1 if a message was successfully popped, 0 if the queue was empty.
 */
uint8_t I2c_Queue_Pop(I2cQueue_t *queue, I2c_QueueMsg_t *msg);

/**
 * @brief Provides access to the next I²C message (without popping).
 * @return Pointer to the next I²C message, or NULL if queue is empty.
 */
I2c_QueueMsg_t *I2c_GetMsg(void);

/**
 * @brief Returns the maximum queue size (for external reference).
 * @return Size of the I²C queue buffer.
 */
uint16_t I2c_GetMsgQueueSize(void);

/* ========================================================================================
 *        Global Variables
 * ======================================================================================== */

/**
 * @brief Global I²C TX queue instance.
 */
extern I2cQueue_t i2c_tx_msg_queue;

#endif /* __OCTOPUS_IIC_H_ */
