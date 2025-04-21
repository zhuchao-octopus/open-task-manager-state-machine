// File: octopus_can.h
// Description: Header for CAN data dispatching and parsing logic
// Author: ak47
// Created: 2025-04-17

#ifndef OCTOPUS_CAN_H
#define OCTOPUS_CAN_H

#include <stdint.h>
#include <stddef.h>
// ========================================================
//                CAN Message ID Definitions
// ========================================================

// --------------------------
// Standard Frame IDs (11-bit)
// --------------------------

// Heartbeat message from the master node.
// Used for network presence and liveness check.
#define CAN_ID_HEARTBEAT_MASTER       0x100

// Heartbeat message from the slave node.
#define CAN_ID_HEARTBEAT_SLAVE        0x101

// Report status or health information from a node.
#define CAN_ID_STATUS_REPORT          0x200

// Sensor data frame, typically periodic broadcast.
#define CAN_ID_SENSOR_DATA            0x210

// Control command frame sent to actuators or other nodes.
#define CAN_ID_CONTROL_COMMAND        0x220

// Diagnostic request sent to a device for status or debug.
#define CAN_ID_DIAGNOSTIC_REQUEST     0x230

// Diagnostic response to a previously received request.
#define CAN_ID_DIAGNOSTIC_RESPONSE    0x231

// Custom application-defined message IDs.
#define CAN_ID_CUSTOM_1               0x300
#define CAN_ID_CUSTOM_2               0x321  // Example used in LCD_CAN_IRQHandler

// --------------------------
// Extended Frame IDs (29-bit)
// --------------------------

// Request to initiate firmware update.
#define CAN_EXT_ID_FIRMWARE_UPDATE_REQ   0x18FF1000

// Frame containing firmware binary data.
#define CAN_EXT_ID_FIRMWARE_UPDATE_DATA  0x18FF1001

// Configuration of runtime parameters (e.g., thresholds, modes).
#define CAN_EXT_ID_PARAMETER_CONFIG      0x18FF2000

// Notification of system events such as alarms or state transitions.
#define CAN_EXT_ID_EVENT_NOTIFICATION    0x18FF3000

// Extended frame for complex diagnostic information.
#define CAN_EXT_ID_DIAGNOSTIC_EXT        0x18FF4000

// --------------------------
// Global CAN Settings
// --------------------------

// Default CAN data length (in bytes)
#define CAN_DATA_LENGTH    8

#ifdef __cplusplus
extern "C" {
#endif

// Structure to represent a CAN message
typedef struct
{
    uint32_t StdId;  /*!< Specifies the standard identifier.
                        This parameter can be a value between 0 to 0x7FF. */

    uint32_t ExtId;  /*!< Specifies the extended identifier.
                        This parameter can be a value between 0 to 0x1FFFFFFF. */

    uint8_t IDE;     /*!< Specifies the type of identifier for the message that
                        will be received. This parameter can be a value of
                        @ref CAN_identifier_type */

    uint8_t RTR;     /*!< Specifies the type of frame for the received message.
                        This parameter can be a value of
                        @ref CAN_remote_transmission_request */

    uint8_t DLC;     /*!< Specifies the length of the frame that will be received.
                        This parameter can be a value between 0 to 8 */

    uint8_t Data[8]; /*!< Contains the data to be received. It ranges from 0 to
                        0xFF. */

    uint8_t FMI;     /*!< Specifies the index of the filter the message stored in
                        the mailbox passes through. This parameter can be a
                        value between 0 to 0xFF */
} CAN_Message_t;

///////////////////////////////////////////////////////////////////////////////////////////
//
// ========================================================
//                Function Declarations
// ========================================================

/**
 * @brief Parse a received CAN message and process it according to its ID.
 * @param msg Pointer to a CAN_Message_t structure containing received data.
 */
void parse_can_message(const CAN_Message_t *msg);

/**
 * @brief Dispatch a CAN message to appropriate handler based on its ID.
 *        This function may be used for routing logic in receive ISR or task.
 * @param msg Pointer to a CAN_Message_t structure.
 */
void dispatch_can_message(const CAN_Message_t *msg);

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef __cplusplus
}
#endif

#endif // OCTOPUS_CAN_H
