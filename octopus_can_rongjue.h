/******************************************************************************
 * @file    octopus_can_rongjue.h
 * @author  Octopus Embedded Team
 * @version V1.0
 * @date    2025-10-14
 * @brief   Header file for Rongjue 500 CAN communication functions.
 *
 * This header defines interfaces for dispatching and sending CAN messages
 * specific to the Rongjue 500 model. It provides message routing and
 * transmission APIs used by the CAN task or interrupt handler.
 *
 * ----------------------------------------------------------------------------
 * Revision History:
 * Date         Author              Notes
 * 2025-10-14   Macky Lee           Initial version
 * ----------------------------------------------------------------------------
 ******************************************************************************/

#ifndef __OCTOPUS_CAN_FUNCTION_RONGJUE__
#define __OCTOPUS_CAN_FUNCTION_RONGJUE__

/*==============================================================================
 * Includes
 *============================================================================*/

/**
 * @file octopus_base.h
 * @brief Base include file for core Octopus project definitions and macros.
 */
#include "octopus_base.h"

/**
 * @file octopus_can_queue.h
 * @brief CAN message queue structures and APIs.
 */
#include "octopus_can_queue.h"

/**
 * @file octopus_tickcounter.h
 * @brief Provides tick-based timing and delay utilities.
 */
#include "octopus_tickcounter.h"

/**
 * @file octopus_vehicle.h
 * @brief Defines the vehicle information structure and related parameters.
 */
#include "octopus_vehicle.h"

typedef struct
{
    bool ready_on;
    float speed_kmh;
    uint32_t odometer_km; /* integer km (or raw units) */
    float odometer_float; /* km with decimal if needed */
    uint8_t drive_mode;   /* enum: 0 normal, 1 ECO, 2 SPORT, ... */
    uint8_t gear;         /* 'P','R','N','D', or 0 */
    uint8_t battery_soc;
    float battery_current; /* A */
    float battery_temp;    /* degC */
    uint16_t battery_fault;
    uint8_t ecu_fault_code;
    uint8_t motor_fault_code;
} RongjueVehicleInfo_t;

/*==============================================================================
 * Function Declarations
 *============================================================================*/

#ifdef CUSTOMER_MODEL_RONGJUE_500

/**
 * @brief Dispatch a CAN message to the appropriate handler based on its ID.
 *
 * This function is responsible for message routing and decoding logic,
 * typically used in the CAN receive ISR or task context.
 *
 * @param queue_msg Pointer to a received CAN queue message structure.
 * @return true if message processed successfully, false otherwise.
 */
bool can_message_dispatcher(const CanQueueMsg_t *queue_msg);

/**
 * @brief Send a CAN message with a specific message ID.
 *
 * This function can be used to transmit periodic or event-driven messages
 * to other ECUs on the CAN bus.
 *
 * @param message_id The 11-bit or 29-bit CAN message identifier.
 * @return true if message sent successfully, false otherwise.
 */
bool can_message_sender(const uint16_t message_id);

#endif /* CUSTOMER_MODEL_RONGJUE_500 */

/*==============================================================================
 * End of File
 *============================================================================*/

#endif /* __OCTOPUS_CAN_FUNCTION_RONGJUE__ */

/************************ (C) COPYRIGHT Octopus Embedded Team *****END OF FILE****/
