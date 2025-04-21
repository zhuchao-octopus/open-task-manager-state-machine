
#ifndef CAN_MESSAGE_TX_H_
#define CAN_MESSAGE_TX_H_

#include <stdint.h>
#include <stdbool.h>
#include "can_type.h"

/*********************************************************************
 * CONSTANTS
 */

#define TX_MSGBOX_START    (1UL)
#define TX_MSGBOX_NUMS     (4UL)

typedef enum {
    CAN_MSG_TX_ICU, //0x309
    CAN_MSG_TX_ICU1, //0x310
    CAN_MSG_TX_ICU2, //0x316

    CAN_MSG_TX_RESPONSE_ID,//0x791应答 ID
    
    CAN_MSG_TX_NUM,
} CanSendID;

extern CAN_msg_tx_config CAN_msg_tx_config_case[CAN_MSG_TX_NUM];

#endif //CAN_MESSAGE_TX_H_

