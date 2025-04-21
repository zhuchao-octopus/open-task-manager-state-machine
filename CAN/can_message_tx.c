
/*****************************************************************************/
#include "can_message_tx.h"

#include <stdlib.h>


/* CAN_msg_tx_config */
CAN_msg_tx_config CAN_msg_tx_config_case[CAN_MSG_TX_NUM] = {
        [CAN_MSG_TX_ICU ] = {.instance = 0,  .id = 0x309, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period = 500,  .period_cnt = 20, .tx_enable = 0, .format = CAN_MSG_FORMAT_INTEL, .mailbox = (TX_MSGBOX_START+0), .status = 0, .timeout = 500, .callback = NULL},
        [CAN_MSG_TX_ICU1] = {.instance = 0,  .id = 0x310, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period =  40,  .period_cnt = 16, .tx_enable = 0, .format = CAN_MSG_FORMAT_INTEL, .mailbox = (TX_MSGBOX_START+1), .status = 0, .timeout =  40, .callback = NULL},
        [CAN_MSG_TX_ICU2] = {.instance = 0,  .id = 0x316, .type = CAN_MSG_TYPE_PERIOD, .data_len = 8, .period =  40,  .period_cnt = 16, .tx_enable = 0, .format = CAN_MSG_FORMAT_INTEL, .mailbox = (TX_MSGBOX_START+2), .status = 0, .timeout =  40, .callback = NULL},
        
        [CAN_MSG_TX_RESPONSE_ID] = {.instance = 0,  .id = 0x791, .type = CAN_MSG_TYPE_EVENT, .data_len = 8, .period =  40,  .period_cnt = 16, .tx_enable = 0, .format = CAN_MSG_FORMAT_INTEL, .mailbox = (TX_MSGBOX_START+3), .status = 0, .timeout =  40, .callback = NULL},
};
