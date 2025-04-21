
#ifndef CAN_MESSAGE_RX_H_
#define CAN_MESSAGE_RX_H_

#include <stdint.h>
#include <stdbool.h>
#include "can_type.h"

typedef enum{
    CAN_NODE_ECU,
    CAN_NODE_BCM,
    CAN_NODE_PKE,
    CAN_NODE_ICU,
    CAN_NODE_UDS,
    CAN_NODE_BCU,
    CAN_NODE_IPU,
    CAN_NODE_VCU,
    CAN_NODE_OBC,
    CAN_NODE_DCDC,
    CAN_NODE_CCS,
    CAN_NODE_MSC,
    CAN_NODE_SL,
    CAN_NODE_PC,

    CAN_NODE_NUM,
}CanNodeType;



typedef enum {
    CAN_MSG_RX_ECU1,        //0x109
    CAN_MSG_RX_ECU3,        //0x111
    CAN_MSG_RX_BCM2,        //0x170
    CAN_MSG_RX_BCM3,        //0x172
    CAN_MSG_RX_BCM4,        //0x173
    CAN_MSG_RX_BCM5,        //0x195
    CAN_MSG_RX_CCS1,        //0x312
    CAN_MSG_RX_CCS2,        //0x313

    CAN_MSG_RX_REQUEST_ID,   //0x781请求 ID
    CAN_MSG_RX_FUNCTION_ID,  //0x7DF功能 ID

    CAN_MSG_RX_BCU1,          //0x185
	
    CAN_MSG_RX_BCU2,          //0x203
    CAN_MSG_RX_BCU3,          //0x20B
	
    CAN_MSG_RX_BCU4,          //0x305
    CAN_MSG_RX_BCU5,          //0x207
    CAN_MSG_RX_BCU6,          //0x502
    CAN_MSG_RX_BCU7,          //0x209
    CAN_MSG_RX_IPU1,          //0x204
    CAN_MSG_RX_IPU2,          //0x100
    CAN_MSG_RX_IPU3,          //0x105
    CAN_MSG_RX_IPU4,          //0x106
    CAN_MSG_RX_IPU5,          //0x103
    CAN_MSG_RX_IPU6,          //0x202
    CAN_MSG_RX_IPU7,          //0x171
    CAN_MSG_RX_OBC1,          //0x319
    CAN_MSG_RX_OBC2,          //0x349
    CAN_MSG_RX_OBC3,          //0x3A5
    CAN_MSG_RX_DCDC1,         //0x299
    CAN_MSG_RX_MSC,           //0x180
    CAN_MSG_RX_SL1,           //0x188
    CAN_MSG_RX_PC1,           //0x590
    
    
    CAN_MSG_RX_NUM,
} CanReceiveID;


extern CAN_msg_rx_config CAN_msg_rx_config_case[CAN_MSG_RX_NUM];
    
#endif //CAN_MESSAGE_RX_H_

