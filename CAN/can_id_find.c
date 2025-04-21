#include "can_id_find.h"

#include <stdint.h>
#include "can_message_rx.h"
#include "can_message_tx.h"

typedef struct {
    uint16_t name;
    uint16_t id;
}CAN_msg_id_map;



const CAN_msg_id_map can_tree_rx[] = {
    {CAN_MSG_RX_ECU1,        0x109},
    {CAN_MSG_RX_ECU3,        0x111},

    {CAN_MSG_RX_BCM2,        0x170},
    {CAN_MSG_RX_BCM3,        0x172},
    {CAN_MSG_RX_BCM4,        0x173},
    {CAN_MSG_RX_BCM5,        0x195},

    {CAN_MSG_RX_CCS1,        0x312},
    {CAN_MSG_RX_CCS2,        0x313},

    {CAN_MSG_RX_REQUEST_ID,  0x781},
    {CAN_MSG_RX_FUNCTION_ID, 0x7DF},

    {CAN_MSG_RX_BCU1,        0x185},
    {CAN_MSG_RX_BCU4,        0x305},
    {CAN_MSG_RX_BCU5,        0x207},
    {CAN_MSG_RX_BCU6,        0x502},
    {CAN_MSG_RX_BCU7,        0x209},
    {CAN_MSG_RX_IPU1,        0x204},
    {CAN_MSG_RX_IPU2,        0x100},
    {CAN_MSG_RX_IPU3,        0x105},
    {CAN_MSG_RX_IPU4,        0x106},
    {CAN_MSG_RX_IPU5,        0x103},
    {CAN_MSG_RX_IPU6,        0x202},
    {CAN_MSG_RX_IPU7,        0x171},
    {CAN_MSG_RX_OBC1,        0x319},
    {CAN_MSG_RX_OBC2,        0x349},
    {CAN_MSG_RX_OBC3,        0x3A5},
    {CAN_MSG_RX_DCDC1,       0x299},
    {CAN_MSG_RX_MSC,         0x180},
    {CAN_MSG_RX_SL1,         0x188},
    {CAN_MSG_RX_PC1,         0x590},
    
};


const CAN_msg_id_map can_tree_tx[] = {
    {CAN_MSG_TX_ICU,    0x309},
    {CAN_MSG_TX_ICU1,   0x310},
    {CAN_MSG_TX_ICU2,   0x316},
    
    {CAN_MSG_TX_RESPONSE_ID, 0x791},
};




/*
    FIND CAN ID MESSAGE NAME BY CAN ID TREE
*/
int findCanIdMessageNameRx(uint32_t can_id)
{
    for (int i = 0; i < sizeof(can_tree_rx)/sizeof(CAN_msg_id_map); i++)
    {
        if (can_tree_rx[i].id == can_id) {
            return can_tree_rx[i].name;
        }
    }

    return -1;
}

int findCanIdMessageNameTx(uint32_t can_id)
{
        for (int i = 0; i < sizeof(can_tree_tx)/sizeof(CAN_msg_id_map); i++)
    {
        if (can_tree_tx[i].id == can_id) {
            return can_tree_tx[i].name;
        }
    }

    return -1;
}
