
#ifndef CAN_SIGNAL_RX_0x3A5
#define CAN_SIGNAL_RX_0x3A5

#include <stdint.h>
#include "../can_type.h"

//OBC3,//    0x3a5
typedef enum {
    OBC3_OBCPlugCnctLamp,                             //充电线连接指示灯
    
    CAN_SIG_RX_OBC3_COUNT,
} CAN_SIG_RX_OBC3;

uint8_t can_msg_0x3a5_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x3a5_timeout_function(void);

void can_msg_0x3a5_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x3a5



