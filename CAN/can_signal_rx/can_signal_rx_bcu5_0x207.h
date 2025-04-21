

#ifndef CAN_SIGNAL_RX_0x207
#define CAN_SIGNAL_RX_0x207

#include <stdint.h>
#include "../can_type.h"

//BCU5,//    0x207
typedef enum {
    BCU5_BCUMILReq,                                   //电池系统故障请求

    
    CAN_SIG_RX_BCU5_COUNT,
} CAN_SIG_RX_BCU5;

uint8_t can_msg_0x207_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x207_timeout_function(void);

void can_msg_0x207_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x207

