

#ifndef CAN_SIGNAL_RX_0x209
#define CAN_SIGNAL_RX_0x209

#include <stdint.h>
#include "../can_type.h"

//BCU7,//    0x209
typedef enum {
    BCU7_BCUErrAmnt,                                    //故障数量
    BCU7_BCUErrNum,                                     //故障代码
    //BCU7_BCUHVPower1S,                                   //整车瞬时功率计算

    
    CAN_SIG_RX_BCU7_COUNT,
} CAN_SIG_RX_BCU7;

uint8_t can_msg_0x209_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x209_timeout_function(void);

void can_msg_0x209_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x209

