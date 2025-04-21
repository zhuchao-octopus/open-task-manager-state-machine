
#ifndef CAN_SIGNAL_RX_0x202
#define CAN_SIGNAL_RX_0x202

#include <stdint.h>
#include "../can_type.h"

//IPU6,//    0x202
typedef enum {
    IPU6_IPUVehSpdVld,                                //车速有效状态
    IPU6_IPUVehSpd,                                   //车速：MCU以转速计算

    
    CAN_SIG_RX_IPU6_COUNT,
} CAN_SIG_RX_IPU6;

uint8_t can_msg_0x202_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x202_timeout_function(void);

void can_msg_0x202_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x202


