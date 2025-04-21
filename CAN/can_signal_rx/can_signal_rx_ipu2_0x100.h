

#ifndef CAN_SIGNAL_RX_0x100
#define CAN_SIGNAL_RX_0x100

#include <stdint.h>
#include "../can_type.h"

//IPU2,//    0x100
typedef enum {
    IPU2_FrntMotSpd,                             //电机转速
    IPU2_FrntMotSpdVld,                          //电机转速有效位
    IPU2_FrntIpuFltRnk,                          //电机控制器故障等级

    
    CAN_SIG_RX_IPU2_COUNT,
} CAN_SIG_RX_IPU2;

uint8_t can_msg_0x100_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x100_timeout_function(void);

void can_msg_0x100_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x100

