

#ifndef CAN_SIGNAL_RX_0x105
#define CAN_SIGNAL_RX_0x105

#include <stdint.h>
#include "../can_type.h"

//IPU3,//    0x105
typedef enum {
    IPU3_FrntMotTOver,                             //电机温度过高
    IPU3_IPUMTPower,                               //电机瞬时功率 
    
    CAN_SIG_RX_IPU3_COUNT,
} CAN_SIG_RX_IPU3;

uint8_t can_msg_0x105_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x105_timeout_function(void);

void can_msg_0x105_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x100

