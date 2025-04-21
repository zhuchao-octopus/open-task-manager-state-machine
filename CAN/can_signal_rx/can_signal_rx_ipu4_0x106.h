

#ifndef CAN_SIGNAL_RX_0x106
#define CAN_SIGNAL_RX_0x106

#include <stdint.h>
#include "../can_type.h"

//IPU4,//    0x106
typedef enum {
    IPU4_FrntIpuErrNr,                             //电机故障代码

    
    CAN_SIG_RX_IPU4_COUNT,
} CAN_SIG_RX_IPU4;

uint8_t can_msg_0x106_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x106_timeout_function(void);

void can_msg_0x106_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x106

