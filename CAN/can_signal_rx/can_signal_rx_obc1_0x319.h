

#ifndef CAN_SIGNAL_RX_0x319
#define CAN_SIGNAL_RX_0x319

#include <stdint.h>
#include "../can_type.h"

//OBC1,//    0x319
typedef enum {
    OBC1_ObcCpValVld,                             //CP信号状态
    OBC1_ObcChrgSts,                              //充电机工作状态

    
    CAN_SIG_RX_OBC1_COUNT,
} CAN_SIG_RX_OBC1;

uint8_t can_msg_0x319_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x319_timeout_function(void);

void can_msg_0x319_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x319

