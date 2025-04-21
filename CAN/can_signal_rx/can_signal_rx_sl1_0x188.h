
#ifndef CAN_SIGNAL_RX_0x188
#define CAN_SIGNAL_RX_0x188

#include <stdint.h>
#include "../can_type.h"

//SL1,//    0x188
typedef enum {
    SL1_SL_Reserve1,                             //预留
    SL1_SL_Reserve2,                             //预留
    SL1_SL_Reserve3,                             //预留
    SL1_SL_Reserve4,                             //预留
    SL1_SL_Reserve5,                             //预留
    SL1_SL_Reserve6,                             //预留
    SL1_SL_Reserve7,                             //预留
    SL1_SL_Reserve8,                             //预留
    
    CAN_SIG_RX_SL1_COUNT,
} CAN_SIG_RX_SL1;

uint8_t can_msg_0x188_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x188_timeout_function(void);

void can_msg_0x188_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x188



