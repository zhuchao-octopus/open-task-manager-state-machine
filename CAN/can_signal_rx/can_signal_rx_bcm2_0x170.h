

#ifndef CAN_SIGNAL_RX_0x170
#define CAN_SIGNAL_RX_0x170

#include <stdint.h>
#include "../can_type.h"

//BCM2,//    0x170
typedef enum {

    BCM2_BCMFault_Coad,

    CAN_SIG_RX_BCM2_COUNT,
} CAN_SIG_RX_BCM2;

uint8_t can_msg_0x170_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x170_timeout_function(void);

void can_msg_0x170_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x170
