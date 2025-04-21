

#ifndef CAN_SIGNAL_RX_0x299
#define CAN_SIGNAL_RX_0x299

#include <stdint.h>
#include "../can_type.h"

//DCDC1,//    0x299
typedef enum {
    DCDC1_DcdcErrNr,                          //DCDC故障码

    
    CAN_SIG_RX_DCDC1_COUNT,
} CAN_SIG_RX_DCDC1;

uint8_t can_msg_0x299_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x299_timeout_function(void);

void can_msg_0x299_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x299

