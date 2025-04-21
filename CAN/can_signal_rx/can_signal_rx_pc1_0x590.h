
#ifndef CAN_SIGNAL_RX_0x590
#define CAN_SIGNAL_RX_0x590

#include <stdint.h>
#include "../can_type.h"

//PC1,//    0x590
typedef enum {
    PC1_ICU_CFG,    //仪表配置

    
    CAN_SIG_RX_PC1_COUNT,
} CAN_SIG_RX_PC1;

uint8_t can_msg_0x590_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x590_timeout_function(void);

void can_msg_0x590_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x590



