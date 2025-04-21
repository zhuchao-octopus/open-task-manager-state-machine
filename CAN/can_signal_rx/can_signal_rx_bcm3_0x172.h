


#ifndef CAN_SIGNAL_RX_0x172
#define CAN_SIGNAL_RX_0x172

#include <stdint.h>
#include "../can_type.h"

//BCM3,//    0x172
typedef enum {
    BCM3_BcmTotMilg,        //总计里程储存值
    BCM3_BcmTotDriHrs,      //行驶总小时数储存值   
    BCM3_BcmAllwUp,         //BCM允许ICU更新总计里程、行驶总小时数             

    CAN_SIG_RX_BCM3_COUNT,
} CAN_SIG_RX_BCM3;

uint8_t can_msg_0x172_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x172_timeout_function(void);

void can_msg_0x172_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x172
