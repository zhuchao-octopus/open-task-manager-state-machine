

#ifndef CAN_SIGNAL_RX_0x111
#define CAN_SIGNAL_RX_0x111

#include <stdint.h>
#include "../can_type.h"



//ECU3,//    0x111
typedef enum {
    ECU3_ECU_BatteryVoltage,        //电池电压 
    ECU3_Fault_Coad,                //故障代码

    CAN_SIG_RX_ECU3_COUNT,
} CAN_SIG_RX_ECU3;

uint8_t can_msg_0x111_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x111_timeout_function(void);

void can_msg_0x111_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x111
