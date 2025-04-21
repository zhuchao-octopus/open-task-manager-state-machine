

#ifndef CAN_SIGNAL_RX_0x203
#define CAN_SIGNAL_RX_0x203

#include <stdint.h>
#include "../can_type.h"

//BCU2,//    0x203
typedef enum {
    BCU2_BCUMaxDchaPwrShoT,                             //电池最大放电功率(短时) 
    BCU2_BCUMaxChrgPwrShoT,                             //电池最大放电功率(短时)
    BCU2_BCUMaxDchaIShoT,                               //电池最大放电功率(短时)
    BCU2_BCUMaxChrgIShoT,                               //电池最大放电功率(短时)
    BCU2_BCU215CycCntr,                                 //本帧报文计数
    BCU2_BcuRlyWlddErr,                                 //主继电器粘连故障
    
    
    CAN_SIG_RX_BCU2_COUNT,
} CAN_SIG_RX_BCU2;

uint8_t can_msg_0x203_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x203_timeout_function(void);

void can_msg_0x203_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x203

