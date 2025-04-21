

#ifndef CAN_SIGNAL_RX_0x20B
#define CAN_SIGNAL_RX_0x20B

#include <stdint.h>
#include "../can_type.h"

//BCU3,//    0x20B
typedef enum {
    BCU3_BCUMaxDchaPwrLongT,                             //电池最大放电功率(长时) 
    BCU3_BCUMaxChrgPwrLongT,                             //电池最大放电功率(长时)
    BCU3_BCUMaxDchaILongT,                               //电池最大放电功率(长时)
    BCU3_BCUMaxChrgILongT,                               //电池最大放电功率(长时)
    BCU3_BCU235CycCntr,                                  //本帧报文计数
    BCU3_BCUMinDchaIU,                                   //电池最小放电电压
    BCU3_BCUCRCChk235,                                   //校验
    
    
    CAN_SIG_RX_BCU3_COUNT,
} CAN_SIG_RX_BCU3;

uint8_t can_msg_0x20B_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x20B_timeout_function(void);

void can_msg_0x20B_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x20B

