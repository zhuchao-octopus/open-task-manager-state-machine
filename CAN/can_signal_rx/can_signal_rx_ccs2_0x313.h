

#ifndef CAN_SIGNAL_RX_0x313
#define CAN_SIGNAL_RX_0x313

#include <stdint.h>
#include "../can_type.h"

//CCS2,//    0x313
typedef enum {
    CCS2_CentralControlPanel_Hours,     //中控屏小时数
    CCS2_CentralControlPanel_Minutes,   //中控屏分钟数
    CCS2_CentralControlPanel_Seconds,   //中控屏秒钟数
    CCS2_Age,                           //年份
    CCS2_Month,                         //月份
    CCS2_Date,                          //日期
    CCS2_Week,                          //星期
    
    CAN_SIG_RX_CCS2_COUNT,
} CAN_SIG_RX_CCS2;

uint8_t can_msg_0x313_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x313_timeout_function(void);

void can_msg_0x313_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x313
