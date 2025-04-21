
#ifndef CAN_SIGNAL_RX_0x180
#define CAN_SIGNAL_RX_0x180

#include <stdint.h>
#include "../can_type.h"

//MSC,//    0x180
typedef enum {

    MSC_CurrentG,                       //当前档位信号
    MSC_CurrentG_flash,                 //当前档位闪烁信号
    MSC_angleSensorFlt,                 //MSC角度传感器损坏
    MSC_gearSwitchFlt,                  //MSC档位开关工作
    MSC_motorOpenFlt,                   //MSC电机开路
    MSC_motorShortFlt,                  //MSC电机短路
    MSC_OLgearShiftFlt,                 //MSC重载换挡失败
    MSC_LLgearShiftFlt,                 //MSC轻载换挡失败
    MSC_self_learningFlt,               //MSC自学习失败
    MSC_reserved,                       //预留

    CAN_SIG_RX_MSC_COUNT,
} CAN_SIG_RX_MSC;

uint8_t can_msg_0x180_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x180_timeout_function(void);

void can_msg_0x180_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x180


