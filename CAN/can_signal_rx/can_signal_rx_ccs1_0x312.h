

#ifndef CAN_SIGNAL_RX_0x312
#define CAN_SIGNAL_RX_0x312

#include <stdint.h>
#include "../can_type.h"

//CCS1,//    0x312
typedef enum {
    CCS1_IC_BrightnessLevel,       //ICU亮度等级
    CCS1_SubtotalMileageClearing,  //小计里程清零
    CCS1_SubtotalTimeClearing,     //小计时间清零
    CCS1_MileageUnitSwitching,     //里程单位切换
    CCS1_LanguageSwitching,        //语言显示切换
    CCS1_TempUnitSwitching,        //温度单位切换
    //CCS1_PressureUnitSwitching,    //压力单位切换
    //CCS1_VolumeUnitSwitching,      //体积单位切换
    
    CAN_SIG_RX_CCS1_COUNT,
} CAN_SIG_RX_CCS1;

uint8_t can_msg_0x312_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x312_timeout_function(void);

void can_msg_0x312_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x312
