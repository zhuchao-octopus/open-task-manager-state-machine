

#ifndef CAN_SIGNAL_RX_0x200
#define CAN_SIGNAL_RX_0x200

#include <stdint.h>
#include "../can_type.h"

//PKE2,//    0x200
typedef enum {
    PKE2_key,                   //钥匙信号
    PKE2_SearchCarSts,          //寻车功能反馈
    PKE2_RKE_BAT_LOW,           //钥匙低电量报警
    PKE2_Front_Break,           //刹车信号
    PKE2_PduLvl,                //电源档位
    PKE2_BodyGuardAgainstTheft, //防盗状态
    PKE2_One_Start_SwSts,       //ONE_START开关
    PKE2_BatteryVlotage,        //电池电压 

    CAN_SIG_RX_PKE2_COUNT,
} CAN_SIG_RX_PKE2;

uint8_t can_msg_0x200_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x200_timeout_function(void);

void can_msg_0x200_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x200
