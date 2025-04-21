
#ifndef CAN_SIGNAL_TX_H_
#define CAN_SIGNAL_TX_H_

#include <stdint.h>
#include <stdbool.h>
#include "../can_type.h"


/* 发送信号 */
typedef enum {
    //0x309
    ICU_IC_Status_NM,                       //网络管理状态
    //0x310
    ICU1_IcuTotMilg,                        //总里程
    ICU1_IcuTotDriHrs,                      //发动机工作总小时数
    ICU1_IcuReq,                            //总计里程、发动机总工作小时数请求
    ICU1_StorgErrOrIcuErr,                  //BCM里程存储值故障或仪表故障
    ICU1_Environment_Temp,                  //环境温度
    //0x316
    ICU2_TripMileage,                       //小计里程
    ICU2_IcuPwrConsptPer100kilomt,          //百公里耗电量(kWh/100Km)
    ICU2_TripTime,                          //小计时间
    

    CAN_TX_SIG_count,
}CanTxSig;

extern CAN_signal_config CAN_signal_tx_info[CAN_TX_SIG_count];

#endif //CAN_SIGNAL_TX_H_

