

#ifndef CAN_SIGNAL_RX_0x185
#define CAN_SIGNAL_RX_0x185

#include <stdint.h>
#include "../can_type.h"

//BCU1,//    0x185
typedef enum {
    BCU1_BCUBattU,                            //电池总电压
    BCU1_BCUBattI,                            //电池总电流
    //BCU1_BCUInsulationSts,                    //绝缘状态
    //BCU1_BcuHeatSftySts,                      //电池热失效状态
    BCM1_BCUSOC,                              //电池荷电量SOC
    //BCU1_BCUHVILSts,                          //高压互锁状态
    //BCU1_BCUPwrUpAllw,                        //允许重新上高压电
    //BCU1_BCUOBCOperModReq,                    //OBC工作模式请求
    //BCU1_BCUOperMod,                          //BMS工作模式
    //BCU1_BCU185CycCntr,                       //本帧报文计数
    //BCU1_BCUFltRnk,                           //电池故障等级
    //BCU1_BCUCRCChk185,                        //校验
    
    CAN_SIG_RX_BCU1_COUNT,
} CAN_SIG_RX_BCU1;

uint8_t can_msg_0x185_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x185_timeout_function(void);

void can_msg_0x185_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x185

