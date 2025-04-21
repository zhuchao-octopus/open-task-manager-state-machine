

#ifndef CAN_SIGNAL_RX_0x305
#define CAN_SIGNAL_RX_0x305

#include <stdint.h>
#include "../can_type.h"

//BCU4,//    0x305
typedef enum {
    //BCU4_BCUChrgUReq,                                   //外插充电需求电压
    //BCU4_BcuAcChrgIReq,                                 //外插交流充电需求电流
    BCU4_BCUChrgSts,                                    //电池充电状态
    //BCU4_BCUChrgErrInfo,                                //BCU充电过程故障监控
    //BCU4_BCU305CycCntr,                                 //本帧报文计数
    //BCU4_BCUChrgMod,                                    //电池充电模式请求
    //BCU4_BCUCRCChk305,                                  //校验
    
    
    CAN_SIG_RX_BCU4_COUNT,
} CAN_SIG_RX_BCU4;

uint8_t can_msg_0x305_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x305_timeout_function(void);

void can_msg_0x305_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x305

