

#ifndef CAN_SIGNAL_RX_0x204
#define CAN_SIGNAL_RX_0x204

#include <stdint.h>
#include "../can_type.h"

//IPU1,//    0x204
typedef enum {
    //IPU1_FrntIpuIDc,                                  //电机控制器直流侧电流
    //IPU1_FrntIpuUDcToRmu,                             //电机控制器直流侧电压
    IPU1_IPUVehRange,                                 //续航里程
    IPU1_IPUInsttPwrCspt,                             //瞬时耗电量(kWh/100Km)
    IPU1_MotSysPwrLmtLp,                              //电机系统限功率指示灯
    IPU1_IPUReadyLp,                                  //Ready信号指示灯
    
    
    CAN_SIG_RX_IPU1_COUNT,
} CAN_SIG_RX_IPU1;

uint8_t can_msg_0x204_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x204_timeout_function(void);

void can_msg_0x204_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x204

