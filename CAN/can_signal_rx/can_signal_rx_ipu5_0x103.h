

#ifndef CAN_SIGNAL_RX_0x103
#define CAN_SIGNAL_RX_0x103

#include <stdint.h>
#include "../can_type.h"

//IPU5,//    0x103
typedef enum {
    IPU5_FrntMotTStatr,                         //电机温度(定子温度)
    IPU5_FrntMotTRotr,                          //电机转子温度信号
    IPU5_FrntIpuTIgbt,                          //电机控制器 IGBT温度
    IPU5_FrntIpuTIgbtPhaU,                      //电机控制器IGBT U相温度
    IPU5_FrntIpuTIgbtPhaV,                      //电机控制器IGBT V相温度
    IPU5_FrntIpuTIgbtPhaW,                      //电机控制器IGBT W相温度
    IPU5_FrntIpuT,                              //电机控制器温度
    //IPU5_FrntIpuTCoolt,                         //电机控制器冷却液温度
    
    CAN_SIG_RX_IPU5_COUNT,
} CAN_SIG_RX_IPU5;

uint8_t can_msg_0x103_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x103_timeout_function(void);

void can_msg_0x103_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x103

