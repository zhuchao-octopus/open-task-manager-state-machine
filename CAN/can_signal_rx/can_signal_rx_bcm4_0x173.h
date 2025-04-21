

#ifndef CAN_SIGNAL_RX_0x173
#define CAN_SIGNAL_RX_0x173

#include <stdint.h>
#include "../can_type.h"

//BCM4,//    0x173
typedef enum {
    BCM4_EPS_FaultCode,                     //EPS故障码
    BCM4_SensorMainCircuit_Voltage,         //传感器主路电压
    BCM4_SensorAuxiliaryCircuit_Voltage,    //传感器辅路电压
    BCM4_IgnitionLock_Voltage,              //点火锁电压
    BCM4_Motor_Current,                     //电机电流
    BCM4_CircuitBoard_Temperature,          //电路板温度

    CAN_SIG_RX_BCM4_COUNT,
} CAN_SIG_RX_BCM4;

uint8_t can_msg_0x173_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x173_timeout_function(void);

void can_msg_0x173_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x173

