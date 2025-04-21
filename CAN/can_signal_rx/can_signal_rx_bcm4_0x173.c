
#include "can_signal_rx_bcm4_0x173.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_BCM4[CAN_SIG_RX_BCM4_COUNT] = {
    [BCM4_EPS_FaultCode]                    = {.id = 0x173, .message_name = CAN_MSG_RX_BCM4, .byte_pos = 0, .bit_pos =  0, .sig_len =  8, .default_val = 0x0, .invalid_val = 0x0},  //EPS故障码
    [BCM4_SensorMainCircuit_Voltage]        = {.id = 0x173, .message_name = CAN_MSG_RX_BCM4, .byte_pos = 1, .bit_pos =  8, .sig_len = 16, .default_val = 0x0, .invalid_val = 0x0},  //传感器主路电压
    [BCM4_SensorAuxiliaryCircuit_Voltage]   = {.id = 0x173, .message_name = CAN_MSG_RX_BCM4, .byte_pos = 3, .bit_pos = 24, .sig_len = 16, .default_val = 0x0, .invalid_val = 0x0},  //传感器辅路电压
    [BCM4_IgnitionLock_Voltage]             = {.id = 0x173, .message_name = CAN_MSG_RX_BCM4, .byte_pos = 5, .bit_pos = 40, .sig_len =  8, .default_val = 0x0, .invalid_val = 0x0},  //点火锁电压
    [BCM4_Motor_Current]                    = {.id = 0x173, .message_name = CAN_MSG_RX_BCM4, .byte_pos = 6, .bit_pos = 48, .sig_len =  8, .default_val = 0x0, .invalid_val = 0x0},  //电机电流
    [BCM4_CircuitBoard_Temperature]         = {.id = 0x173, .message_name = CAN_MSG_RX_BCM4, .byte_pos = 7, .bit_pos = 56, .sig_len =  8, .default_val = 0x0, .invalid_val = 0x0},  //电路板温度
};


uint8_t can_msg_0x173_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_BCM4[sig], sig_val);
}

void can_msg_0x173_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCM4_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_BCM4[i]);
    }
}

void can_msg_0x173_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));
    //app_car_warn_proc_BCM4();
}


