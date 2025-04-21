

#include "can_signal_rx_ipu5_0x103.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_IPU5[CAN_SIG_RX_IPU5_COUNT] = {
    [IPU5_FrntMotTStatr]        = {.id = 0x103, .message_name = CAN_MSG_RX_IPU5, .byte_pos = 0, .bit_pos =  0, .sig_len =  8, .default_val = 0x46, .invalid_val = 0xFF},  //电机温度(定子温度)
    [IPU5_FrntMotTRotr]         = {.id = 0x103, .message_name = CAN_MSG_RX_IPU5, .byte_pos = 1, .bit_pos =  8, .sig_len =  8, .default_val = 0x46, .invalid_val = 0xFF},  //电机转子温度信号
    [IPU5_FrntIpuTIgbt]         = {.id = 0x103, .message_name = CAN_MSG_RX_IPU5, .byte_pos = 2, .bit_pos = 16, .sig_len =  8, .default_val = 0x46, .invalid_val = 0xFF},  //电机控制器 IGBT温度
    [IPU5_FrntIpuTIgbtPhaU]     = {.id = 0x103, .message_name = CAN_MSG_RX_IPU5, .byte_pos = 3, .bit_pos = 24, .sig_len =  8, .default_val = 0x46, .invalid_val = 0xFF},  //电机控制器IGBT U相温度   
    [IPU5_FrntIpuTIgbtPhaV]     = {.id = 0x103, .message_name = CAN_MSG_RX_IPU5, .byte_pos = 4, .bit_pos = 32, .sig_len =  8, .default_val = 0x46, .invalid_val = 0xFF},  //电机控制器IGBT V相温度
    [IPU5_FrntIpuTIgbtPhaW]     = {.id = 0x103, .message_name = CAN_MSG_RX_IPU5, .byte_pos = 5, .bit_pos = 40, .sig_len =  8, .default_val = 0x46, .invalid_val = 0xFF},  //电机控制器IGBT W相温度
    [IPU5_FrntIpuT]             = {.id = 0x103, .message_name = CAN_MSG_RX_IPU5, .byte_pos = 6, .bit_pos = 48, .sig_len =  8, .default_val = 0x46, .invalid_val = 0xFF},  //电机控制器温度
    //[IPU5_FrntIpuTCoolt]        = {.id = 0x103, .message_name = CAN_MSG_RX_IPU5, .byte_pos = 7, .bit_pos = 56, .sig_len =  8, .default_val = 0x46, .invalid_val = 0xFF},  //电机控制器冷却液温度
    
};


uint8_t can_msg_0x103_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_IPU5[sig], sig_val);
}

void can_msg_0x103_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_IPU5_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_IPU5[i]);
    }
}

void can_msg_0x103_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}

