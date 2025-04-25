

#include "can_signal_rx_ipu2_0x100.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"



const CAN_signal_config CAN_signal_rx_IPU2[CAN_SIG_RX_IPU2_COUNT] = {
    [IPU2_FrntMotSpd]       = {.id = 0x100, .message_name = CAN_MSG_RX_IPU2, .byte_pos = 1, .bit_pos =  8, .sig_len = 16, .default_val = 0x4E20, .invalid_val = 0x4E20},  //电机转速
    [IPU2_FrntMotSpdVld]    = {.id = 0x100, .message_name = CAN_MSG_RX_IPU2, .byte_pos = 4, .bit_pos = 35, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //电机转速有效位
    [IPU2_FrntIpuFltRnk]    = {.id = 0x100, .message_name = CAN_MSG_RX_IPU2, .byte_pos = 4, .bit_pos = 36, .sig_len =  3, .default_val =    0x0, .invalid_val =    0x0},  //电机控制器故障等级
    
   
};


uint8_t can_msg_0x100_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_IPU2[sig], sig_val);
}

void can_msg_0x100_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_IPU2_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_IPU2[i]);
    }
}

void can_msg_0x100_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}

