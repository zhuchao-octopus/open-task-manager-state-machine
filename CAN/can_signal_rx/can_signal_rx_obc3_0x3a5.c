
#include "can_signal_rx_obc3_0x3a5.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_OBC3[CAN_SIG_RX_OBC3_COUNT] = {
    [OBC3_OBCPlugCnctLamp]       = {.id = 0x3A5, .message_name = CAN_MSG_RX_OBC3, .byte_pos = 1, .bit_pos = 10, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //充电线连接指示灯
    
};


uint8_t can_msg_0x3a5_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_OBC3[sig], sig_val);
}

void can_msg_0x3a5_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_OBC3_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_OBC3[i]);
    }
}

void can_msg_0x3a5_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}




