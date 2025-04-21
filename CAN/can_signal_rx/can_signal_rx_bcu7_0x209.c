

#include "can_signal_rx_bcu7_0x209.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_BCU7[CAN_SIG_RX_BCU7_COUNT] = {
    [BCU7_BCUErrAmnt]       = {.id = 0x209, .message_name = CAN_MSG_RX_BCU7, .byte_pos = 0, .bit_pos =  0, .sig_len =  8, .default_val =    0x0, .invalid_val =    0x0},  //故障数量
    [BCU7_BCUErrNum]        = {.id = 0x209, .message_name = CAN_MSG_RX_BCU7, .byte_pos = 2, .bit_pos = 16, .sig_len = 16, .default_val =    0x0, .invalid_val =  0x7FF},  //故障代码
    //[BCU7_BCUHVPower1S]     = {.id = 0x209, .message_name = CAN_MSG_RX_BCU7, .byte_pos = 4, .bit_pos = 32, .sig_len = 16, .default_val =    0x0, .invalid_val =    0x0},  //整车瞬时功率计算  
};


uint8_t can_msg_0x209_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_BCU7[sig], sig_val);
}

void can_msg_0x209_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCU7_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_BCU7[i]);
    }
}

void can_msg_0x209_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

    //app_car_warn_proc_BCU7();
}

