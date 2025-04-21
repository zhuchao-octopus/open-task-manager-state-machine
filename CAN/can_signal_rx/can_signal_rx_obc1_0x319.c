

#include "can_signal_rx_obc1_0x319.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"



const CAN_signal_config CAN_signal_rx_OBC1[CAN_SIG_RX_OBC1_COUNT] = {
    [OBC1_ObcCpValVld]       = {.id = 0x319, .message_name = CAN_MSG_RX_OBC1, .byte_pos = 5, .bit_pos = 46, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //CP信号状态
    [OBC1_ObcChrgSts]        = {.id = 0x319, .message_name = CAN_MSG_RX_OBC1, .byte_pos = 6, .bit_pos = 50, .sig_len =  4, .default_val = 0x0, .invalid_val = 0xF},  //充电机工作状态
    
   
};


uint8_t can_msg_0x319_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_OBC1[sig], sig_val);
}

void can_msg_0x319_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_OBC1_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_OBC1[i]);
    }
}

void can_msg_0x319_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}

