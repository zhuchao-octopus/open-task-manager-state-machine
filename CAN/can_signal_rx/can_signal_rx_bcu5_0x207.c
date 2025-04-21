

#include "can_signal_rx_bcu5_0x207.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_BCU5[CAN_SIG_RX_BCU5_COUNT] = {
    [BCU5_BCUMILReq]       = {.id = 0x207, .message_name = CAN_MSG_RX_BCU5, .byte_pos = 6, .bit_pos = 53, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0},  //电池系统故障请求  
};


uint8_t can_msg_0x207_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_BCU5[sig], sig_val);
}

void can_msg_0x207_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCU5_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_BCU5[i]);
    }
}

void can_msg_0x207_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}

