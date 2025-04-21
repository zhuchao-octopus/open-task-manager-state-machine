
#include "can_signal_rx_sl1_0x188.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_SL1[CAN_SIG_RX_SL1_COUNT] = {
    [SL1_SL_Reserve1]       = {.id = 0x188, .message_name = CAN_MSG_RX_SL1, .byte_pos = 2, .bit_pos = 16, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //预留
    [SL1_SL_Reserve2]       = {.id = 0x188, .message_name = CAN_MSG_RX_SL1, .byte_pos = 2, .bit_pos = 17, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //预留
    [SL1_SL_Reserve3]       = {.id = 0x188, .message_name = CAN_MSG_RX_SL1, .byte_pos = 2, .bit_pos = 18, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //预留
    [SL1_SL_Reserve4]       = {.id = 0x188, .message_name = CAN_MSG_RX_SL1, .byte_pos = 2, .bit_pos = 19, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //预留
    [SL1_SL_Reserve5]       = {.id = 0x188, .message_name = CAN_MSG_RX_SL1, .byte_pos = 2, .bit_pos = 20, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //预留
    [SL1_SL_Reserve6]       = {.id = 0x188, .message_name = CAN_MSG_RX_SL1, .byte_pos = 2, .bit_pos = 21, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //预留
    [SL1_SL_Reserve7]       = {.id = 0x188, .message_name = CAN_MSG_RX_SL1, .byte_pos = 2, .bit_pos = 22, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //预留
    [SL1_SL_Reserve8]       = {.id = 0x188, .message_name = CAN_MSG_RX_SL1, .byte_pos = 2, .bit_pos = 23, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //预留
};


uint8_t can_msg_0x188_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_SL1[sig], sig_val);
}

void can_msg_0x188_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_SL1_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_SL1[i]);
    }
}

void can_msg_0x188_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}



