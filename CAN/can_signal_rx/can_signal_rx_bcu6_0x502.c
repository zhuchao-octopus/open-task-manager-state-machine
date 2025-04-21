

#include "can_signal_rx_bcu6_0x502.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_BCU6[CAN_SIG_RX_BCU6_COUNT] = {
    [BCU6_BCUCellTOver]        = {.id = 0x502, .message_name = CAN_MSG_RX_BCU6, .byte_pos = 0, .bit_pos =  1, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //电池单体温度过高
    [BCU6_BCUBattUOver]        = {.id = 0x502, .message_name = CAN_MSG_RX_BCU6, .byte_pos = 0, .bit_pos =  3, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //动力蓄电池包过压报警
    [BCU6_BCUBattUnder]        = {.id = 0x502, .message_name = CAN_MSG_RX_BCU6, .byte_pos = 0, .bit_pos =  4, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //动力蓄电池包欠压报警
    [BCU6_BCUBattSOCUnder]     = {.id = 0x502, .message_name = CAN_MSG_RX_BCU6, .byte_pos = 1, .bit_pos = 12, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //SOC低报警
    [BCU6_BCUBattSOCHi]        = {.id = 0x502, .message_name = CAN_MSG_RX_BCU6, .byte_pos = 1, .bit_pos = 13, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //SOC太高报警
    [BCU6_BCUBattSOCLo]        = {.id = 0x502, .message_name = CAN_MSG_RX_BCU6, .byte_pos = 1, .bit_pos = 14, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //SOC太低报警
    [BCU6_BCUBattSOCJump]      = {.id = 0x502, .message_name = CAN_MSG_RX_BCU6, .byte_pos = 2, .bit_pos = 20, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //SOC跳变故障
    [BCU6_BCUBattChgOverWarn]  = {.id = 0x502, .message_name = CAN_MSG_RX_BCU6, .byte_pos = 2, .bit_pos = 22, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //动力电池过充报警
    
};


uint8_t can_msg_0x502_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_BCU6[sig], sig_val);
}

void can_msg_0x502_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCU6_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_BCU6[i]);
    }
}

void can_msg_0x502_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}

