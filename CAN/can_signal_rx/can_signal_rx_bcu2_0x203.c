

#include "can_signal_rx_bcu2_0x203.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"



const CAN_signal_config CAN_signal_rx_BCU2[CAN_SIG_RX_BCU2_COUNT] = {
    [BCU2_BCUMaxDchaPwrShoT]          = {.id = 0x203, .message_name = CAN_MSG_RX_BCU2, .byte_pos = 1, .bit_pos = 14, .sig_len = 10, .default_val =   0x0, .invalid_val =  0x3FF},  //电池最大充电功率(短时)
    [BCU2_BCUMaxChrgPwrShoT]          = {.id = 0x203, .message_name = CAN_MSG_RX_BCU2, .byte_pos = 2, .bit_pos = 20, .sig_len = 10, .default_val =   0x0, .invalid_val =  0x3FF},  //电池最大充电功率(短时)
    [BCU2_BCUMaxDchaIShoT]            = {.id = 0x203, .message_name = CAN_MSG_RX_BCU2, .byte_pos = 3, .bit_pos = 26, .sig_len = 10, .default_val =   0x0, .invalid_val =  0x3FF},  //电池最大充电功率(短时)
    [BCU2_BCUMaxChrgIShoT]            = {.id = 0x203, .message_name = CAN_MSG_RX_BCU2, .byte_pos = 4, .bit_pos = 32, .sig_len = 10, .default_val =   0x0, .invalid_val =  0x3FF},  //电池最大充电功率(短时)
    [BCU2_BCU215CycCntr]              = {.id = 0x203, .message_name = CAN_MSG_RX_BCU2, .byte_pos = 4, .bit_pos = 35, .sig_len = 10, .default_val =   0x0, .invalid_val =    0xF},  //本帧报文计数
    [BCU2_BcuRlyWlddErr]              = {.id = 0x203, .message_name = CAN_MSG_RX_BCU2, .byte_pos = 6, .bit_pos = 52, .sig_len =  1, .default_val =   0x0, .invalid_val =    0x0},  //主继电器粘连故障
};


uint8_t can_msg_0x201_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_BCU2[sig], sig_val);
}

void can_msg_0x201_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCU2_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_BCU2[i]);
    }
}

void can_msg_0x201_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}



