

#include "can_signal_rx_bcu3_0x20B.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_BCU3[CAN_SIG_RX_BCU3_COUNT] = {
    [BCU3_BCUMaxDchaPwrLongT]          = {.id = 0x203, .message_name = CAN_MSG_RX_BCU3, .byte_pos = 1, .bit_pos = 14, .sig_len = 10, .default_val =   0x0, .invalid_val =  0x3FF},  //电池最大充电功率(长时)
    [BCU3_BCUMaxChrgPwrLongT]          = {.id = 0x203, .message_name = CAN_MSG_RX_BCU3, .byte_pos = 2, .bit_pos = 20, .sig_len = 10, .default_val =   0x0, .invalid_val =  0x3FF},  //电池最大充电功率(长时)
    [BCU3_BCUMaxDchaILongT]            = {.id = 0x203, .message_name = CAN_MSG_RX_BCU3, .byte_pos = 3, .bit_pos = 26, .sig_len = 10, .default_val =   0x0, .invalid_val =  0x3FF},  //电池最大充电功率(长时)
    [BCU3_BCUMaxChrgILongT]            = {.id = 0x203, .message_name = CAN_MSG_RX_BCU3, .byte_pos = 4, .bit_pos = 32, .sig_len = 10, .default_val =   0x0, .invalid_val =  0x3FF},  //电池最大充电功率(长时)
    [BCU3_BCU235CycCntr]               = {.id = 0x203, .message_name = CAN_MSG_RX_BCU3, .byte_pos = 6, .bit_pos = 48, .sig_len =  4, .default_val =   0x0, .invalid_val =    0xF},  //本帧报文计数
    [BCU3_BCUMinDchaIU]                = {.id = 0x203, .message_name = CAN_MSG_RX_BCU3, .byte_pos = 6, .bit_pos = 55, .sig_len =  9, .default_val =   0x0, .invalid_val =  0x1FF},  //电池最小放电电压
    [BCU3_BCUCRCChk235]                = {.id = 0x203, .message_name = CAN_MSG_RX_BCU3, .byte_pos = 7, .bit_pos = 56, .sig_len =  8, .default_val =   0x0, .invalid_val =  0x1FF},  //校验
    
};


uint8_t can_msg_0x20B_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_BCU3[sig], sig_val);
}

void can_msg_0x20B_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCU3_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_BCU3[i]);
    }
}

void can_msg_0x20B_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}

