
#include "can_signal_rx_pke2_0x200.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"

#define CAN_MSG_RX_PKE2 500

const CAN_signal_config CAN_signal_rx_PKE2[CAN_SIG_RX_PKE2_COUNT] = {
    [PKE2_key]                   = {.id = 0x200, .message_name = CAN_MSG_RX_PKE2, .byte_pos = 0, .bit_pos =  1, .sig_len =  2, .default_val = 0x0, .invalid_val = 0x0},//钥匙信号
    ///[PKE2_SearchCarSts]          = {.id = 0x200, .message_name = CAN_MSG_RX_PKE2, .byte_pos = 0, .bit_pos =  3, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},//寻车功能反馈
    ///[PKE2_RKE_BAT_LOW]           = {.id = 0x200, .message_name = CAN_MSG_RX_PKE2, .byte_pos = 0, .bit_pos =  5, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},//钥匙低电量报警
    ///[PKE2_Front_Break]           = {.id = 0x200, .message_name = CAN_MSG_RX_PKE2, .byte_pos = 0, .bit_pos =  7, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},//刹车信号
    ///[PKE2_PduLvl]                = {.id = 0x200, .message_name = CAN_MSG_RX_PKE2, .byte_pos = 1, .bit_pos =  8, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},//电源档位
    ///[PKE2_BodyGuardAgainstTheft] = {.id = 0x200, .message_name = CAN_MSG_RX_PKE2, .byte_pos = 1, .bit_pos = 10, .sig_len =  2, .default_val = 0x0, .invalid_val = 0x0},//防盗状态
    ///[PKE2_One_Start_SwSts]       = {.id = 0x200, .message_name = CAN_MSG_RX_PKE2, .byte_pos = 1, .bit_pos = 12, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},//ONE_START开关
    ///[PKE2_BatteryVlotage]        = {.id = 0x200, .message_name = CAN_MSG_RX_PKE2, .byte_pos = 3, .bit_pos = 24, .sig_len = 16, .default_val = 0x0, .invalid_val = 0x0},//电池电压 
};


uint8_t can_msg_0x200_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_PKE2[sig], sig_val);
}

void can_msg_0x200_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_PKE2_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_PKE2[i]);
    }
}

void can_msg_0x200_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));
}
