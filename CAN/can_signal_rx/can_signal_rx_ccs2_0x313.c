
#include "can_signal_rx_ccs2_0x313.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_CCS2[CAN_SIG_RX_CCS2_COUNT] = {
    [CCS2_CentralControlPanel_Hours]   = {.id = 0x313, .message_name = CAN_MSG_RX_CCS2, .byte_pos = 0, .bit_pos =  0, .sig_len =  5, .default_val = 0x0, .invalid_val = 0x0},//中控屏小时数
    [CCS2_CentralControlPanel_Minutes] = {.id = 0x313, .message_name = CAN_MSG_RX_CCS2, .byte_pos = 1, .bit_pos =  8, .sig_len =  6, .default_val = 0x0, .invalid_val = 0x0},//中控屏分钟数
    [CCS2_CentralControlPanel_Seconds] = {.id = 0x313, .message_name = CAN_MSG_RX_CCS2, .byte_pos = 2, .bit_pos = 16, .sig_len =  6, .default_val = 0x0, .invalid_val = 0x0},//中控屏秒钟数
    [CCS2_Age]                         = {.id = 0x313, .message_name = CAN_MSG_RX_CCS2, .byte_pos = 3, .bit_pos = 24, .sig_len = 16, .default_val = 0x0, .invalid_val = 0x0},//年份
    [CCS2_Month]                       = {.id = 0x313, .message_name = CAN_MSG_RX_CCS2, .byte_pos = 5, .bit_pos = 40, .sig_len =  4, .default_val = 0x0, .invalid_val = 0x0},//月份
    [CCS2_Date]                        = {.id = 0x313, .message_name = CAN_MSG_RX_CCS2, .byte_pos = 6, .bit_pos = 48, .sig_len =  5, .default_val = 0x0, .invalid_val = 0x0},//日期
    [CCS2_Week]                        = {.id = 0x313, .message_name = CAN_MSG_RX_CCS2, .byte_pos = 5, .bit_pos = 44, .sig_len =  4, .default_val = 0x0, .invalid_val = 0x0},//星期
};

uint8_t can_msg_0x313_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_CCS2[sig], sig_val);
}

void can_msg_0x313_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_CCS2_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_CCS2[i]);
    }
}

void can_msg_0x313_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));
}
