

#include "can_signal_rx_msc_0x180.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_MSC[CAN_SIG_RX_MSC_COUNT] = {

    [MSC_CurrentG]          = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 1, .bit_pos =  8, .sig_len =  3, .default_val = 0x0, .invalid_val = 0x0},  //当前档位信号
    [MSC_CurrentG_flash]    = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 1, .bit_pos = 11, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //当前档位闪烁信号
    [MSC_angleSensorFlt]    = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 2, .bit_pos = 16, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //MSC角度传感器损坏
    [MSC_gearSwitchFlt]     = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 2, .bit_pos = 17, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //MSC档位开关工作
    [MSC_motorOpenFlt]      = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 2, .bit_pos = 18, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //MSC电机开路
    [MSC_motorShortFlt]     = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 2, .bit_pos = 19, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //MSC电机短路
    [MSC_OLgearShiftFlt]    = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 2, .bit_pos = 20, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //MSC重载换挡失败
    [MSC_LLgearShiftFlt]    = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 2, .bit_pos = 21, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //MSC轻载换挡失败
    [MSC_self_learningFlt]  = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 2, .bit_pos = 22, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //MSC自学习失败
    [MSC_reserved]          = {.id = 0x180, .message_name = CAN_MSG_RX_MSC, .byte_pos = 2, .bit_pos = 23, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //预留
};


uint8_t can_msg_0x180_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_MSC[sig], sig_val);
}

void can_msg_0x180_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_MSC_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_MSC[i]);
    }
}

void can_msg_0x180_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

    //app_car_warn_proc_BCM2();    
}


