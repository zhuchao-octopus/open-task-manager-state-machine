
#include "can_signal_rx_ccs1_0x312.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"

const CAN_signal_config CAN_signal_rx_CCS1[CAN_SIG_RX_CCS1_COUNT] = {
    [CCS1_IC_BrightnessLevel]      = {.id = 0x312, .message_name = CAN_MSG_RX_CCS1, .byte_pos = 0, .bit_pos =  0, .sig_len = 4, .default_val = 0x0, .invalid_val = 0x0},//仪表亮度等级
    [CCS1_SubtotalMileageClearing] = {.id = 0x312, .message_name = CAN_MSG_RX_CCS1, .byte_pos = 0, .bit_pos =  4, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0},//小计里程清零
    [CCS1_SubtotalTimeClearing]    = {.id = 0x312, .message_name = CAN_MSG_RX_CCS1, .byte_pos = 0, .bit_pos =  5, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0},//小计时间清零
    [CCS1_MileageUnitSwitching]    = {.id = 0x312, .message_name = CAN_MSG_RX_CCS1, .byte_pos = 0, .bit_pos =  6, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0},//里程单位切换
    [CCS1_LanguageSwitching]       = {.id = 0x312, .message_name = CAN_MSG_RX_CCS1, .byte_pos = 0, .bit_pos =  7, .sig_len = 4, .default_val = 0x0, .invalid_val = 0x0},//语言显示切换
    [CCS1_TempUnitSwitching]       = {.id = 0x312, .message_name = CAN_MSG_RX_CCS1, .byte_pos = 1, .bit_pos = 11, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0},//温度单位切换
    //[IVI2_PressureUnitSwitching]   = {.id = 0x312, .message_name = CAN_MSG_RX_CCS1, .byte_pos = 1, .bit_pos = 12, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0},//压力单位切换
    //[IVI2_VolumeUnitSwitching]     = {.id = 0x312, .message_name = CAN_MSG_RX_CCS1, .byte_pos = 1, .bit_pos = 13, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0},//体积单位切换
};



uint8_t can_msg_0x312_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_CCS1[sig], sig_val);
}

void can_msg_0x312_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_CCS1_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_CCS1[i]);
    }
}

void can_msg_0x312_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));
    
    if(timeout == 0)
    {
        uint32_t sig_val = 0;
        can_msg_0x312_get_data(CCS1_SubtotalMileageClearing, &sig_val);
        if(sig_val)
        {
            //sys_app_mark_set_clear_trip();
        }
        
        can_msg_0x312_get_data(CCS1_SubtotalTimeClearing, &sig_val);
        if(sig_val)
        {
            //sys_app_mark_set_clear_trip_time();
        }
    }

}
