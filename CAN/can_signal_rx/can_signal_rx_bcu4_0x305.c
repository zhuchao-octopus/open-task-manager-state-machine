

#include "can_signal_rx_bcu4_0x305.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_BCU4[CAN_SIG_RX_BCU4_COUNT] = {
    //[BCU4_BCUChrgUReq]       = {.id = 0x305, .message_name = CAN_MSG_RX_BCU4, .byte_pos = 1, .bit_pos =  8, .sig_len = 16, .default_val =    0x0, .invalid_val = 0xFFFF},  //外插充电需求电压
    //[BCU4_BcuAcChrgIReq]     = {.id = 0x305, .message_name = CAN_MSG_RX_BCU4, .byte_pos = 3, .bit_pos = 29, .sig_len = 11, .default_val =    0x0, .invalid_val =  0x7FF},  //外插交流充电需求电流
    [BCU4_BCUChrgSts]        = {.id = 0x305, .message_name = CAN_MSG_RX_BCU4, .byte_pos = 6, .bit_pos = 52, .sig_len =  4, .default_val =    0x0, .invalid_val =    0x0},  //电池充电状态
    //[BCU4_BCUChrgErrInfo]    = {.id = 0x305  .message_name = CAN_MSG_RX_BCU4, .byte_pos = 5, .bit_pos = 43, .sig_len =  4, .default_val =    0x0, .invalid_val =    0x0},  //BCU充电过程故障监控
    //[BCU4_BCU305CycCntr]     = {.id = 0x305, .message_name = CAN_MSG_RX_BCU4, .byte_pos = 6, .bit_pos = 48, .sig_len =  4, .default_val =    0x0, .invalid_val =    0xF},  //本帧报文计数
    //[BCU4_BCUChrgMod]        = {.id = 0x305, .message_name = CAN_MSG_RX_BCU4, .byte_pos = 6, .bit_pos = 52, .sig_len =  4, .default_val =    0x0, .invalid_val =    0x0},  //电池充电模式请求
    //[BCU4_BCUCRCChk305]      = {.id = 0x305, .message_name = CAN_MSG_RX_BCU4, .byte_pos = 7, .bit_pos = 56, .sig_len =  8, .default_val =    0x0, .invalid_val =    0x0},  //校验
    
};


uint8_t can_msg_0x305_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_BCU4[sig], sig_val);
}

void can_msg_0x305_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCU4_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_BCU4[i]);
    }
}

void can_msg_0x305_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}

