

#include "can_signal_rx_bcu1_0x185.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_BCU1[CAN_SIG_RX_BCU1_COUNT] = {
    [BCU1_BCUBattU]              = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 1, .bit_pos = 11, .sig_len = 13, .default_val =    0x0, .invalid_val = 0x1FFF},  //电池总电压
    [BCU1_BCUBattI]              = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 3, .bit_pos = 29, .sig_len = 14, .default_val = 0x1770, .invalid_val = 0x3FFF},  //电池总电流
    //[BCU1_BCUInsulationSts]      = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 4, .bit_pos = 32, .sig_len =  2, .default_val =    0x0, .invalid_val =    0x0},  //绝缘状态
    //[BCU1_BcuHeatSftySts]        = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 4, .bit_pos = 34, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //电池热失控状态
    [BCM1_BCUSOC]                = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 4, .bit_pos = 35, .sig_len = 10, .default_val =    0x0, .invalid_val =  0x3FF},  //电池荷电量SOC
    //[BCU1_BCUHVILSts]            = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 5, .bit_pos = 41, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //高压互锁状态
    //[BCU1_BCUPwrUpAllw]          = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 5, .bit_pos = 42, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //允许重新上高压电
    //[BCU1_BCUOBCOperModReq]      = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 5, .bit_pos = 43, .sig_len =  2, .default_val =    0x0, .invalid_val =    0x0},  //OBC工作模式请求
    //[BCU1_BCUOperMod]            = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 5, .bit_pos = 45, .sig_len =  3, .default_val =    0x0, .invalid_val =    0x0},  //BMS工作模式
    //[BCU1_BCU185CycCntr]         = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 6, .bit_pos = 48, .sig_len =  4, .default_val =    0x0, .invalid_val = 0x1FFF},  //本帧报文计数
    //[BCU1_BCUFltRnk]             = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 6, .bit_pos = 52, .sig_len =  4, .default_val =    0x0, .invalid_val =    0x0},  //电池故障等级
    //[BCU1_BCUCRCChk185]          = {.id = 0x185, .message_name = CAN_MSG_RX_BCU1, .byte_pos = 7, .bit_pos = 56, .sig_len =  8, .default_val =    0x0, .invalid_val =    0x0},  //校验
    
};


uint8_t can_msg_0x185_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_BCU1[sig], sig_val);
}

void can_msg_0x185_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCU1_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_BCU1[i]);
    }
}

void can_msg_0x185_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}

