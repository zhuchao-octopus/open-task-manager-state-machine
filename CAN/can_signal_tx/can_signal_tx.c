
/*****************************************************************************/
#include "can_signal_tx.h"



CAN_signal_config CAN_signal_tx_info[CAN_TX_SIG_count] = {
    //0x309
    [ICU_IC_Status_NM]                  = {.id = 0x309, .byte_pos = 0, .bit_pos =  0,  .sig_len = 4, .default_val = 0x0, .invalid_val = 0x0},     //网络管理状态   预留
    //0x310
    [ICU1_IcuTotMilg]                   = {.id = 0x310, .byte_pos = 0, .bit_pos =  0,  .sig_len = 24, .default_val = 0x0, .invalid_val = 0x0},    //总里程
    [ICU1_IcuTotDriHrs]                 = {.id = 0x310, .byte_pos = 3, .bit_pos = 24,  .sig_len = 24, .default_val = 0x0, .invalid_val = 0x0},    //发动机工作总小时数
    [ICU1_IcuReq]                       = {.id = 0x310, .byte_pos = 6, .bit_pos = 48,  .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},    //总计里程、发动机总工作小时数请求
    [ICU1_StorgErrOrIcuErr]             = {.id = 0x310, .byte_pos = 6, .bit_pos = 49,  .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},    //BCM里程存储值故障或仪表故障
    [ICU1_Environment_Temp]             = {.id = 0x310, .byte_pos = 7, .bit_pos = 56,  .sig_len =  8, .default_val = 0x0, .invalid_val = 0x0},    //环境温度
    //0x316
    [ICU2_TripMileage]                  = {.id = 0x316, .byte_pos = 0, .bit_pos =  0,  .sig_len = 16, .default_val = 0x0, .invalid_val = 0x0},    //小计里程
    [ICU2_IcuPwrConsptPer100kilomt]     = {.id = 0x316, .byte_pos = 2, .bit_pos = 16,  .sig_len = 10, .default_val = 0x0, .invalid_val = 0x0},    //百公里耗电量(kWh/100Km)
    [ICU2_TripTime]                     = {.id = 0x316, .byte_pos = 6, .bit_pos = 48,  .sig_len = 16, .default_val = 0x0, .invalid_val = 0x0},    //小计时间
    
};
