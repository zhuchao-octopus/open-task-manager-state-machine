
#include "can_signal_rx_bcm5_0x195.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_BCM5[CAN_SIG_RX_BCM5_COUNT] = {
    [BCM5_SecondDrive]                = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 0, .bit_pos =  7, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //二驱
    [BCM5_HandBrakeFlag]              = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 1, .bit_pos =  9, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //手刹
    [BCM5_SeatBelt_FlashingAlarm]     = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 1, .bit_pos = 10, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //安全带闪烁报警
    [BCM5_BrakeFluid_AlarmSwitch]     = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 1, .bit_pos = 11, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //制动液报警开关
    [BCM5_DriversSeatBelt_Switch]     = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 1, .bit_pos = 12, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //主驾驶安全带开关
    [BCM5_ElsSwitch]                  = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 1, .bit_pos = 14, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //应急灯开关
    [BCM5_ParkingAlarm]               = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 1, .bit_pos = 15, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //驻车行驶报警
    [BCM5_Reserve_1]                  = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 2, .bit_pos = 19, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //预留1
    [BCM5_Reserve_2]                  = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 2, .bit_pos = 20, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //预留2
    [BCM5_Reserve_3]                  = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 2, .bit_pos = 21, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //预留3
    [BCM5_Reserve_4]                  = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 2, .bit_pos = 22, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //预留4
    [BCM5_Reserve_5]                  = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 2, .bit_pos = 23, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //预留5
    [BCM5_PositionLamp]               = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 3, .bit_pos = 24, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //位置灯
    [BCM5_LowBeam]                    = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 3, .bit_pos = 25, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //近光灯
    [BCM5_HighBeam]                   = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 3, .bit_pos = 26, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //远光灯
    [BCM5_FourWD_FourWheelDrive]      = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 3, .bit_pos = 27, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //四驱
    [BCM5_FrontAxle_DifferentialLock] = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 3, .bit_pos = 28, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //前桥差速锁
    [BCM5_LeftTurnLamp]               = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 3, .bit_pos = 29, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //左转向灯
    [BCM5_RightTurnLamp]              = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 3, .bit_pos = 30, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //右转向灯
    [BCM5_RearAxle_DifferentialLock]  = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 3, .bit_pos = 31, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //后桥差速锁
    [BCM5_Coolant_temp]               = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 5, .bit_pos = 40, .sig_len = 8, .default_val =0x46, .invalid_val =0xFF}, //冷却液温度
    [BCM5_Cool_pumpsd]                = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 6, .bit_pos = 48, .sig_len = 8, .default_val = 0x0, .invalid_val =0xFF}, //冷却水泵转速
    [BCM5_Cool_Fan]                   = {.id = 0x195, .message_name = CAN_MSG_RX_BCM5, .byte_pos = 7, .bit_pos = 56, .sig_len = 1, .default_val = 0x0, .invalid_val = 0x0}, //冷却风扇

};


uint8_t can_msg_0x195_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_BCM5[sig], sig_val);
}

void can_msg_0x195_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCM5_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_BCM5[i]);
    }
}

void can_msg_0x195_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));
}
