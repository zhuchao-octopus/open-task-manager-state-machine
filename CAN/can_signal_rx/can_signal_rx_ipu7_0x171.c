
#include "can_signal_rx_ipu7_0x171.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_IPU7[CAN_SIG_RX_IPU7_COUNT] = {
    [IPU7_IPUBrakeFlag]             = {.id = 0x171, .message_name = CAN_MSG_RX_IPU7, .byte_pos = 0, .bit_pos =  1, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //脚刹
    [IPU7_IPUVehicleInDriveFlag]    = {.id = 0x171, .message_name = CAN_MSG_RX_IPU7, .byte_pos = 0, .bit_pos =  2, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //D挡
    [IPU7_IPUVehicleInRearlag]      = {.id = 0x171, .message_name = CAN_MSG_RX_IPU7, .byte_pos = 0, .bit_pos =  3, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //R挡
    [IPU7_IPUVehicleInNeutralFlag]  = {.id = 0x171, .message_name = CAN_MSG_RX_IPU7, .byte_pos = 0, .bit_pos =  4, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //N挡
    [IPU7_IPUH_Gear]                = {.id = 0x171, .message_name = CAN_MSG_RX_IPU7, .byte_pos = 0, .bit_pos =  5, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //H挡：标准模式
    [IPU7_IPUL_Gear]                = {.id = 0x171, .message_name = CAN_MSG_RX_IPU7, .byte_pos = 0, .bit_pos =  6, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //L挡：运动模式
    [IPU7_IPUM_Gear]                = {.id = 0x171, .message_name = CAN_MSG_RX_IPU7, .byte_pos = 0, .bit_pos =  7, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //M挡：经济模式
    [IPU7_IPUAccPeadal]             = {.id = 0x171, .message_name = CAN_MSG_RX_IPU7, .byte_pos = 1, .bit_pos =  8, .sig_len =  8, .default_val =    0x0, .invalid_val =    0x0},  //油门开度
    
       
};


uint8_t can_msg_0x171_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_IPU7[sig], sig_val);
}

void can_msg_0x171_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_IPU7_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_IPU7[i]);
    }
}

void can_msg_0x171_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}


