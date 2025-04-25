
#include "can_signal_rx_ipu6_0x202.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_IPU6[CAN_SIG_RX_IPU6_COUNT] = {
    [IPU6_IPUVehSpdVld]     = {.id = 0x202, .message_name = CAN_MSG_RX_IPU6, .byte_pos = 5, .bit_pos = 44, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //车速有效状态
    [IPU6_IPUVehSpd]        = {.id = 0x202, .message_name = CAN_MSG_RX_IPU6, .byte_pos = 6, .bit_pos = 52, .sig_len =  8, .default_val =    0x0, .invalid_val =   0xFF},  //车速：MCU以转速计算

    
};


uint8_t can_msg_0x202_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_IPU6[sig], sig_val);
}

void can_msg_0x202_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_IPU6_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_IPU6[i]);
    }
}

void can_msg_0x202_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}


