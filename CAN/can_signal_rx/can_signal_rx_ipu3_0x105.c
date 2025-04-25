

#include "can_signal_rx_ipu3_0x105.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_IPU3[CAN_SIG_RX_IPU3_COUNT] = {
    [IPU3_FrntMotTOver]       = {.id = 0x105, .message_name = CAN_MSG_RX_IPU3, .byte_pos = 4, .bit_pos = 39, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //电机温度过高
    [IPU3_IPUMTPower]         = {.id = 0x105, .message_name = CAN_MSG_RX_IPU3, .byte_pos = 6, .bit_pos = 48, .sig_len = 12, .default_val = 0x0, .invalid_val = 0x0},  //电机瞬时功率
   
};


uint8_t can_msg_0x105_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_IPU3[sig], sig_val);
}

void can_msg_0x105_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_IPU3_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_IPU3[i]);
    }
}

void can_msg_0x105_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

}

