
#include "can_signal_rx_bcm3_0x172.h"
#include "../can_function.h"
#include "../can_type.h"
#include "../can_message_rx.h"

const CAN_signal_config CAN_signal_rx_BCM3[CAN_SIG_RX_BCM3_COUNT] = {
    [BCM3_BcmTotMilg]       = {.id = 0x172, .message_name = CAN_MSG_RX_BCM3, .byte_pos = 0, .bit_pos =  0, .sig_len = 24, .default_val = 0x0, .invalid_val = 0x0},  //总计里程储存值
    [BCM3_BcmTotDriHrs]     = {.id = 0x172, .message_name = CAN_MSG_RX_BCM3, .byte_pos = 3, .bit_pos = 24, .sig_len = 24, .default_val = 0x0, .invalid_val = 0x0},  //行驶总小时数储存值
    [BCM3_BcmAllwUp]        = {.id = 0x172, .message_name = CAN_MSG_RX_BCM3, .byte_pos = 6, .bit_pos = 48, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //BCM允许ICU更新总计里程、行驶总小时数
};


uint8_t can_msg_0x172_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_BCM3[sig], sig_val);
}

void can_msg_0x172_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCM3_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_BCM3[i]);
    }
}

void can_msg_0x172_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));
}
