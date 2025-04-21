
#include "can_signal_rx_bcm2_0x170.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"

const CAN_signal_config CAN_signal_rx_BCM2[CAN_SIG_RX_BCM2_COUNT] = {

    [BCM2_BCMFault_Coad] = {.id = 0x170, .message_name = CAN_MSG_RX_BCM2, .byte_pos = 7, .bit_pos = 56, .sig_len =  8, .default_val = 0x0, .invalid_val = 0x0},  //故障码
};


uint8_t can_msg_0x170_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_BCM2[sig], sig_val);
}

void can_msg_0x170_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_BCM2_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_BCM2[i]);
    }
}

void can_msg_0x170_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

    //app_car_warn_proc_BCM2();    
}
