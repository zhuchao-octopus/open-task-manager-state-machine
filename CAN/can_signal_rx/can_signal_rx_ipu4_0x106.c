

#include "can_signal_rx_ipu4_0x106.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_IPU4[CAN_SIG_RX_IPU4_COUNT] = {
    [IPU4_FrntIpuErrNr]       = {.id = 0x106, .message_name = CAN_MSG_RX_IPU4, .byte_pos = 1, .bit_pos = 12, .sig_len =  8, .default_val = 0x0, .invalid_val = 0xFF},  //电机温度过高
    
   
};


uint8_t can_msg_0x106_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_IPU4[sig], sig_val);
}

void can_msg_0x106_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_IPU4_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_IPU4[i]);
    }
}

void can_msg_0x106_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

    //app_car_warn_proc_IPU4();
}

