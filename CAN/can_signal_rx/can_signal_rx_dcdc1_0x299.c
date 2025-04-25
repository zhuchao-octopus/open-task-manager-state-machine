

#include "can_signal_rx_dcdc1_0x299.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"



const CAN_signal_config CAN_signal_rx_DCDC1[CAN_SIG_RX_DCDC1_COUNT] = {
    [DCDC1_DcdcErrNr]             = {.id = 0x299, .message_name = CAN_MSG_RX_DCDC1, .byte_pos = 1, .bit_pos =  8, .sig_len =  8, .default_val = 0x0, .invalid_val = 0x0},  //DCDC故障码
    
   
};


uint8_t can_msg_0x299_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_DCDC1[sig], sig_val);
}

void can_msg_0x299_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_DCDC1_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_DCDC1[i]);
    }
}

void can_msg_0x299_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

    //app_car_warn_proc_DCDC1();

}

