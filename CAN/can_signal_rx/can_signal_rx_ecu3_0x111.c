
#include "can_signal_rx_ecu3_0x111.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_ECU3[CAN_SIG_RX_ECU3_COUNT] = {
    [ECU3_ECU_BatteryVoltage]        = {.id = 0x111, .message_name = CAN_MSG_RX_ECU3, .byte_pos = 1, .bit_pos =  8, .sig_len =  8, .default_val = 0x0, .invalid_val = 0x0},//电池电压 
    [ECU3_Fault_Coad]                = {.id = 0x111, .message_name = CAN_MSG_RX_ECU3, .byte_pos = 6, .bit_pos = 48, .sig_len = 16, .default_val = 0x0, .invalid_val = 0x0},//故障代码
};


uint8_t can_msg_0x111_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_ECU3[sig], sig_val);
}

void can_msg_0x111_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_ECU3_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_ECU3[i]);
    }
}

void can_msg_0x111_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

    //app_car_warn_proc_ECU3();
}
