

#include "can_signal_rx_obc2_0x349.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"



const CAN_signal_config CAN_signal_rx_OBC2[CAN_SIG_RX_OBC2_COUNT] = {
    [OBC2_ObcTOverWarn]             = {.id = 0x349, .message_name = CAN_MSG_RX_OBC2, .byte_pos = 7, .bit_pos = 60, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //OBC过温报警
    [OBC2_ObcStopChrgAcUnderWarn]   = {.id = 0x349, .message_name = CAN_MSG_RX_OBC2, .byte_pos = 7, .bit_pos = 61, .sig_len =  1, .default_val = 0x0, .invalid_val = 0x0},  //交流电压过低停止充电报警
    [OBC2_ObcErrNr]                 = {.id = 0x349, .message_name = CAN_MSG_RX_OBC2, .byte_pos = 7, .bit_pos = 62, .sig_len =  8, .default_val = 0x0, .invalid_val = 0x0},  //充电机故障码
    
   
};


uint8_t can_msg_0x349_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_OBC2[sig], sig_val);
}

void can_msg_0x349_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_OBC2_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_OBC2[i]);
    }
}

void can_msg_0x349_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

    //app_car_warn_proc_OBC2();
}

