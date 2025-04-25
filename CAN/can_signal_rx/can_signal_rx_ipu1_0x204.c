

#include "can_signal_rx_ipu1_0x204.h"


#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


const CAN_signal_config CAN_signal_rx_IPU1[CAN_SIG_RX_IPU1_COUNT] = {
    //[IPU1_FrntIpuIDc]          = {.id = 0x204, .message_name = CAN_MSG_RX_IPU1, .byte_pos = 1, .bit_pos = 10, .sig_len = 14, .default_val = 0x1F40, .invalid_val = 0x3FFF},  //电机控制器直流侧电流
    //[IPU1_FrntIpuUDcToRmu]     = {.id = 0x204, .message_name = CAN_MSG_RX_IPU1, .byte_pos = 3, .bit_pos = 28, .sig_len = 14, .default_val =    0x0, .invalid_val = 0x3FFF},  //电机控制器直流侧电压
    [IPU1_IPUVehRange]         = {.id = 0x204, .message_name = CAN_MSG_RX_IPU1, .byte_pos = 4, .bit_pos = 32, .sig_len = 12, .default_val =    0x0, .invalid_val =    0x0},  //续航里程
    [IPU1_IPUInsttPwrCspt]     = {.id = 0x204, .message_name = CAN_MSG_RX_IPU1, .byte_pos = 6, .bit_pos = 54, .sig_len = 10, .default_val =    0x0, .invalid_val =    0x0},  //瞬时耗电量(kWh/100Km)
    [IPU1_MotSysPwrLmtLp]      = {.id = 0x204, .message_name = CAN_MSG_RX_IPU1, .byte_pos = 7, .bit_pos = 58, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //电机系统限功率指示灯
    [IPU1_IPUReadyLp]          = {.id = 0x204, .message_name = CAN_MSG_RX_IPU1, .byte_pos = 7, .bit_pos = 59, .sig_len =  1, .default_val =    0x0, .invalid_val =    0x0},  //Ready信号指示灯
    
};


uint8_t can_msg_0x204_get_data(uint32_t sig, uint32_t *sig_val){
    return can_function_read_signal_value_callback(CAN_signal_rx_IPU1[sig], sig_val);
}

void can_msg_0x204_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_IPU1_COUNT; i++) {
        can_function_timeout_reset_candata_callback(CAN_signal_rx_IPU1[i]);
    }
}

void can_msg_0x204_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));

    //int16_t sum = 0;
    
    if(timeout == 0)
    {
        ///CarInfo* info = data_carinfo_get_carinfo();
        
        uint32_t sig_val = 0;
        can_function_read_signal_value(CAN_MSG_RX_IPU1, IPU1_IPUInsttPwrCspt, &sig_val);
        
        ///info->pwr_inject_sum += ((int64_t)sig_val - 500);
        ///info->pwr_inject_count++; 

        //sum = info->pwr_inject_sum;
        //PRINTF("SUM__011 = %d \r\n",sum);
        //PRINTF("COUNT__022 = %d \r\n",info->pwr_inject_count);
        
    }
}

