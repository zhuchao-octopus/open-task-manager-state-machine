
/*****************************************************************************/
#include "can_function.h"

#include "can_signal_rx/can_signal_rx.h"
#include "can_signal_tx/can_signal_tx.h"
#include "can_message_rx.h"
#include "can_message_tx.h"
#include "can_message_l.h"
#include "../octopus_tickcounter.h" 

#define CAN_WAIT_TIMER_TIMEOUT (10 * 1000)

static uint32_t l_t_can_wait_timer;

static void can_fuction_app_register(void);

void can_fuction_init(void) {
    can_fuction_load_data();
    can_ml_init();

    StartTickCounter(&l_t_can_wait_timer);
}

void can_fuction_load_data(){
    //CAN_msg_rx_config_case
    uint8_t idx = 0;
    for (idx = 0; idx < CAN_MSG_RX_NUM; idx++) {
        CAN_msg_rx_config_case[idx].is_data_update = 1;
    }
    can_fuction_app_register();
}

void can_fuction_deinit(void) {
    can_ml_deinit();
}

void can_fuction_loop_rt(void) {
    //接收CAN数据
    bool res = can_ml_receive();
    if(res)
    {
        StartTickCounter(&l_t_can_wait_timer);
    }
   //发送CAN数据
    can_ml_transmit();
}

void can_fuction_loop_2ms(void) {
}

void can_fuction_loop_10ms(void) {
    //超时管理函数
    can_ml_rx_timeout_manager();
}

void can_fuction_PowerOff(void) {

}

void can_fuction_PowerOn(void) {

}

bool can_fuction_isCanWKTimeout(void)
{
    if(GetTickCounter(&l_t_can_wait_timer) > CAN_WAIT_TIMER_TIMEOUT)
    {
        return true;
    }
    return false;
}

void can_fuction_tx_enable(bool enable)
{
    if(enable)
    {
        can_ml_set_user_tx_enable(0);
    }
    else
    {
        can_ml_set_user_tx_disable(0);
    }
}

uint8_t can_fuction_timeout_reset_candata_callback(CAN_signal_config config) {
    return can_ml_resetCanDataByCANID(config.id, config.invalid_val, config.byte_pos, config.bit_pos, config.sig_len);
}


uint8_t can_fuction_read_signal_value_callback(CAN_signal_config config, uint32_t *sig_val) {
    return can_ml_readCanDataByMessageName(config.message_name, sig_val, config.byte_pos, config.bit_pos, config.sig_len);
}

uint8_t can_fuction_read_signal_timeout_flag(CanReceiveID canReceiveID) {
    return CAN_msg_rx_config_case[canReceiveID].error_status;
}

void can_fuction_write_signal_value(uint8_t sig_id, uint32_t sig_val) {
    uint8_t byte_pos;
    uint8_t bit_pos;
    uint8_t sig_len;

    if (sig_id < CAN_TX_SIG_count) {
        byte_pos = CAN_signal_tx_info[sig_id].byte_pos;
        bit_pos = CAN_signal_tx_info[sig_id].bit_pos;
        sig_len = CAN_signal_tx_info[sig_id].sig_len;
        can_ml_writeCanDataByCANID(CAN_signal_tx_info[sig_id].id, sig_val, byte_pos, bit_pos, sig_len);
    }
}

void can_fuction_read_signal_value(CanReceiveID sigId,uint8_t sig,uint32_t *sig_val) {
    switch (sigId){

    case CAN_MSG_RX_ECU3:        //0x111
        can_msg_0x111_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_BCM2:        //0x170
        can_msg_0x170_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_BCM3:        //0x172   
        can_msg_0x172_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_BCM4:        //0x173
        can_msg_0x173_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_BCM5:        //0x195
        can_msg_0x195_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_CCS1:        //0x312
        can_msg_0x312_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_CCS2:        //0x313
        can_msg_0x313_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_IPU1:       //0x204
        can_msg_0x204_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_IPU2:       //0x100
        can_msg_0x100_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_IPU3:       //0x105
        can_msg_0x105_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_IPU4:       //0x106
        can_msg_0x106_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_IPU5:       //0x103
        can_msg_0x103_get_data(sig, sig_val);
        break; 
    case CAN_MSG_RX_BCU1:       //0x185
        can_msg_0x185_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_BCU4:       //0x305
        can_msg_0x305_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_BCU5:       //0x207
        can_msg_0x207_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_BCU6:       //0x502
        can_msg_0x502_get_data(sig, sig_val);
        break;     
    case CAN_MSG_RX_BCU7:       //0x209
        can_msg_0x209_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_IPU6:       //0x202
        can_msg_0x202_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_IPU7:       //0x171
        can_msg_0x171_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_OBC1:       //0x319
        can_msg_0x319_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_OBC2:       //0x349
        can_msg_0x349_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_OBC3:       //0x3a5
        can_msg_0x3a5_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_DCDC1:       //0x299
        can_msg_0x299_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_MSC:        //0x180
        can_msg_0x180_get_data(sig, sig_val);
        break;
    case CAN_MSG_RX_SL1:         //0x188
        can_msg_0x188_get_data(sig, sig_val);
        break;
    default:
        break;
    }
}

void can_message_case_init(){
    //CAN_msg_rx_config_case
    uint8_t idx = 0;
    for (idx = 0; idx < CAN_MSG_RX_NUM; idx++) {
        CAN_msg_rx_config_case[idx].is_data_update = 1;
    }
    can_fuction_app_register();
}

void can_fuction_app_register(void) {
    can_ml_Register_CAN_id_rx_handle(CAN_msg_rx_config_case, CAN_MSG_RX_NUM);
    can_ml_Register_CAN_id_tx_handle(CAN_msg_tx_config_case, CAN_MSG_TX_NUM);
}
