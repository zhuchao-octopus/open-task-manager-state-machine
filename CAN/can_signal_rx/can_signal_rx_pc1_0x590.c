
#include "can_signal_rx_pc1_0x590.h"

#include "../can_type.h"
#include "../can_message_rx.h"
#include "../can_function.h"


///static uint8_t odo_rest_cfg_stp = 0;
///static TimerType odo_rest_cfg_timer = 0;

#define MOTPOWER_15         0
#define MOTPOWER_18         1

#define REST_ODO_FLAG       1
#define SET_MOT_15_FLAG     2
#define SET_MOT_18_FLAG     3

const CAN_signal_config CAN_signal_rx_PC1[CAN_SIG_RX_PC1_COUNT] = {
    [PC1_ICU_CFG] = {.id = 0x590, .message_name = CAN_MSG_RX_PC1, .byte_pos = 0, .bit_pos = 0, .sig_len = 8, .default_val = 0x0, .invalid_val = 0x0},  //仪表配置
};


uint8_t can_msg_0x590_get_data(uint32_t sig, uint32_t *sig_val){
    return can_fuction_read_signal_value_callback(CAN_signal_rx_PC1[sig], sig_val);
}

void can_msg_0x590_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
    uint8_t i = 0;
    for (i = 0; i < CAN_SIG_RX_PC1_COUNT; i++) {
        can_fuction_timeout_reset_candata_callback(CAN_signal_rx_PC1[i]);
    }
}

void can_msg_0x590_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));
	
    #if 0
    if(timeout == 0)
    {
        //static uint32_t sig_flag = 0;
        static uint8_t sig_flag = 0;
        uint32_t sig_val = 0;

        UserSetting* set = data_carinfo_get_user_setting();
        can_msg_0x590_get_data(PC1_ICU_CFG, &sig_val);
        
        if(IsTimerStart(&odo_rest_cfg_timer))
        {
            if(GetTimer(&odo_rest_cfg_timer) > 5000)
            {
                StopTimer(&odo_rest_cfg_timer);
                odo_rest_cfg_stp = 0;
                sig_flag = 0;
                set->powerflag = false;
            }
        }
        switch(odo_rest_cfg_stp)
        {
            case 0:
                if(sig_val == 0x2E)
                {
                    StartTimer(&odo_rest_cfg_timer);
                    odo_rest_cfg_stp = 1;
                    //PRINTF("MOTORPOWER _______0000___ 2E \r\n");
                }
                break;
            case 1:
                if(sig_val == 0x18)
                {
                    StartTimer(&odo_rest_cfg_timer);
                    odo_rest_cfg_stp = 2;
                    sig_flag = REST_ODO_FLAG;
                }
                else if(sig_val == 0x21)
                {
                    StartTimer(&odo_rest_cfg_timer);
                    odo_rest_cfg_stp = 2;
                    sig_flag = SET_MOT_15_FLAG;
                    //PRINTF("MOTORPOWER _______1111___ 21 \r\n");
                }
                else if(sig_val == 0x22)
                {
                    StartTimer(&odo_rest_cfg_timer);
                    odo_rest_cfg_stp = 2;
                    sig_flag = SET_MOT_18_FLAG;
                    //PRINTF("MOTORPOWER _______1111___ 22 \r\n");
                }
                else
                {
                    StopTimer(&odo_rest_cfg_timer);
                    odo_rest_cfg_stp = 0;
                    sig_flag = 0;
                }
                break;
             case 2:
                if(sig_val == 0x01)
                {   
                    if(sig_flag == REST_ODO_FLAG)
                    {
                        sys_app_mark_set_clear_odo_and_engine_hours_pc();//复位ODO里程
                    }
                    else if(sig_flag == SET_MOT_15_FLAG)
                    {
                        set->pc_motorpower = MOTPOWER_15;//15kw电机
                        set->powerflag = true;
                    }
                    else if(sig_flag == SET_MOT_18_FLAG)
                    {
                        set->pc_motorpower = MOTPOWER_18;//18kw电机
                        set->powerflag = true;
                    }
                }
                StopTimer(&odo_rest_cfg_timer);
                odo_rest_cfg_stp = 0;
                sig_flag = 0;
                break;
        }
    }
		#endif
}


