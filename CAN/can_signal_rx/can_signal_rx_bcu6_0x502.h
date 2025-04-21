
#ifndef CAN_SIGNAL_RX_0x502
#define CAN_SIGNAL_RX_0x502

#include <stdint.h>
#include "../can_type.h"

//BCU6,//    0x502
typedef enum {
    BCU6_BCUCellTOver,                                   //电池单体温度过高
    BCU6_BCUBattUOver,                                   //动力蓄电池包过压报警
    BCU6_BCUBattUnder,                                   //动力蓄电池包欠压报警
    BCU6_BCUBattSOCUnder,                                //SOC低报警
    BCU6_BCUBattSOCHi,                                   //SOC太高报警
    BCU6_BCUBattSOCLo,                                   //SOC太低报警
    BCU6_BCUBattSOCJump,                                 //SOC跳变故障
    BCU6_BCUBattChgOverWarn,                             //动力电池过充报警
    
    CAN_SIG_RX_BCU6_COUNT,
} CAN_SIG_RX_BCU6;

uint8_t can_msg_0x502_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x502_timeout_function(void);

void can_msg_0x502_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x502


