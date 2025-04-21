

#ifndef CAN_SIGNAL_RX_0x349
#define CAN_SIGNAL_RX_0x349

#include <stdint.h>
#include "../can_type.h"

//OBC2,//    0x349
typedef enum {
    OBC2_ObcTOverWarn,                          //OBC过温报警
    OBC2_ObcStopChrgAcUnderWarn,                //交流电压过低停止充电报警
    OBC2_ObcErrNr,                              //充电机故障码

    
    CAN_SIG_RX_OBC2_COUNT,
} CAN_SIG_RX_OBC2;

uint8_t can_msg_0x349_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x349_timeout_function(void);

void can_msg_0x349_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x349

