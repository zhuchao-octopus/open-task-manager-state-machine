
#include "can_signal_rx_uds_request_0x781.h"

#include "../can_message_rx.h"
#include "../can_function.h"


void can_msg_0x781_timeout_function(void) {
    CANMSG_TIMEROUT_DBG(PRINTF("%s\n", __FUNCTION__));
}


void can_msg_0x781_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout) {
    CANMSG_NORMAL_DBG(PRINTF("%s\n", __FUNCTION__));
    
}
