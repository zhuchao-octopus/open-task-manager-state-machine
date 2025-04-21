

#ifndef CAN_SIGNAL_RX_0x781
#define CAN_SIGNAL_RX_0x781

#include <stdint.h>
#include "../can_type.h"

void can_msg_0x781_timeout_function(void);

void can_msg_0x781_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x781
