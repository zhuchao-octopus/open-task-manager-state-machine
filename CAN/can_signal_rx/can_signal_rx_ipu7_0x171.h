
#ifndef CAN_SIGNAL_RX_0x171
#define CAN_SIGNAL_RX_0x171

#include <stdint.h>
#include "../can_type.h"

//IPU7,//    0x171
typedef enum {
    IPU7_IPUBrakeFlag,                           //脚刹
    IPU7_IPUVehicleInDriveFlag,                  //D挡
    IPU7_IPUVehicleInRearlag,                    //R挡
    IPU7_IPUVehicleInNeutralFlag,                //N挡
    IPU7_IPUH_Gear,                              //H档：标准模式   
    IPU7_IPUL_Gear,                              //L档：运动模式
    IPU7_IPUM_Gear,                              //M档：经济模式
    IPU7_IPUAccPeadal,                           //油门开度

    
    CAN_SIG_RX_IPU7_COUNT,
} CAN_SIG_RX_IPU7;

uint8_t can_msg_0x171_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x171_timeout_function(void);

void can_msg_0x171_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x171



