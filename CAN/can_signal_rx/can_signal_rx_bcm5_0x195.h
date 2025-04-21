

#ifndef CAN_SIGNAL_RX_0x195
#define CAN_SIGNAL_RX_0x195

#include <stdint.h>
#include "../can_type.h"

//BCM5,//    0x195
typedef enum {
    BCM5_SecondDrive,                //二驱
    BCM5_HandBrakeFlag,              //手刹
    BCM5_SeatBelt_FlashingAlarm,     //安全带闪烁报警
    BCM5_BrakeFluid_AlarmSwitch,     //制动液报警开关
    BCM5_DriversSeatBelt_Switch,     //主驾驶安全带开关
    BCM5_ElsSwitch,                  //应急灯开关
    BCM5_ParkingAlarm,               //驻车行驶报警
    BCM5_Reserve_1,                  //预留1
    BCM5_Reserve_2,                  //预留2
    BCM5_Reserve_3,                  //预留3
    BCM5_Reserve_4,                  //预留4
    BCM5_Reserve_5,                  //预留5
    BCM5_PositionLamp,               //位置灯
    BCM5_LowBeam,                    //近光灯
    BCM5_HighBeam,                   //远光灯
    BCM5_FourWD_FourWheelDrive,      //四驱
    BCM5_FrontAxle_DifferentialLock, //前桥差速锁
    BCM5_LeftTurnLamp,               //左转向灯
    BCM5_RightTurnLamp,              //右转向灯
    BCM5_RearAxle_DifferentialLock,  //后桥差速锁
    BCM5_Coolant_temp,               //冷却液温度
    BCM5_Cool_pumpsd,                //冷却水泵转速
    BCM5_Cool_Fan,                   //冷却风扇

    CAN_SIG_RX_BCM5_COUNT,
} CAN_SIG_RX_BCM5;

uint8_t can_msg_0x195_get_data(uint32_t sig, uint32_t *sig_val);

void can_msg_0x195_timeout_function(void);

void can_msg_0x195_normal_function(CAN_msg_rx_config* can_info, uint8_t timeout);


#endif //CAN_SIGNAL_RX_0x195
