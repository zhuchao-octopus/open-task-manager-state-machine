/** ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 *
 */
#ifndef __OCTOPUS_TASK_MANAGER_CARINFO_H__
#define __OCTOPUS_TASK_MANAGER_CARINFO_H__

/*******************************************************************************
 * INCLUDES
 */
 
#include "octopus_platform.h"

#ifdef __cplusplus
extern "C"{
#endif
/**
 * \defgroup APP_SETTING APP:SETTING
 * @{
 */


/*******************************************************************************
 * DEBUG SWITCH MACROS
 */


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * TYPEDEFS
 */
typedef struct
{
    uint8_t  sideStand;                  //单撑断电检测        0:单撑收起     1:单撑放下
    uint8_t  bootGuard;                  //启动保护                0:非保护          1:保护中
    uint8_t  hallFault;                  //霍尔故障(电机) 0:正常            1:故障
    uint8_t  throttleFault;              //转把故障
    uint8_t  controllerFault;            //控制器故障
    uint8_t  lowVoltageProtection;       //欠压保护
    uint8_t  cruise;                     //巡航指示灯
    uint8_t  assist;                     //助力指示灯
    uint8_t  motorFault;                 //电机故障
    uint8_t  gear;                       //挡位//0~7
    uint8_t  motorRunning;               //电机运行                 1运行
    uint8_t  brake;                      //刹车
    uint8_t  controllerProtection;       //控制器保护
    uint8_t  coastCharging;              //滑行充电
    uint8_t  antiSpeedProtection;        //防飞车保护
    uint8_t  seventyPercentCurrent;      //70%电流
    uint8_t  pushToTalk;                 //启用一键通
    uint8_t  ekkBackupPower;             //启用EKK备用电源
    uint8_t  overCurrentProtection;      //过流保护
    uint8_t  motorShaftLockProtection;   //堵转保护
    uint8_t  reverse;                    //倒车
    uint8_t  electronicBrake;            //电子刹车
    uint8_t  speedLimit;                 //限速
    uint8_t  current;                    //电流 单位：A
    uint16_t hallCounter;                //0.5s 内三个霍尔变化的个数
    uint8_t  soc;                        //电量/电量 0-100% 5灯指示为 90,70,50,30,20（百分比，建议对应的电压大体为 47V，46V,44.5V,43V,41V)，4 灯指示为 90,70,50,30
    uint8_t  voltageSystem;              //电压系统  0x01:36V  0x02:48V  0x04:60V  0x08:64V  0x10:72V  0x20:80V  0x40:84V   0x80:96V
}carinfo_sif_t;

typedef struct
{
    uint8_t SideStand;                  //单撑断电检测        0:单撑收起     1:单撑放下
    uint8_t BootGuard;                  //启动保护                0:非保护          1:保护中
    uint8_t hallFault;                  //霍尔故障(电机) 0:正常            1:故障
    uint8_t throttleFault;              //转把故障
    uint8_t controllerFault;            //控制器故障
    uint8_t lowVoltageProtection;       //欠压保护
    uint8_t cruise;                     //巡航指示灯
    uint8_t assist;                     //助力指示灯
    uint8_t motorFault;                 //电机故障
    uint8_t gear;                       //挡位//0~7
    uint8_t motorRunning;               //电机运行                 1运行
    uint8_t brake;                      //刹车
    uint8_t controllerProtection;       //控制器保护
    uint8_t coastCharging;              //滑行充电
    uint8_t antiSpeedProtection;        //防飞车保护
    uint8_t seventyPercentCurrent;      //70%电流
    uint8_t pushToTalk;                 //启用一键通
    uint8_t ekkBackupPower;             //启用EKK备用电源
    uint8_t overCurrentProtection;      //过流保护
    uint8_t motorShaftLockProtection;   //堵转保护
    uint8_t reverse;                    //倒车
    uint8_t electronicBrake;            //电子刹车
    uint8_t speedLimit;                 //限速
    uint32_t current;                   //电流 单位：0.1A
    uint32_t voltage;                   //电压 单位：0.1V
    uint8_t voltageSystem;              //电压系统  0x01:36V  0x02:48V  0x04:60V  0x08:64V  0x10:72V  0x20:80V  0x40:84V   0x80:96V
    uint8_t soc;                        //电量5 灯指示为 90,70,50,30,20（百分比，建议对应的电压大体为 47V，46V,44.5V,43V,41V)
    uint32_t speed;                     //车速 单位：0.1km/h
    uint32_t speed_real;                //实际车速 单位：0.1km/h
    uint32_t rpm;                       //转速 单位：rpm
}carinfo_t;



typedef struct
{
    uint16_t current;                   //电流 单位：0.1A
    uint16_t voltage;                   //电压 单位：0.1V
    uint8_t  soc;                        //电量/电量 0-100% 5灯指示为 90,70,50,30,20（百分比，建议对应的电压大体为 47V，46V,44.5V,43V,41V)，4 灯指示为 90,70,50,30
    uint16_t speed;                     //车速 单位：0.1km/h
    uint16_t speed_real;                //实际车速 单位：0.1km/h
    uint16_t rpm;                       //转速 单位：rpm offset:-20000
    uint8_t  voltageSystem;             //电压系统  0x01:36V  0x02:48V  0x04:60V  0x08:64V  0x10:72V  0x20:80V  0x40:84V   0x80:96V
}carinfo_meter_t;

typedef struct
{
    uint8_t highBeam;       //远光灯
    uint8_t lowBeam;        //近光灯
    uint8_t position;       //位置灯
    uint8_t frontFog;       //前雾灯
    uint8_t rearFog;        //后雾灯
    uint8_t leftTurn;       //左转灯
    uint8_t rightTurn;      //右转灯
    uint8_t ready;          //Ready灯
    uint8_t charge;         //电池充放电灯
    uint8_t parking;        //驻车灯
    uint8_t ecuFault;       //ECU故障灯
    uint8_t sensorFault;    //传感器故障灯
    uint8_t motorFault;     //电机故障灯
    uint8_t bt;             //蓝牙指示灯
    uint8_t wifi;           //WIFI指示灯

}carinfo_indicator_t;


typedef struct
{
    uint16_t avgEnergyConsumption; //平均能耗
    uint16_t travelTime;           //行驶总时间,单位分钟
    uint16_t avgSpeed;             //平均速度，单位0.1km/h
}carinfo_drivinfo_form_t;

typedef enum{
    DRIVINFO_GEAR_UNKNOWN          = 0x00,      //未知
    DRIVINFO_GEAR_MANUAL_1         = 0x01,      //手动1档
    DRIVINFO_GEAR_MANUAL_2         = 0x02,      //手动2档
    DRIVINFO_GEAR_MANUAL_3         = 0x03,      //手动3档
    DRIVINFO_GEAR_MANUAL_4         = 0x04,      //手动4档
    DRIVINFO_GEAR_MANUAL_5         = 0x05,      //手动5档
    DRIVINFO_GEAR_MANUAL_6         = 0x06,      //手动6档
    DRIVINFO_GEAR_MANUAL_7         = 0x07,      //手动7档

    DRIVINFO_GEAR_REVERSE          = 0x0A,      //(R)倒车档
    DRIVINFO_GEAR_PARK             = 0x0B,      //(P)驻车档
    DRIVINFO_GEAR_NEUTRAL          = 0x0C,      //(N)空档
    DRIVINFO_GEAR_DRIVE            = 0x0D,      //(D)前进档
    DRIVINFO_GEAR_SPORT            = 0x0E,      //(S)运动档
    DRIVINFO_GEAR_FAULT            = 0x0F,      //(F)故障
}carinfo_drivinfo_gear_t;

typedef enum{
    DRIVINFO_DRIVEMODE_Comfort  = 0x00,    //Comfort
    DRIVINFO_DRIVEMODE_ECO      = 0x01,    //ECO
    DRIVINFO_DRIVEMODE_AUTO     = 0x02,    //AUTO
    DRIVINFO_DRIVEMODE_Sport    = 0x03,    //Sport
    DRIVINFO_DRIVEMODE_SNOW     = 0x04,    //SNOW
    DRIVINFO_DRIVEMODE_OFF_ROAD = 0x05,    //OFF ROAD
}carinfo_drivinfo_drivemode_t;

typedef struct
{
    uint32_t odo;                       //总里程，单位0.1km
    uint16_t tripA;                     //小计里程A，单位0.1km
    uint16_t tripB;                     //小计里程B，单位0.1km
    uint8_t energyType;                 //能源类型  0x00:燃油(0.L/100KM)  0x01电能(0.1kWh/100KM)
    uint16_t enduranceMileage;          //续航里程
    uint16_t insEnergyConsumption;      //瞬时能耗

    carinfo_drivinfo_form_t odoForm;    //里程统计信息表单
    carinfo_drivinfo_form_t tripAForm;  //小计里程A统计信息
    carinfo_drivinfo_form_t tripBForm;  //小计里程B统计信息

    carinfo_drivinfo_gear_t gear;            //档位信息
    carinfo_drivinfo_drivemode_t driveMode;  //驾驶模式
}carinfo_drivinfo_t;

/*******************************************************************************
 * CONSTANTS
 */


/*******************************************************************************
 * GLOBAL VARIABLES DECLEAR
 */

/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */

void app_carinfo_init_running(void);
void app_carinfo_start_running(void);
void app_carinfo_assert_running(void);
void app_carinfo_running(void);
void app_carinfo_post_running(void);
void app_carinfo_stop_running(void);


///void app_carinfo_on_enter_run(void);

///void app_carinfo_on_exit_post_run(void);

uint16_t app_carinfo_getSpeed(void);
carinfo_meter_t* app_carinfo_get_meter_info(void);
carinfo_indicator_t* app_carinfo_get_indicator_info(void);
carinfo_drivinfo_t* app_carinfo_get_drivinfo_info(void);

/**
 * end of group APP_SETTING
 * @}
 */

#ifdef __cplusplus
}
#endif


#endif
