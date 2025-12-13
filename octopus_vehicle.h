/********************************** (C) COPYRIGHT *******************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * @file    octopus_task_manager_carinfo.h
 * @brief   This header file defines structures, macros, constants, and function
 *          declarations related to managing car information within the Octopus
 *          task manager system.
 *
 * @details This file contains the definitions for monitoring various car
 *          parameters, including electrical and mechanical states such as
 *          battery voltage, motor faults, gear positions, and more. It also
 *          provides the necessary functions to manage and retrieve car information
 *          during system operation. This is essential for vehicle control systems
 *          that require precise monitoring of the car's status in real-time.
 *
 * @note    This software (modified or not) and binary are intended for use
 *          with microcontrollers manufactured by Nanjing Qinheng Microelectronics.
 * @author  Octopus Team
 * @version 1.0.0
 * @date    2024-12-09
 *******************************************************************************/
#ifndef __OCTOPUS_TASK_MANAGER_CARINFO_H__
#define __OCTOPUS_TASK_MANAGER_CARINFO_H__

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_base.h" //  Base include file for the Octopus project.

/*******************************************************************************
 * MACROS
 */
#define CELL_COUNT 64
#define ERROR_CODE_COUNT 10
#define DEFAULT_CONSUMPTION_WH_PER_KM (10.0f) // 经验值：Wh/km，可按实际调整
#define DEFAULT_SAFETY_RESERVE_RATIO (0.10f)  // 预留 10% 容量为安全余量（可设0）

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */

/*******************************************************************************
 * TYPEDEFS
 */
typedef enum
{
    DRIVE_MOD_REAR = 0, // 后驱
    DRIVE_MOD_FRONT,    // 前驱
    DRIVE_MOD_DOUBLE,   // 双驱
} drive_mode_;

typedef enum
{
    BMS_MODE_INIT = 0x00,      // 初始化模式
    BMS_MODE_STANDBY = 0x01,   // 待机模式
    BMS_MODE_DISCHARGE = 0x02, // 放电模式
    BMS_MODE_CHARGE = 0x03,    // 充电模式
    BMS_MODE_SLEEP = 0x04      // 睡眠模式
} BMS_Mode_t;
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
// 故障-故障信息
typedef enum __attribute__((packed))
{
    // ===========================
    // 基础状态
    // ===========================
    ERROR_CODE_IDLE = 0x00,                            // 无动作
    ERROR_CODE_NORMAL = 0x01,                          // 正常状态
    ERROR_CODE_BRAKE = 0x03,                           // 已刹车
    ERROR_CODE_THROTTLE_NOT_ZERO = 0x04,               // 转把没有归位（停在高位处）
    ERROR_CODE_THROTTLE_HALLSENSOR_ABNORMALITY = 0x05, // 转把故障

    // ===========================
    // 电压保护
    // ===========================
    ERROR_CODE_LOW_VOLTAGE_PROTECTION = 0x06,  // 低电压保护
    ERROR_CODE_OVER_VOLTAGE_PROTECTION = 0x07, // 过电压保护

    // ===========================
    // 电机相关
    // ===========================
    ERROR_CODE_HALLSENSOR_ABNORMALITY = 0x08,               // 电机霍尔信号线故障
    ERROR_CODE_MOTOR_ABNORMALITY = 0x09,                    // 电机相线故障
    ERROR_CODE_MOTOR_OVERHEAT = 0x31,                       // 电机过热
    ERROR_CODE_MOTOR_PHASE_LOSS = 0x38,                     // 电机缺相
    ERROR_CODE_MOTOR_LOCK = 0x3F,                           // 电机锁定保护
    ERROR_CODE_MOTOR_CURRENT_IMBALANCE = 0x40,              // 电机相电流不平衡
    ERROR_CODE_MOTOR_DRIVER_OVERLOAD = 0x41,                // 电机驱动器过载
    ERROR_CODE_MOTOR_UNDER_TEMP = 0x42,                     // 电机温度过低
    ERROR_CODE_MOTOR_HALL_NOISE = 0x43,                     // 霍尔信号噪声过大
    ERROR_CODE_MOTOR_TEMPERATURE_SENSOR_ABNORMALITY = 0x14, // 电机温度传感器故障

    // ===========================
    // 控制器相关
    // ===========================
    ERROR_CODE_CONTROLLER_OVERHEAT = 0x10,                       // 控制器过热保护
    ERROR_CODE_CONTROLLER_TEMPERATURE_SENSOR_ABNORMALITY = 0x11, // 控制器温度传感器故障
    ERROR_CODE_CURRENT_SENSOR_ABNORMALITY = 0x12,                // 电流传感器故障
    ERROR_CODE_CONTROLLER_ABNORMALITY = 0x15,                    // 控制器故障
    ERROR_CODE_CONTROLLER_UNDER_TEMP = 0x32,                     // 控制器温度过低
    ERROR_CODE_CONTROLLER_COMMUNICATION_LOST = 0x44,             // 控制器内部CAN通信丢失
    ERROR_CODE_CONTROLLER_SOFTWARE_EXCEPTION = 0x45,             // 控制器软件异常/重启
    ERROR_CODE_CONTROLLER_HARDWARE_FAULT = 0x46,                 // 控制器硬件故障(MOSFET/IGBT)
    ERROR_CODE_CONTROLLER_PROTECTION_TRIGGER = 0x47,             // 控制器防护触发（短路/过流）
    ERROR_CODE_EEPROM_ABNORMALITY = 0x3A,                        // 控制器EEPROM故障
    ERROR_CODE_FAN_ABNORMALITY = 0x3B,                           // 风扇故障

    // ===========================
    // 助力/传感器
    // ===========================
    ERROR_CODE_ASSIST_POWER_SENSOR_ABNORMALITY = 0x16, // 助力传感器故障
    ERROR_CODE_SPEED_SENSOR_ABNORMALITY = 0x21,        // 速度传感器故障
    ERROR_CODE_BRAKE_SENSOR_ABNORMALITY = 0x33,        // 刹车传感器故障
    ERROR_CODE_THROTTLE_SHORT_CIRCUIT = 0x34,          // 转把短路
    ERROR_CODE_THROTTLE_OPEN_CIRCUIT = 0x35,           // 转把断路
    ERROR_CODE_TORQUE_SENSOR_ABNORMALITY = 0x3D,       // 扭矩传感器故障
    ERROR_CODE_REVERSE_SENSOR_ABNORMALITY = 0x3C,      // 倒车传感器故障
    ERROR_CODE_BATTERY_TEMP_SENSOR_ABNORMALITY = 0x3E, // 电池温度传感器故障

    // ===========================
    // 电池相关
    // ===========================
    ERROR_CODE_BATTERY_OVERHEAT = 0x13,                  // 电池过热
    ERROR_CODE_BATTERY_UNDER_VOLTAGE = 0x36,             // 电池欠压
    ERROR_CODE_BATTERY_OVER_VOLTAGE = 0x37,              // 电池过压
    ERROR_CODE_BATTERY_SINGLE_CELL_OVER_VOLTAGE = 0x48,  // 电池单体过压
    ERROR_CODE_BATTERY_SINGLE_CELL_UNDER_VOLTAGE = 0x49, // 电池单体欠压
    ERROR_CODE_BATTERY_TEMP_DIFFERENCE = 0x4A,           // 电池温差过大
    ERROR_CODE_BATTERY_OVER_CURRENT = 0x4B,              // 电池过流
    ERROR_CODE_BMS_ABNORMALITY = 0x22,                   // BMS通讯故障

    // ===========================
    // 灯光/外设
    // ===========================
    ERROR_CODE_LAMP_ABNORMALITY = 0x23,        // 大灯故障
    ERROR_CODE_LAMP_SENSOR_ABNORMALITY = 0x24, // 大灯传感器故障
    ERROR_CODE_LIGHT_OVERCURRENT = 0x50,       // 大灯过流/熔断
    ERROR_CODE_TURN_SIGNAL_FAULT = 0x51,       // 转向灯故障
    ERROR_CODE_DISPLAY_FAULT = 0x52,           // 仪表/显示屏故障

    // ===========================
    // 通讯相关
    // ===========================
    ERROR_CODE_COMMUNICATION_ABNORMALITY = 0x30,    // 总体通讯故障
    ERROR_CODE_CAN_BMS_TIMEOUT = 0x53,              // BMS CAN通信超时
    ERROR_CODE_CAN_MOTOR_CONTROLLER_TIMEOUT = 0x54, // 电机控制器CAN通信异常
    ERROR_CODE_CAN_EXTERNAL_DEVICE_ERROR = 0x55     // 外部设备CAN总线错误
} __attribute__((packed)) ERROR_CODE;

#define ERROR_FLAG_BYTES 16 // 16 字节 = 128 bit，可表示最多 128 个错误
// 设置错误
#define CAR_INFOR_SET_ERROR(err_struct, code) ((err_struct).flags[(code) / 8] |= (1 << ((code) % 8)))
// 清除错误
#define CAR_INFOR_CLEAR_ERROR(err_struct, code) ((err_struct).flags[(code) / 8] &= ~(1 << ((code) % 8)))
// 检查错误
#define CAR_INFOR_CHECK_ERROR(err_struct, code) (((err_struct).flags[(code) / 8] & (1 << ((code) % 8))) != 0)

#define ERROR_CODE_BEGIN ERROR_CODE_THROTTLE_NOT_ZERO       // 故障码开始
#define ERROR_CODE_END ERROR_CODE_COMMUNICATION_ABNORMALITY // 故障码结束

#pragma pack(push, 1)
typedef struct
{
    uint8_t flags[ERROR_FLAG_BYTES];
} __attribute__((aligned(4))) CarErrorCodeFlags_t;

typedef struct
{
    uint8_t ready;      // Ready status (1 = system ready to operate)
    uint8_t high_beam;  // High beam light status (1 = ON)
    uint8_t low_beam;   // Low beam light status (1 = ON)
    uint8_t width_lamp; // Position/marker light status
    uint8_t front_fog;  // Front fog light status
    uint8_t rear_fog;   // Rear fog light status
    uint8_t left_turn;  // Left turn signal indicator status
    uint8_t right_turn; // Right turn signal indicator status

    uint8_t parking;             // Parking status (1 = in parking mode)
    uint8_t brake;               // Brake status (1 = braking)
    uint8_t horn;                // Horn status (1 = horn active)
    uint8_t cruise_control;      // Cruise control status (1 = active)
    uint8_t start_poles;         // Start behavior after motor poles setup
    uint8_t motor_poles;         // Number of poles in motor (raw value)
    uint8_t horizontal_position; // Horizontal orientation/position sensor value

    uint8_t walk_assist; // Walk assist status (1 = enabled)
    uint8_t drive_mode;  // Drive mode selection (0 = eco, 1 = normal, etc.)
    uint8_t start_mode;  // Start mode setting (e.g., throttle/pedal)

    uint8_t reverse;
    uint8_t reserve; // Wi-Fi indicator status (1 = connected)
} __attribute__((aligned(4))) carinfo_indicator_t;

typedef struct
{
    uint32_t trip_odo;      // Total distance traveled (unit: 1 meters), also known as trip odometer
    uint32_t trip_time;     // Total ride time (unit: seconds)
    uint32_t trip_distance; // Trip distance   (unit: 1 meters), resettable

    uint16_t speed_average; // Displayed vehicle speed (unit: 0.1 km/h)
    uint16_t speed_actual;  // Actual wheel speed (unit: 0.1 km/h)
    uint16_t speed_max;
    uint16_t speed_limit; // Speed limit setting; 0 = OFF, range: 10–90 km/h

    uint16_t rpm;           // Motor RPM (raw value, offset by -20000)
    uint8_t gear;           // Current gear level (0 = Neutral, 1–N)
    uint8_t gear_level_max; // Maximum selectable gear level
    uint8_t wheel_diameter; // Wheel diameter (unit: inch)
    uint8_t reserve;
} __attribute__((packed, aligned(4))) carinfo_meter_t;

typedef struct
{
    uint16_t voltage;      // Battery voltage (unit: millivolts or volts depending on context  0.1V unit)
    uint16_t current;      // Battery or motor current (unit: milliamps or amps  0.1Ah unit)
    uint16_t power;        // Instantaneous power output (unit: watts)
    uint16_t soc;          // State of Charge, 0–100% (based on voltage/SOC curve)
    uint16_t range;        // Estimated remaining range (unit: 100 meters)
    uint16_t range_max;    // Estimated maximum range (unit: 100 meters)
    uint16_t throttle_pwm; // Throttle signal PWM duty (0–1000 for 0–100%)

    uint8_t current_limit;    // Current limit, range: 6~50A, default: 12A, unit: 1A
    uint8_t rel_charge_state; // Relative charge state (e.g., fast/slow charging, enum value)
    uint8_t abs_charge_state; // Absolute charge state (e.g., charging, full, fault, enum value)
    uint8_t reserve1;
    uint16_t reserve2;
} __attribute__((aligned(4))) carinfo_battery_t;

// 故障信息
typedef struct
{
    uint8_t fault_ecu;    // ECU fault status
    uint8_t fault_sensor; // Sensor fault status//assist_power_sensor_switch
    uint8_t fault_motor;  // Motor fault status

    uint8_t fault_fuse;     // Fuse fault status (0: OK, 1: Fault, others: reserved)
    uint8_t fault_plug;     // Charging plug fault status
    uint8_t fault_battery;  // Battery fault status
    uint8_t fault_brake;    // Brake fault status
    uint8_t fault_throttle; // Throttle fault status
} __attribute__((aligned(4))) carinfo_error_t;
#pragma pack(pop)

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
#if 0		
    typedef struct
    {
        uint16_t avgEnergyConsumption; // Average energy consumption
        uint16_t travelTime;           // Travel time in minutes
        uint16_t avgSpeed;             // Average speed (0.1 km/h)
    } carinfo_drivinfo_form_t;

    typedef enum
    {
        DRIVINFO_GEAR_UNKNOWN = 0x00,  // Unknown
        DRIVINFO_GEAR_MANUAL_1 = 0x01, // Manual gear 1
        DRIVINFO_GEAR_MANUAL_2 = 0x02, // Manual gear 2
        DRIVINFO_GEAR_MANUAL_3 = 0x03, // Manual gear 3
        DRIVINFO_GEAR_MANUAL_4 = 0x04, // Manual gear 4
        DRIVINFO_GEAR_MANUAL_5 = 0x05, // Manual gear 5
        DRIVINFO_GEAR_MANUAL_6 = 0x06, // Manual gear 6
        DRIVINFO_GEAR_MANUAL_7 = 0x07, // Manual gear 7

        DRIVINFO_GEAR_REVERSE = 0x0A, // Reverse gear
        DRIVINFO_GEAR_PARK = 0x0B,    // Park gear
        DRIVINFO_GEAR_NEUTRAL = 0x0C, // Neutral gear
        DRIVINFO_GEAR_DRIVE = 0x0D,   // Drive gear
        DRIVINFO_GEAR_SPORT = 0x0E,   // Sport mode
        DRIVINFO_GEAR_FAULT = 0x0F,   // Gear fault
    } carinfo_drivinfo_gear_t;

    typedef enum
    {
        DRIVINFO_DRIVEMODE_Comfort = 0x00,  // Comfort mode
        DRIVINFO_DRIVEMODE_ECO = 0x01,      // ECO mode
        DRIVINFO_DRIVEMODE_AUTO = 0x02,     // Auto mode
        DRIVINFO_DRIVEMODE_Sport = 0x03,    // Sport mode
        DRIVINFO_DRIVEMODE_SNOW = 0x04,     // Snow mode
        DRIVINFO_DRIVEMODE_OFF_ROAD = 0x05, // Off-road mode
    } carinfo_drivinfo_drivemode_t;

#pragma pack(push, 1)
    typedef struct
    {
        uint32_t odo;                  // Odometer (0.1 km)
        uint16_t tripA;                // Trip A distance (0.1 km)
        uint16_t tripB;                // Trip B distance (0.1 km)
        uint8_t energyType;            // Energy type: 0x00: fuel (L/100KM), 0x01: electric (0.1 kWh/100KM)
        uint16_t enduranceMileage;     // Endurance mileage (0.1 km)
        uint16_t insEnergyConsumption; // Instantaneous energy consumption (0.1 kWh/km)

        carinfo_drivinfo_form_t odoForm;   // Odometer driving information
        carinfo_drivinfo_form_t tripAForm; // Trip A driving information
        carinfo_drivinfo_form_t tripBForm; // Trip B driving information

        carinfo_drivinfo_gear_t gear;           // Gear information
        carinfo_drivinfo_drivemode_t driveMode; // Driving mode
    } carinfo_drivinfo_t;
#endif
/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * GLOBAL VARIABLES DECLEAR
 */

/*******************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */
#ifdef __cplusplus
extern "C"
{
#endif
    void task_vehicle_init_running(void);
    void task_vehicle_start_running(void);
    void task_vehicle_assert_running(void);
    void task_vehicle_running(void);
    void task_vehicle_post_running(void);
    void task_vehicle_stop_running(void);

    /**
     * @brief Retrieve current vehicle speed (0.1 km/h).
     * @return uint16_t Speed in 0.1 km/h.
     */
    uint16_t task_carinfo_getSpeed(void);

    /**
     * @brief Retrieve indicator status structure.
     * @return Pointer to current carinfo_indicator_t.
     */
    carinfo_indicator_t *task_carinfo_get_indicator_info(void);
    /**
     * @brief Retrieve meter information pointer.
     * @return Pointer to current carinfo_meter_t.
     */
    carinfo_meter_t *task_carinfo_get_meter_info(void);
    carinfo_battery_t *task_carinfo_get_battery_info(void);
    carinfo_error_t *task_carinfo_get_error_info(void);
    /**
     * @brief Retrieve drive information.
     * @return Pointer to current carinfo_drivinfo_t.
     */
    // carinfo_drivinfo_t *task_carinfo_get_drivinfo_info(void);

    /**
     * @brief Initializes the car indicator module.
     *        This function resets all indicator flags, change flags, and warning flags.
     */
    // void car_indicator_init(void);

    /**
     * @brief Gets the value of the indicator flag array at the specified index.
     *
     * @param index The index to retrieve (0 to INDICATOR_TYPE_ARRAY_COUNT - 1).
     * @param indicator Pointer to store the value at the specified index.
     * @return uint8_t Returns 1 if the index is valid and the value is retrieved, 0 otherwise.
     */
    // uint8_t car_indicator_get_indicator(uint8_t index, uint8_t *indicator);

    /**
     * @brief Returns the number of bytes used to store all indicator flags.
     *
     * @return uint8_t The count of indicator flag array elements.
     */
    // uint8_t data_indicator_get_indicator_type_array_count(void);

    /**
     * @brief Gets the status of a specific indicator.
     *
     * @param type The indicator type to check.
     * @return uint8_t Returns 1 if the indicator is on, 0 if off.
     */
    // uint8_t car_indicator_get_indicatorflag(IndicatorType type);

    /**
     * @brief Sets the status of a specific indicator.
     *        If the status has changed, the change flag will be updated accordingly.
     *
     * @param type The indicator type to set.
     * @param flag The new status (1 for on, 0 for off).
     */
    // void car_indicator_set_indicatorflag(IndicatorType type, uint8_t flag);

    /**
     * @brief Checks whether any indicator flag has changed since last clear.
     *
     * @return uint8_t Returns 1 if any change has occurred, 0 otherwise.
     */
    // uint8_t car_indicator_get_indicatorflag_changed(void);

    /**
     * @brief Clears all indicator flag change records.
     */
    // void car_indicator_clear_indicatorflag_changed(void);

    /**
     * @brief Gets the current status of the hazard warning flag.
     *
     * @return uint8_t Returns 1 if hazard is active, 0 otherwise.
     */
    // uint8_t car_indicator_get_hazard(void);

    /**
     * @brief Sets the hazard warning flag status.
     *
     * @param flag 1 to enable hazard, 0 to disable.
     */
    // void car_indicator_set_hazard(uint8_t flag);

    /**
     * @brief Gets the warning flag status for a specific indicator.
     *
     * @param type The indicator type to check.
     * @return uint8_t Returns 1 if warning flag is set, 0 otherwise.
     */
    // uint8_t car_indicator_get_warnflag(IndicatorType type);

    /**
     * @brief Sets or clears the warning flag for a specific indicator.
     *        Setting the flag also starts the warning timer.
     *
     * @param type The indicator type to set.
     * @param flag 1 to set the warning, 0 to clear.
     */
    // void car_indicator_set_warnflag(IndicatorType type, uint8_t flag);

    /**
     * @brief Checks whether there is an active warning event.
     *        This can be based on current warning flags or timer condition.
     *
     * @return uint8_t Returns 1 if a warning event is active, 0 otherwise.
     */
    // uint8_t car_indicator_get_warnflag_event(void);

    // void car_indicator_proc_turn_signal(void);
    // void car_meter_proc_speed_rpm(void);
    void task_car_reset_trip(void);
    bool task_car_has_error_code(void);

    void battary_update_simulate_infor(void);
    void carinfo_add_error_code(ERROR_CODE error_code, bool code_append, bool update_immediately);

    extern carinfo_meter_t lt_carinfo_meter;         // Local meter data structure
    extern carinfo_indicator_t lt_carinfo_indicator; // Local indicator data structure
    extern carinfo_battery_t lt_carinfo_battery;
    extern carinfo_error_t lt_carinfo_error;
    extern CarErrorCodeFlags_t CarErrorCodeFlags;

    //////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////
    extern uint16_t adc_get_value_v(void);

#ifdef __cplusplus
}
#endif

#endif
