//
// Created by kuro on 2018/11/29.
//

#ifndef SOURCES_DATA_UDS_H
#define SOURCES_DATA_UDS_H

#include <stdint.h>
#include <stdbool.h>


typedef struct {
    bool  vehicle_speed_ctrl;
    uint16_t vehicle_speed_value;              // 发送车速 单位为0.1KM/h 
    
    bool  engine_speed_rpm_ctrl;
    uint16_t engine_speed_rpm_value;            // 发送发动机速度 offset:-20000 单位rpm
    
    bool  engine_coolant_temp_ctrl;
    uint8_t engine_coolant_temp_value;          // 发动机冷却液温度 -40 ~ 130℃ offset:-40  无效0xFF
    
    bool  residual_fuel_ctrl;
    uint8_t residual_fuel_value;               // 剩余燃油位 0-80   无效0xFF
    
    bool  actual_gear_ctrl;
    uint8_t actual_gear_value;                 // 档位

    bool  outside_temperature_ctrl;
    uint16_t outside_temperature_value;       // 外部温度 -50.0~125.0℃ 单位1℃ offset:-50
    
    bool      back_light_ctrl;
    uint8_t   back_light_value;                 //背光控制
    
    bool      lcd_ctrl;
    uint8_t   lcd_value;                        //LCD测试
    
    bool      buzzer_ctrl;
    uint8_t   buzzer_value;                      //扬声器测试
}UdsInfo;

UdsInfo* data_usdinfo_get_usdinfo();


#endif //SOURCES_DATA_UDS_H
