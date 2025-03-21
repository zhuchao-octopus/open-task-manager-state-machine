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

#include "octopus_platform.h"

#ifdef __cplusplus
extern "C"
{
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
        uint8_t sideStand;                // ���Ŷϵ���        0:��������     1:���ŷ���
        uint8_t bootGuard;                // ��������                0:�Ǳ���          1:������
        uint8_t hallFault;                // ��������(���) 0:����            1:����
        uint8_t throttleFault;            // ת�ѹ���
        uint8_t controllerFault;          // ����������
        uint8_t lowVoltageProtection;     // Ƿѹ����
        uint8_t cruise;                   // Ѳ��ָʾ��
        uint8_t assist;                   // ����ָʾ��
        uint8_t motorFault;               // �������
        uint8_t gear;                     // ��λ//0~7
        uint8_t motorRunning;             // �������                 1����
        uint8_t brake;                    // ɲ��
        uint8_t controllerProtection;     // ����������
        uint8_t coastCharging;            // ���г��
        uint8_t antiSpeedProtection;      // ���ɳ�����
        uint8_t seventyPercentCurrent;    // 70%����
        uint8_t pushToTalk;               // ����һ��ͨ
        uint8_t ekkBackupPower;           // ����EKK���õ�Դ
        uint8_t overCurrentProtection;    // ��������
        uint8_t motorShaftLockProtection; // ��ת����
        uint8_t reverse;                  // ����
        uint8_t electronicBrake;          // ����ɲ��
        uint8_t speedLimit;               // ����
        uint8_t current;                  // ���� ��λ��A
        uint16_t hallCounter;             // 0.5s �����������仯�ĸ���
        uint8_t soc;                      // ����/���� 0-100% 5��ָʾΪ 90,70,50,30,20���ٷֱȣ������Ӧ�ĵ�ѹ����Ϊ 47V��46V,44.5V,43V,41V)��4 ��ָʾΪ 90,70,50,30
        uint8_t voltageSystem;            // ��ѹϵͳ  0x01:36V  0x02:48V  0x04:60V  0x08:64V  0x10:72V  0x20:80V  0x40:84V   0x80:96V
    } carinfo_sif_t;

    typedef struct
    {
        uint8_t SideStand;                // ���Ŷϵ���        0:��������     1:���ŷ���
        uint8_t BootGuard;                // ��������                0:�Ǳ���          1:������
        uint8_t hallFault;                // ��������(���) 0:����            1:����
        uint8_t throttleFault;            // ת�ѹ���
        uint8_t controllerFault;          // ����������
        uint8_t lowVoltageProtection;     // Ƿѹ����
        uint8_t cruise;                   // Ѳ��ָʾ��
        uint8_t assist;                   // ����ָʾ��
        uint8_t motorFault;               // �������
        uint8_t gear;                     // ��λ//0~7
        uint8_t motorRunning;             // �������                 1����
        uint8_t brake;                    // ɲ��
        uint8_t controllerProtection;     // ����������
        uint8_t coastCharging;            // ���г��
        uint8_t antiSpeedProtection;      // ���ɳ�����
        uint8_t seventyPercentCurrent;    // 70%����
        uint8_t pushToTalk;               // ����һ��ͨ
        uint8_t ekkBackupPower;           // ����EKK���õ�Դ
        uint8_t overCurrentProtection;    // ��������
        uint8_t motorShaftLockProtection; // ��ת����
        uint8_t reverse;                  // ����
        uint8_t electronicBrake;          // ����ɲ��
        uint8_t speedLimit;               // ����
        uint32_t current;                 // ���� ��λ��0.1A
        uint32_t voltage;                 // ��ѹ ��λ��0.1V
        uint8_t voltageSystem;            // ��ѹϵͳ  0x01:36V  0x02:48V  0x04:60V  0x08:64V  0x10:72V  0x20:80V  0x40:84V   0x80:96V
        uint8_t soc;                      // ����5 ��ָʾΪ 90,70,50,30,20���ٷֱȣ������Ӧ�ĵ�ѹ����Ϊ 47V��46V,44.5V,43V,41V)
        uint32_t speed;                   // ���� ��λ��0.1km/h
        uint32_t speed_real;              // ʵ�ʳ��� ��λ��0.1km/h
        uint32_t rpm;                     // ת�� ��λ��rpm
    } carinfo_t;

#pragma pack(push, 1)
    typedef struct
    {
        uint16_t current;      // ���� ��λ��0.1A
        uint16_t voltage;      // ��ѹ ��λ��0.1V
        uint8_t soc;           // ����/���� 0-100% 5��ָʾΪ 90,70,50,30,20���ٷֱȣ������Ӧ�ĵ�ѹ����Ϊ 47V��46V,44.5V,43V,41V)��4 ��ָʾΪ 90,70,50,30
        uint16_t speed;        // ���� ��λ��0.1km/h
        uint16_t speed_real;   // ʵ�ʳ��� ��λ��0.1km/h
        uint16_t rpm;          // ת�� ��λ��rpm offset:-20000
        uint8_t voltageSystem; // ��ѹϵͳ  0x01:36V  0x02:48V  0x04:60V  0x08:64V  0x10:72V  0x20:80V  0x40:84V   0x80:96V
    } carinfo_meter_t;

#pragma pack(push, 1)
    typedef struct
    {
        uint8_t highBeam;    // Զ���
        uint8_t lowBeam;     // �����
        uint8_t position;    // λ�õ�
        uint8_t frontFog;    // ǰ����
        uint8_t rearFog;     // ������
        uint8_t leftTurn;    // ��ת��
        uint8_t rightTurn;   // ��ת��
        uint8_t ready;       // Ready��
        uint8_t charge;      // ��س�ŵ��
        uint8_t parking;     // פ����
        uint8_t ecuFault;    // ECU���ϵ�
        uint8_t sensorFault; // ���������ϵ�
        uint8_t motorFault;  // ������ϵ�
        uint8_t bt;          // ����ָʾ��
        uint8_t wifi;        // WIFIָʾ��

    } carinfo_indicator_t;

    typedef struct
    {
        uint16_t avgEnergyConsumption; // ƽ���ܺ�
        uint16_t travelTime;           // ��ʻ��ʱ��,��λ����
        uint16_t avgSpeed;             // ƽ���ٶȣ���λ0.1km/h
    } carinfo_drivinfo_form_t;

    typedef enum
    {
        DRIVINFO_GEAR_UNKNOWN = 0x00,  // δ֪
        DRIVINFO_GEAR_MANUAL_1 = 0x01, // �ֶ�1��
        DRIVINFO_GEAR_MANUAL_2 = 0x02, // �ֶ�2��
        DRIVINFO_GEAR_MANUAL_3 = 0x03, // �ֶ�3��
        DRIVINFO_GEAR_MANUAL_4 = 0x04, // �ֶ�4��
        DRIVINFO_GEAR_MANUAL_5 = 0x05, // �ֶ�5��
        DRIVINFO_GEAR_MANUAL_6 = 0x06, // �ֶ�6��
        DRIVINFO_GEAR_MANUAL_7 = 0x07, // �ֶ�7��

        DRIVINFO_GEAR_REVERSE = 0x0A, //(R)������
        DRIVINFO_GEAR_PARK = 0x0B,    //(P)פ����
        DRIVINFO_GEAR_NEUTRAL = 0x0C, //(N)�յ�
        DRIVINFO_GEAR_DRIVE = 0x0D,   //(D)ǰ����
        DRIVINFO_GEAR_SPORT = 0x0E,   //(S)�˶���
        DRIVINFO_GEAR_FAULT = 0x0F,   //(F)����
    } carinfo_drivinfo_gear_t;

    typedef enum
    {
        DRIVINFO_DRIVEMODE_Comfort = 0x00,  // Comfort
        DRIVINFO_DRIVEMODE_ECO = 0x01,      // ECO
        DRIVINFO_DRIVEMODE_AUTO = 0x02,     // AUTO
        DRIVINFO_DRIVEMODE_Sport = 0x03,    // Sport
        DRIVINFO_DRIVEMODE_SNOW = 0x04,     // SNOW
        DRIVINFO_DRIVEMODE_OFF_ROAD = 0x05, // OFF ROAD
    } carinfo_drivinfo_drivemode_t;

#pragma pack(push, 1)
    typedef struct
    {
        uint32_t odo;                  // ����̣���λ0.1km
        uint16_t tripA;                // С�����A����λ0.1km
        uint16_t tripB;                // С�����B����λ0.1km
        uint8_t energyType;            // ��Դ����  0x00:ȼ��(0.L/100KM)  0x01����(0.1kWh/100KM)
        uint16_t enduranceMileage;     // �������
        uint16_t insEnergyConsumption; // ˲ʱ�ܺ�

        carinfo_drivinfo_form_t odoForm;   // ���ͳ����Ϣ����
        carinfo_drivinfo_form_t tripAForm; // С�����Aͳ����Ϣ
        carinfo_drivinfo_form_t tripBForm; // С�����Bͳ����Ϣ

        carinfo_drivinfo_gear_t gear;           // ��λ��Ϣ
        carinfo_drivinfo_drivemode_t driveMode; // ��ʻģʽ
    } carinfo_drivinfo_t;

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

    /// void app_carinfo_on_enter_run(void);

    /// void app_carinfo_on_exit_post_run(void);

    uint16_t app_carinfo_getSpeed(void);
    carinfo_meter_t *app_carinfo_get_meter_info(void);
    carinfo_indicator_t *app_carinfo_get_indicator_info(void);
    carinfo_drivinfo_t *app_carinfo_get_drivinfo_info(void);

    /**
     * end of group APP_SETTING
     * @}
     */

#ifdef __cplusplus
}
#endif

#endif
