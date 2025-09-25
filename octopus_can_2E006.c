
/*****************************************************************************/
#include "octopus_can_2E006.h"
#include "octopus_platform.h"
#include "octopus_tickcounter.h" // Tick Counter: provides timing and delay utilities
#include "octopus_message.h"     // Message IDs: defines identifiers for inter-task communication
#include "octopus_msgqueue.h"    // Message Queue: API for sending/receiving messages between tasks
#include "octopus_uart_ptl.h"    // UART Protocol Layer: handles protocol-level UART operations
#include "octopus_uart_upf.h"

#ifdef CUSTOMER_MODEL_CA_500

static BMS_Task_H_001_t g_bms1;
static BMS_Task_H_002_t g_bms2;

static MCU_Task_H_001_t g_mcu1;
static MCU_Task_H_002_t g_mcu2;

static VCU_Task_L_001_t g_vcu1;
static VCU_Task_L_002_t g_vcu2;

static ABS_Task_L_001_t g_abs;
static EBS_Task_L_001_t g_ebs;

static Charger_Task_L_001_t g_chg1;
static Charger_Task_L_002_t g_chg2;

static void CAN_update_vehicle_infor(const CanQueueMsg_t *msg);
static float Calculate_Speed_From_RPM(uint32_t rpm, uint16_t tire_radius_mm, uint8_t gear_ratio);
/**
 * @brief Dispatch a CAN message to appropriate handler based on its ID.
 *        This function may be used for routing logic in receive ISR or task.
 * @param msg Pointer to a CAN_Message_t structure.
 */
bool can_message_dispatcher(const CanQueueMsg_t *queue_msg)
{
    // LOG_BUFF_LEVEL((uint8_t *)queue_msg, sizeof(CanQueueMsg_t));
    bool updated = false;

    switch (queue_msg->std_id)
    {
    case CAN_ID_BMS_TASK_H_001:
        CAN_Parse_BMS_Task_H_001(queue_msg);
        break;
    case CAN_ID_BMS_TASK_H_002:
        CAN_Parse_BMS_Task_H_002(queue_msg);
        break;

    case CAN_ID_VCU_TASK_L_001:
        CAN_Parse_VCU_Task_L_001(queue_msg);
        break;
    case CAN_ID_VCU_TASK_L_002:
        CAN_Parse_VCU_Task_L_002(queue_msg);
        break;

    case CAN_ID_MCU_TASK_H_001:
        updated = CAN_Parse_MCU_Task_H_001(queue_msg);
        if (updated)
        {
            //LOG_LEVEL("g_mcu1.motor_speed_rpm: %d\n", g_mcu1.motor_speed_rpm);
            //LOG_LEVEL("g_mcu1.info           : %d\n", g_mcu1.info.raw);
            //LOG_LEVEL("g_mcu1.fault_state    : %d\n", g_mcu1.fault_state.raw);
            CAN_update_vehicle_infor(queue_msg);
        }

        break;
    case CAN_ID_MCU_TASK_H_002:
        updated = CAN_Parse_MCU_Task_H_002(queue_msg);
        if (updated)
        {
            CAN_update_vehicle_infor(queue_msg);
        }
        break;

    case CAN_ID_CHARGER_TASK_L_001:
        CAN_Parse_Charger_Task_L_001(queue_msg);
        break;
    case CAN_ID_CHARGER_TASK_L_002:
        CAN_Parse_Charger_Task_L_002(queue_msg);
        break;

    case CAN_ID_ABS_TASK_L_001:
        CAN_Parse_ABS_Task_L_001(queue_msg);
        break;
    case CAN_ID_EBS_LEVEL:
        CAN_Parse_EBS_Level(queue_msg);
        break;

    default:
        break; // unknown ID
    }
		
		return updated;
}

bool can_message_sender(const uint16_t message_id)
{
    switch (message_id)
    {
    // ===== BMS =====
    case CAN_ID_BMS_TASK_H_001:
        CAN_SEND_STRUCT_STD(CAN_ID_BMS_TASK_H_001, &g_bms1);
        break;
    case CAN_ID_BMS_TASK_H_002:
        CAN_SEND_STRUCT_STD(CAN_ID_BMS_TASK_H_002, &g_bms2);
        break;

    // ===== VCU =====
    case CAN_ID_VCU_TASK_L_001:
        CAN_SEND_STRUCT_STD(CAN_ID_VCU_TASK_L_001, &g_vcu1);
        break;
    case CAN_ID_VCU_TASK_L_002:
        CAN_SEND_STRUCT_STD(CAN_ID_VCU_TASK_L_002, &g_vcu2);
        break;

    // ===== MCU =====
    case CAN_ID_MCU_TASK_H_001:
        CAN_SEND_STRUCT_STD(CAN_ID_MCU_TASK_H_001, &g_mcu1);
        break;
    case CAN_ID_MCU_TASK_H_002:
        CAN_SEND_STRUCT_STD(CAN_ID_MCU_TASK_H_002, &g_mcu2);
        break;

    // ===== Charger =====
    case CAN_ID_CHARGER_TASK_L_001:
        CAN_SEND_STRUCT_STD(CAN_ID_CHARGER_TASK_L_001, &g_chg1);
        break;
    case CAN_ID_CHARGER_TASK_L_002:
        CAN_SEND_STRUCT_STD(CAN_ID_CHARGER_TASK_L_002, &g_chg2);
        break;

    // ===== ABS =====
    case CAN_ID_ABS_TASK_L_001:
        CAN_SEND_STRUCT_STD(CAN_ID_ABS_TASK_L_001, &g_abs);
        break;

    // ===== EBS =====
    case CAN_ID_EBS_LEVEL:
        CAN_SEND_STRUCT_STD(CAN_ID_EBS_LEVEL, &g_ebs);
        break;

    default:
        // 未知消息ID，可添加错误处理或日志
        break;
    }
		
		return false;
}

// -------- BMS --------
void CAN_Parse_BMS_Task_H_001(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;
    g_bms1.soc_percent = msg->data[0];
    g_bms1.max_discharge_A = (msg->data[1] | (msg->data[2] << 8)) * 0.1f;
    g_bms1.remain_capacity_mAh = (msg->data[3] | (msg->data[4] << 8));
    g_bms1.realtime_current_A = ((msg->data[5] | (msg->data[6] << 8)) * 0.1f) - 55.0f;
    g_bms1.fault_code = msg->data[7];
}

void CAN_Parse_BMS_Task_H_002(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;
    g_bms2.voltage_V = (msg->data[0] | (msg->data[1] << 8)) * 0.1f;
    g_bms2.current_A = ((msg->data[2] | (msg->data[3] << 8)) * 0.1f) - 400.0f;
    g_bms2.temp_min_C = (int8_t)msg->data[4];
    g_bms2.temp_max_C = (int8_t)msg->data[5];
}

// -------- VCU --------
void CAN_Parse_VCU_Task_L_001(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;
    g_vcu1.gear = msg->data[0];
    g_vcu1.work_mode = msg->data[1];
    g_vcu1.state_bits = msg->data[2];
}

void CAN_Parse_VCU_Task_L_002(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;
    g_vcu2.power_limit = msg->data[0];
    g_vcu2.regen_level = msg->data[1];
    g_vcu2.alive_counter = msg->data[2];
}

// -------- MCU --------
bool CAN_Parse_MCU_Task_H_001(const CanQueueMsg_t *msg)
{
    bool b_ret = false;
	  uint16_t motor_speed_rpm; 
	  MCU_Task_H_001_t g_mcu1_new;
	
    if (msg->data_len != 8)
        return false;

		motor_speed_rpm = (uint16_t)(msg->data[0] | (msg->data[1] << 8));
		//if(motor_speed_rpm <= 0) motor_speed_rpm = 0;		
    g_mcu1_new.motor_speed_rpm = motor_speed_rpm - 8000;
		
		//LOG_LEVEL("g_mcu1_new.motor_speed_rpm:%d, motor_speed_rpm:%d\n", g_mcu1_new.motor_speed_rpm, motor_speed_rpm);
		
    g_mcu1_new.info.raw = (uint32_t)(msg->data[2] | (msg->data[3] << 8) | (msg->data[4] << 16) | (msg->data[5] << 24));
    g_mcu1_new.fault_state.raw = (uint16_t)(msg->data[6] | (msg->data[7] << 8));

    if (g_mcu1_new.motor_speed_rpm != g_mcu1.motor_speed_rpm)
        b_ret = true;

    if (g_mcu1_new.info.raw != g_mcu1.info.raw)
        b_ret = true;

    if (g_mcu1_new.fault_state.raw != g_mcu1.fault_state.raw)
        b_ret = true;

    if (b_ret)
    {
        g_mcu1 = g_mcu1_new;
    }

    return b_ret;
}

bool CAN_Parse_MCU_Task_H_002(const CanQueueMsg_t *msg)
{
    bool b_ret = false;
    if (msg == NULL)
        return false;
    if (msg->std_id != 0x171)
        return false;
    if (msg->data_len != 8)
        return false;

    MCU_Task_H_002_t new_frame;

    /* 1) MCU 温度 (Byte0) */
    new_frame.mcu_temp = (int8_t)msg->data[0] - 40;

    /* 2) Motor 温度 (Byte1) */
    new_frame.motor_temp = (int8_t)msg->data[1] - 40;

    /* 3) Bus Current (Byte2-3, 小端, 0.1A/bit, 偏移 -1950) */
    uint16_t raw_bus_cur = (uint16_t)(msg->data[2] | ((msg->data[3] & 0x07) << 8));
    new_frame.bus_current = (int16_t)raw_bus_cur - 97;

    /* 4) Phase Current (Byte4-5, 小端, 0.1A/bit, 偏移 -4000) */
    uint16_t raw_phase_cur = (uint16_t)(msg->data[4] << 5 | (msg->data[3] & 0x01F));
    new_frame.phase_current = (int16_t)raw_phase_cur - 4000;

    /* 5) Bus Voltage (Byte6, 1V/bit) */
    new_frame.bus_voltage = msg->data[5];

    /* 6) Throttle Voltage (Byte7, 0.5V/bit) */
    new_frame.throttle_volt = msg->data[6];

    if (new_frame.mcu_temp != g_mcu2.mcu_temp)
        b_ret = true;

    if (new_frame.motor_temp != g_mcu2.motor_temp)
        b_ret = true;

    if (new_frame.bus_current != g_mcu2.bus_current)
        b_ret = true;

    if (new_frame.phase_current != g_mcu2.phase_current)
        b_ret = true;

    if (new_frame.bus_voltage != g_mcu2.bus_voltage)
        b_ret = true;

    if (new_frame.throttle_volt != g_mcu2.throttle_volt)
        b_ret = true;

    if (b_ret)
    {
        /* 更新全局变量 */
        g_mcu2 = new_frame;
    }

    return b_ret;
}

// -------- Charger --------
void CAN_Parse_Charger_Task_L_001(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;
    g_chg1.output_voltage_V =
        (msg->data[0] | (msg->data[1] << 8)) * 0.01f;
    g_chg1.output_current_A =
        (msg->data[2] | (msg->data[3] << 8)) * 0.05f;
    g_chg1.temp_C = (int8_t)msg->data[4] - 40;
    g_chg1.alive_counter = msg->data[5];
}

void CAN_Parse_Charger_Task_L_002(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;
    g_chg2.charger_state = msg->data[0];
    g_chg2.fault_code = msg->data[1];
    g_chg2.alive_counter = msg->data[2];
}

// -------- ABS / EBS --------
void CAN_Parse_ABS_Task_L_001(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;

    g_abs.abs_state = msg->data[0];
    g_abs.front_speed_rpm = msg->data[1] | (msg->data[2] << 8);
    g_abs.rear_speed_rpm = msg->data[3] | (msg->data[4] << 8);
    g_abs.dtc = msg->data[5];
}

void CAN_Parse_EBS_Level(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;
    g_ebs.ebs_state = msg->data[0];
}

uint16_t get_wheel_radius_mm(void)
{
    if (lt_carinfo_meter.wheel_diameter >= SETTING_WHEEL_MAX)
    {
		   return lt_carinfo_meter.wheel_diameter * 25.4;
    }
		
    switch (lt_carinfo_meter.wheel_diameter)
    {
    case SETTING_WHEEL_16_Inch:
        return 203;
    case SETTING_WHEEL_18_Inch:
        return 229;
    case SETTING_WHEEL_20_Inch:
        return 254;
    case SETTING_WHEEL_22_Inch:
        return 279;
    case SETTING_WHEEL_24_Inch:
        return 305;
    case SETTING_WHEEL_26_Inch:
        return 330;
    case SETTING_WHEEL_27_Inch:
        return 343;
    case SETTING_WHEEL_27_5_Inch:
        return 349;
    case SETTING_WHEEL_28_Inch:
        return 356;
    case SETTING_WHEEL_29_Inch:
        return 368;
    }
    return 1000;
}
// ERROR_CODE_IDLE = 0X00,                                      // 无动作
// ERROR_CODE_NORMAL = 0X01,                                    // 正常状态
// ERROR_CODE_BRAKE = 0X03,                                     // 已刹车
// ERROR_CODE_THROTTLE_NOT_ZERO = 0X04,                         // 转把没有归位（停在高位处）
// ERROR_CODE_THROTTLE_HALLSENSOR_ABNORMALITY = 0X05,           // 转把故障
// ERROR_CODE_LOW_VOLTAGE_PROTECTION = 0X06,                    // 低电压保护
// ERROR_CODE_OVER_VOLTAGE_PROTECTION = 0X07,                   // 过电压保护
// ERROR_CODE_HALLSENSOR_ABNORMALITY = 0X08,                    // 电机霍尔信号线故障
// ERROR_CODE_MOTOR_ABNORMALITY = 0X09,                         // 电机相线故障
// ERROR_CODE_CONTROLLER_OVERHEAT = 0X10,                       // 控制器温度高已达到保护点
// ERROR_CODE_CONTROLLER_TEMPERATURE_SENSOR_ABNORMALITY = 0X11, // 控制器温度传感器故障
// ERROR_CODE_CURRENT_SENSOR_ABNORMALITY = 0X12,                // 电流传感器故障
// ERROR_CODE_BATTERY_OVERHEAT = 0X13,                          // 电池内温度故障
// ERROR_CODE_MOTOR_TEMPERATURE_SENSOR_ABNORMALITY = 0X14,      // 电机内温度传感器故障
// ERROR_CODE_CONTROLLER_ABNORMALITY = 0X15,                    // 控制器故障
// ERROR_CODE_ASSIST_POWER_SENSOR_ABNORMALITY = 0X16,           // 助力传感器故障
// ERROR_CODE_SPEED_SENSOR_ABNORMALITY = 0X21,                  // 速度传感器故障
// ERROR_CODE_BMS_ABNORMALITY = 0X22,                           // BMS通讯故障
// ERROR_CODE_LAMP_ABNORMALITY = 0X23,                          // 大灯故障
// ERROR_CODE_LAMP_SENSOR_ABNORMALITY = 0X24,                   // 大灯传感器故障
// ERROR_CODE_COMMUNICATION_ABNORMALITY = 0X30,                 // 通讯故障
void CAN_update_vehicle_infor(const CanQueueMsg_t *msg)
{
    switch (msg->std_id)
    {
    case CAN_ID_BMS_TASK_H_001:
        break;
    case CAN_ID_BMS_TASK_H_002:
        break;
    case CAN_ID_VCU_TASK_L_001:
        break;
    case CAN_ID_VCU_TASK_L_002:
        break;

    case CAN_ID_MCU_TASK_H_001:
        lt_carinfo_meter.rpm = g_mcu1.motor_speed_rpm;
        lt_carinfo_meter.speed_actual = Calculate_Speed_From_RPM(g_mcu1.motor_speed_rpm, get_wheel_radius_mm(), 1);
        lt_carinfo_meter.gear = g_mcu1.info.bits.forwardGear;
        lt_carinfo_indicator.brake = g_mcu1.info.bits.brake;
        lt_carinfo_indicator.cruise_control = g_mcu1.info.bits.cruise;
        lt_carinfo_battery.rel_charge_state = g_mcu1.info.bits.charge;
        lt_carinfo_indicator.drive_mode = g_mcu1.info.bits.workMode;

        carinfo_add_error_code(ERROR_CODE_CURRENT_SENSOR_ABNORMALITY, g_mcu1.fault_state.bits.overCurrent, false);
        carinfo_add_error_code(ERROR_CODE_OVER_VOLTAGE_PROTECTION, g_mcu1.fault_state.bits.overVoltage, false);
        carinfo_add_error_code(ERROR_CODE_LOW_VOLTAGE_PROTECTION, g_mcu1.fault_state.bits.underVoltage, false);

        carinfo_add_error_code(ERROR_CODE_THROTTLE_HALLSENSOR_ABNORMALITY, g_mcu1.fault_state.bits.throttleFault, false);
        carinfo_add_error_code(ERROR_CODE_MOTOR_ABNORMALITY, g_mcu1.fault_state.bits.pLockFault, false);
        carinfo_add_error_code(ERROR_CODE_HALLSENSOR_ABNORMALITY, g_mcu1.fault_state.bits.motorHallFault, false);
        carinfo_add_error_code(ERROR_CODE_CONTROLLER_ABNORMALITY, g_mcu1.fault_state.bits.angleFault, false);
        carinfo_add_error_code(ERROR_CODE_MOTOR_TEMPERATURE_SENSOR_ABNORMALITY, g_mcu1.fault_state.bits.motorOverTemp1, false);
        carinfo_add_error_code(ERROR_CODE_MOTOR_TEMPERATURE_SENSOR_ABNORMALITY, g_mcu1.fault_state.bits.motorOverTemp2, true);
		
		    //LOG_LEVEL("lt_carinfo_meter.speed_actual: %d\n", lt_carinfo_meter.speed_actual);
		    send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_INDICATOR, FRAME_CMD_CARINFOR_INDICATOR);
				send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_METER, FRAME_CMD_CARINFOR_METER);
        break;

    case CAN_ID_MCU_TASK_H_002:
        lt_carinfo_battery.current = g_mcu2.bus_current * 10;
        lt_carinfo_battery.voltage = g_mcu2.bus_voltage * 10;
		    //LOG_LEVEL("lt_carinfo_battery.voltage=%d lt_carinfo_battery.current=%d\n", lt_carinfo_battery.voltage,lt_carinfo_battery.current);
        send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_BATTERY, FRAME_CMD_CARINFOR_BATTERY);
        break;

    case CAN_ID_CHARGER_TASK_L_001:
        break;
    case CAN_ID_CHARGER_TASK_L_002:
        break;
    case CAN_ID_ABS_TASK_L_001:
        break;
    case CAN_ID_EBS_LEVEL:
        break;
    default:
        break; // unknown ID
    }
}

/**
 * @brief  根据电机转速 (RPM)、轮胎直径和减速比，计算车辆的行驶速度 (km/h)。
 *
 * @param  rpm                电机转速，单位：RPM (revolutions per minute, 转/分钟)
 * @param  tire_diameter_mm   轮胎直径，单位：mm
 * @param  gear_ratio         减速比 (电机转速 : 车轮转速)，例如减速比=10 表示车轮转一圈电机转10圈
 *
 * @return 车速，单位：km/h
 *
 * 计算公式推导：
 * 1. 轮胎直径 D (米) = tire_diameter_mm / 1000
 * 2. 轮胎周长 C (米) = π * D
 * 3. 电机转速 rpm (转/分钟)，车轮转速 wheel_rpm = rpm / gear_ratio
 * 4. 车轮每分钟行驶距离 (米/分钟) = wheel_rpm * C
 * 5. 换算成 km/h：
 *        speed(km/h) = (wheel_rpm * C [米/分钟]) × (60 [分钟/小时]) / 1000 [米/公里]
 *                    = wheel_rpm * π * D * (60 / 1000)
 * 6. 再整理常数：
 *        (60 / 1000) = 0.06
 *
 * 因此最终公式：
 *        speed_kmh = wheel_rpm * π * D * 0.06
 */
float Calculate_Speed_From_RPM(uint32_t rpm, uint16_t tire_radius_mm, uint8_t gear_ratio)
{
	  //LOG_LEVEL("tire_diameter_mm: %d\n", tire_radius_mm);
	
    // 1. 轮胎直径 (m)
    double D_m = (tire_radius_mm*2) / 1000.0f;

    // 2. 车轮转速 (RPM)，由电机转速除以减速比得到
    double wheel_rpm = (float)rpm / (float)gear_ratio;

    // 3. 根据公式计算车速 (km/h)
    double speed_kmh = wheel_rpm * D_m * (float)PI_FLOAT * 0.06f;

    return speed_kmh * 10;
}

#endif
