
/*****************************************************************************/
#include "octopus_can_2E006.h"

static BMS_Task_H_001_t g_bms1;
static BMS_Task_H_002_t g_bms2;
static MCU_Task_H_001_t g_mcu1;
static MCU_Task_H_002_t g_mcu2;
static VCU_Task_L_001_t g_vcu1;
static VCU_Task_L_002_t g_vcu2;
static Charger_Task_L_001_t g_chg1;
static Charger_Task_L_002_t g_chg2;
static ABS_Task_L_001_t g_abs;
static EBS_Task_L_001_t g_ebs;

/**
 * @brief Dispatch a CAN message to appropriate handler based on its ID.
 *        This function may be used for routing logic in receive ISR or task.
 * @param msg Pointer to a CAN_Message_t structure.
 */
void can_message_dispatcher(const CanQueueMsg_t *queue_msg)
{
    LOG_BUFF_LEVEL((uint8_t *)queue_msg, sizeof(CanQueueMsg_t));
    switch (queue_msg->id)
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
        CAN_Parse_MCU_Task_H_001(queue_msg);
        break;
    case CAN_ID_MCU_TASK_H_002:
        CAN_Parse_MCU_Task_H_002(queue_msg);
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
}

void can_message_sender(const uint16_t message_id)
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
void CAN_Parse_MCU_Task_H_001(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;
    g_mcu1.motor_speed_rpm = msg->data[0] | (msg->data[1] << 8);
    g_mcu1.info = msg->data[2];
    g_mcu1.fault_state = msg->data[3];
}

void CAN_Parse_MCU_Task_H_002(const CanQueueMsg_t *msg)
{
    if (msg->data_len != 8)
        return;
    g_mcu2.bus_voltage_0p1V = (msg->data[0] | (msg->data[1] << 8)) * 0.1f;
    g_mcu2.bus_current_0p1A = (msg->data[2] | (msg->data[3] << 8)) * 0.1f;
    g_mcu2.mcu_temp_off40 = (int8_t)msg->data[4];
    g_mcu2.motor_temp_off40 = (int8_t)msg->data[5];
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
