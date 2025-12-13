/*******************************************************************************
 * FILE: octopus_car_controller.c
 *
 * DESCRIPTION:
 * This file contains the implementation of the car controller application
 * for the Octopus platform. It includes functions for handling communication
 * between various car system modules, such as the meter, indicator, and
 * drivetrain information. It also includes battery voltage retrieval and
 * system information frame (SIF) updates.
 *
 * MODULES:
 * - Meter module
 * - Indicator module
 * - Drivetrain info module
 * - Car controller message processing
 * - Battery voltage retrieval
 *
 * NOTE: This file is part of the Octopus car control system and interfaces
 * with the Octopus platform to manage the car's state and communication with
 * external modules.
 *
 * File Name: octopus_task_manager_platform.h
 * @version  1.0.0
 * @date     2024-12-11
 * @author   Octopus Team
 *
 ******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */
#include "octopus_vehicle.h"
#include "octopus_utils.h"
#include "octopus_sif.h"
#include "octopus_ipc.h"
#include "octopus_flash.h"
#include "octopus_uart_ptl.h"    // Include UART protocol header
#include "octopus_uart_upf.h"    // Include UART protocol header
#include "octopus_tickcounter.h" // Include tick counter for timing operations
#include "octopus_msgqueue.h"    // Include message queue header for task communication
#include "octopus_message.h"     // Include message id for inter-task communication
#include "octopus_dhf.h"

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */
/// #define CARINFOR_PTL_ACK
/// #define TEST_LOG_DEBUG_SIF  // Uncomment to enable debug logging for SIF module

/*******************************************************************************
 * MACROS
 */
// #define TEST_LOG_DEBUG_VEHICLE
// #define CAR_EXTEND_ERROR_CODE // CarErrorCodeFlags_t
/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * CONSTANTS
 */
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
/*******************************************************************************
 * LOCAL FUNCTIONS DECLARATION
 */
static bool meter_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool meter_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

// static bool indicator_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
// static bool indicator_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

// static bool drivinfo_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
// static bool drivinfo_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

static void task_car_controller_msg_handler(void); // Process messages related to car controller

/*******************************************************************************
 * GLOBAL VARIABLES
 */
static uint8_t car_error_code[ERROR_CODE_COUNT];
/*******************************************************************************
 * STATIC VARIABLES
 */

carinfo_meter_t lt_carinfo_meter = {0};         // Local meter data structure
carinfo_indicator_t lt_carinfo_indicator = {0}; // Local indicator data structure
carinfo_battery_t lt_carinfo_battery = {0};
carinfo_error_t lt_carinfo_error;
CarErrorCodeFlags_t car_error_code_flags; // CarErrorCodeFlags;
// static carinfo_drivinfo_t lt_drivinfo = {0};   // Local drivetrain information

// Timer variables
// static uint32_t l_t_msg_wait_meter_timer; // Timer for 10 ms message wait (not used currently)
static uint32_t l_t_msg_wait_50_timer;  // Timer for 50 ms message wait
static uint32_t l_t_msg_car_trip_timer; // Timer for 100 ms message wait

static uint32_t l_t_trip_saving_timer; // Timer for state of charge monitoring
static uint32_t l_t_chd_timer;         // communication heartbeat detection timer;
// static bool l_t_speed_changed = false;
// static bool l_t_gear_changed = false;
/*******************************************************************************
 * EXTERNAL VARIABLES
 */

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void task_vehicle_init_running(void)
{
    LOG_LEVEL("task_vehicle_init_running\r\n");
    ptl_register_module(MCU_TO_SOC_MOD_CARINFOR, meter_module_send_handler, meter_module_receive_handler);
    // ptl_register_module(MCU_TO_SOC_MOD_INDICATOR, indicator_module_send_handler, indicator_module_receive_handler);
    // ptl_register_module(MCU_TO_SOC_MOD_DRIV_INFO, drivinfo_module_send_handler, drivinfo_module_receive_handler);

    OTMS(TASK_MODULE_CAR_INFOR, OTMS_S_INVALID);
    lt_carinfo_meter.speed_actual = 0;
    lt_carinfo_meter.speed_max = 0;
    lt_carinfo_meter.speed_average = 0;
    lt_carinfo_indicator.width_lamp = 0;
    // lt_carinfo_meter.wheel_diameter = SETTING_WHEEL_27_Inch;
}

void task_vehicle_start_running(void)
{
    LOG_LEVEL("task_vehicle_start_running\r\n");
    OTMS(TASK_MODULE_CAR_INFOR, OTMS_S_ASSERT_RUN);
#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    lt_carinfo_indicator.ready = 0; // ready flag
    lt_carinfo_meter.trip_distance = 0;
    lt_carinfo_meter.trip_time = 0;
#endif
}

void task_vehicle_assert_running(void)
{
    ptl_reqest_running(MCU_TO_SOC_MOD_CARINFOR);
    // ptl_reqest_running(MCU_TO_SOC_MOD_INDICATOR);
    // ptl_reqest_running(MCU_TO_SOC_MOD_DRIV_INFO);
    StartTickCounter(&l_t_msg_wait_50_timer);
    // StartTickCounter(&l_t_msg_wait_100_timer);
    StartTickCounter(&l_t_trip_saving_timer);
    StartTickCounter(&l_t_chd_timer);
    OTMS(TASK_MODULE_CAR_INFOR, OTMS_S_RUNNING);
}

void task_vehicle_running(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_SIF
    sif_controller_updating();
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    // if (GetTickCounter(&l_t_msg_wait_50_timer) < 10)
    //     return;
    // StartTickCounter(&l_t_msg_wait_50_timer);
    task_car_controller_msg_handler();
#endif
}

void task_vehicle_post_running(void)
{
    ptl_release_running(MCU_TO_SOC_MOD_CARINFOR);
    OTMS(TASK_MODULE_CAR_INFOR, OTMS_S_ASSERT_RUN);
}

void task_vehicle_stop_running(void)
{
    LOG_LEVEL("_stop_running\r\n");
    OTMS(TASK_MODULE_CAR_INFOR, OTMS_S_INVALID);
}
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * LOCAL FUNCTIONS IMPLEMENTATION
 */
// This function handles the sending of METER module data through the protocol layer.
// Depending on the frame type and command, it constructs appropriate data and fills the buffer for transmission.
bool meter_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    MY_ASSERT(buff);
    // uint8_t tmp[128] = {0};

    if (MCU_TO_SOC_MOD_CARINFOR == frame_type)
    {
        switch (param1)
        {
        case FRAME_CMD_CARINFOR_INDICATOR:
#ifdef TEST_LOG_DEBUG_VEHICLE
            LOG_LEVEL("lt_carinfo_indicator.ready=%d\r\n", lt_carinfo_indicator.ready);
#endif
            ptl_build_frame(MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_INDICATOR, (uint8_t *)&lt_carinfo_indicator, sizeof(carinfo_indicator_t), buff);
            return true;

        case FRAME_CMD_CARINFOR_METER:
#ifdef TEST_LOG_DEBUG_VEHICLE
            LOG_LEVEL("lt_meter.speed_actual=%d\r\n", lt_carinfo_meter.speed_actual);
#endif
            ptl_build_frame(MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_METER, (uint8_t *)&lt_carinfo_meter, sizeof(carinfo_meter_t), buff);
            return true;

        case FRAME_CMD_CARINFOR_BATTERY:
#ifdef TEST_LOG_DEBUG_VEHICLE
            LOG_LEVEL("lt_carinfo_battery.voltage=%d lt_carinfo_battery.current=%d\n", lt_carinfo_battery.voltage, lt_carinfo_battery.current);
#endif
            ptl_build_frame(MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_BATTERY, (uint8_t *)&lt_carinfo_battery, sizeof(carinfo_battery_t), buff);
            return true;

        case FRAME_CMD_CARINFOR_ERROR:
            // memcpy(tmp, &lt_carinfo_error, sizeof(carinfo_error_t));
            // LOG_LEVEL("(uint8_t *)&lt_carinfo_error size=%d\r\n", sizeof(carinfo_error_t));
#ifdef TEST_LOG_DEBUG_VEHICLE
            LOG_LEVEL("lt_carinfo_error.fault_battery=%d\r\n", lt_carinfo_error.fault_battery);
#endif

#ifdef CAR_EXTEND_ERROR_CODE
            if (param2)
                ptl_build_frame(MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, (uint8_t *)&car_error_code_flags, sizeof(CarErrorCodeFlags_t), buff);
            else
#endif
                ptl_build_frame(MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, (uint8_t *)&lt_carinfo_error, sizeof(carinfo_error_t), buff);

            return true;
        default:
            break;
        }
    }
    return false; // Unsupported frame or command
}

// This function handles reception of METER module data from protocol layer.
// It parses incoming payload and updates the corresponding meter information.
bool meter_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    MY_ASSERT(payload); // Ensure the payload pointer is valid
    MY_ASSERT(ackbuff); // Ensure the ack buffer pointer is valid

    // Handle M2A (Module to Application) frames — actual data updates
    // LOG_LEVEL("payload->frame_type=%d payload->data_len=%d\r\n",payload->frame_type, payload->data_len);
    if (MCU_TO_SOC_MOD_CARINFOR == payload->frame_type)
    {
        switch (payload->frame_cmd)
        {
        case FRAME_CMD_CARINFOR_INDICATOR:
            if (payload->data_len == sizeof(carinfo_indicator_t))
            {
                memcpy(&lt_carinfo_indicator, payload->data, payload->data_len);
                send_message(TASK_MODULE_IPC, MSG_OTSM_DEVICE_CAR_EVENT, MSG_IPC_CMD_CAR_GET_INDICATOR_INFO, 0);
            }
            else
            {
                LOG_LEVEL("wrong indicator data payload->data_len=%d\r\n", payload->data_len);
            }
            break;

        case FRAME_CMD_CARINFOR_METER:
            if (payload->data_len == sizeof(carinfo_meter_t))
            {
                memcpy(&lt_carinfo_meter, payload->data, payload->data_len);
                send_message(TASK_MODULE_IPC, MSG_OTSM_DEVICE_CAR_EVENT, MSG_IPC_CMD_CAR_GET_METER_INFO, 0);
            }
            else
            {
                LOG_LEVEL("wrong meter data payload->data_len=%d\r\n", payload->data_len);
            }
            break;

        case FRAME_CMD_CARINFOR_BATTERY:
            if (payload->data_len == sizeof(carinfo_battery_t))
            {
                memcpy(&lt_carinfo_battery, payload->data, payload->data_len);
                send_message(TASK_MODULE_IPC, MSG_OTSM_DEVICE_CAR_EVENT, MSG_IPC_CMD_CAR_GET_BATTERY_INFO, 0);
            }
            else
            {
                LOG_LEVEL("wrong battery data payload->data_len=%d battery size=%d\r\n", payload->data_len, sizeof(carinfo_battery_t));
            }
            break;
        case FRAME_CMD_CARINFOR_ERROR:
            if (payload->data_len == sizeof(carinfo_error_t))
            {
                memcpy(&lt_carinfo_error, payload->data, payload->data_len);
                send_message(TASK_MODULE_IPC, MSG_OTSM_DEVICE_CAR_EVENT, MSG_IPC_CMD_CAR_GET_ERROR_INFO, 0);
            }
            else
            {
                LOG_LEVEL("wrong error status data payload->data_len=%d sizeof(carinfo_error_t)=%d\r\n", payload->data_len, sizeof(carinfo_error_t));
            }
            break;
        default:
            break;
        }
    }

    return false; // Unsupported frame type or command
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void task_car_reset_trip(void)
{
    lt_carinfo_meter.trip_distance = 0;
    lt_carinfo_meter.trip_time = 0;
}

void battary_update_simulate_infor(void)
{
    lt_carinfo_battery.reserve2 = adc_get_value_v();
#if 1
    calculate_battery_soc_voltage_only(lt_carinfo_battery.voltage * 100,
                                       (lt_carinfo_battery.reserve2 * 100),
                                       lt_carinfo_battery.current * 100,
                                       system_meter_infor.trip_odo,
                                       DEFAULT_CONSUMPTION_WH_PER_KM, DEFAULT_SAFETY_RESERVE_RATIO,
                                       lt_carinfo_meter.speed_average,
                                       &lt_carinfo_battery.power, &lt_carinfo_battery.soc,
                                       &lt_carinfo_battery.range, &lt_carinfo_battery.range_max);
#endif
}

void task_car_controller_msg_handler(void)
{
    // static uint32_t calculate_count = 0;
    uint32_t trip_timer = 0;
    uint32_t trip_saving_timer = 0;
    uint32_t delta_distance = 0;

    if ((lt_carinfo_meter.speed_actual > 0))
    {
        if (!IsTickCounterStart(&l_t_msg_car_trip_timer))
            StartTickCounter(&l_t_msg_car_trip_timer);
    }
    else
    {
        StopTickCounter(&l_t_msg_car_trip_timer);
    }

    Msg_t *msg = get_message(TASK_MODULE_CAR_INFOR);
    if (msg->msg_id == NO_MSG)
    {
        trip_timer = GetTickCounter(&l_t_msg_car_trip_timer);
        trip_saving_timer = GetTickCounter(&l_t_trip_saving_timer);
        if (trip_timer > 2000)
        {
            if (lt_carinfo_meter.speed_actual > lt_carinfo_meter.speed_max)
                lt_carinfo_meter.speed_max = lt_carinfo_meter.speed_actual;
            if (lt_carinfo_meter.speed_average == 0)
                lt_carinfo_meter.speed_average = lt_carinfo_meter.speed_actual;

            lt_carinfo_meter.speed_average = (lt_carinfo_meter.speed_average + lt_carinfo_meter.speed_actual) / 2;

            trip_timer = trip_timer / 1000;
            delta_distance = calculateTotalDistance(lt_carinfo_meter.speed_actual, trip_timer);

            lt_carinfo_meter.trip_time = lt_carinfo_meter.trip_time + trip_timer;
            lt_carinfo_meter.trip_distance = lt_carinfo_meter.trip_distance + delta_distance;
            lt_carinfo_meter.trip_odo = lt_carinfo_meter.trip_odo + delta_distance;

            system_meter_infor.trip_odo = system_meter_infor.trip_odo + delta_distance;
            system_meter_infor.speed_average = lt_carinfo_meter.speed_average;

            battary_update_simulate_infor();
            RestartTickCounter(&l_t_msg_car_trip_timer);
        }

        if (trip_saving_timer > 60000 * 1 && delta_distance > 0)
        {
            flash_save_carinfor_meter();

            RestartTickCounter(&l_t_trip_saving_timer);
        }

        // if (lt_carinfo_battery.soc == 0 || trip_saving_timer % 60000 == 0)
        if (trip_saving_timer % 3000 == 0)
        {
            battary_update_simulate_infor();
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_BATTERY, 0);
        }

        if (GetTickCounter(&l_t_chd_timer) > 1000 * 10)
        {
            // lt_carinfo_indicator.ready = 0;
            StartTickCounter(&l_t_chd_timer);
        }
        return;
    }

    if (MCU_TO_SOC_MOD_CARINFOR == msg->msg_id)
    {
        switch (msg->param1)
        {
        case FRAME_CMD_CARINFOR_INDICATOR:
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_INDICATOR, 0);
            break;
        case FRAME_CMD_CARINFOR_METER:
            // LOG_LEVEL("lt_carinfo_meter.trip_odo=%d\r\n", lt_carinfo_meter.trip_odo);
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_METER, 0);
            break;
        case FRAME_CMD_CARINFOR_BATTERY:
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_BATTERY, 0);
            break;
        case FRAME_CMD_CARINFOR_ERROR:
            send_message(TASK_MODULE_PTL_1, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, 0);
            break;
        default:
            break;
        }
    }

    else if (MSG_OTSM_DEVICE_GPIO_EVENT == msg->msg_id)
    {
    }
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
//  添加错误代码

void carinfo_add_error_code(ERROR_CODE error_code, bool code_append, bool update_immediately)
{
    bool update_send = false;
    if (code_append)
    {
#ifdef CAR_EXTEND_ERROR_CODE
        if (!CAR_INFOR_CHECK_ERROR(car_error_code_flags, error_code))
        {
            CAR_INFOR_SET_ERROR(car_error_code_flags, error_code);
            update_send = true;
        }
#else
        if (error_code != car_error_code[0])
        {
            if (error_code >= ERROR_CODE_BEGIN && error_code <= ERROR_CODE_END)
            {
                // 历史故障信息顺位下移
                for (int i = 0; i < ERROR_CODE_COUNT - 1; i++)
                {
                    car_error_code[i + 1] = car_error_code[i];
                }
                // 插入置顶的最新故障信息
                car_error_code[0] = error_code;
            }
        }
#endif
    }
    else
    {
#ifdef CAR_EXTEND_ERROR_CODE
        if (CAR_INFOR_CHECK_ERROR(car_error_code_flags, error_code))
        {
            CAR_INFOR_CLEAR_ERROR(car_error_code_flags, error_code);
            update_send = true;
        }
#else
        if (error_code == car_error_code[0])
        {
            if (error_code >= ERROR_CODE_BEGIN && error_code <= ERROR_CODE_END)
            {
                // 历史故障信息顺位下移
                for (int i = 0; i < ERROR_CODE_COUNT - 1; i++)
                {
                    car_error_code[i] = car_error_code[i + 1];
                }
                // 插入置顶的最新故障信息
                car_error_code[0] = error_code;
            }
        }
#endif
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// notify send error status
    if (error_code < ERROR_CODE_BEGIN)
    {
        lt_carinfo_error.fault_ecu = 0;
        lt_carinfo_error.fault_sensor = 0;
        lt_carinfo_error.fault_motor = 0;
        lt_carinfo_error.fault_fuse = 0;
        lt_carinfo_error.fault_plug = 0;
        lt_carinfo_error.fault_battery = 0;
        lt_carinfo_error.fault_brake = 0;
        lt_carinfo_error.fault_throttle = 0;
        // send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, FRAME_CMD_CARINFOR_ERROR);
    }
    switch (error_code)
    {
    case ERROR_CODE_IDLE:
    case ERROR_CODE_NORMAL:
        break;

    case ERROR_CODE_THROTTLE_NOT_ZERO:
    case ERROR_CODE_THROTTLE_HALLSENSOR_ABNORMALITY:
        if (code_append)
            lt_carinfo_error.fault_throttle = error_code;
        else
            lt_carinfo_error.fault_throttle = 0;
        // send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, FRAME_CMD_CARINFOR_ERROR); // FRAME_CMD__CARINFOR_ERROR
        break;

    case ERROR_CODE_MOTOR_ABNORMALITY:
    case ERROR_CODE_CONTROLLER_ABNORMALITY:
        if (code_append)
        {
            lt_carinfo_error.fault_motor = error_code;
        }
        else
        {
            lt_carinfo_error.fault_motor = 0;
        }
        // send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, FRAME_CMD_CARINFOR_ERROR); // FRAME_CMD__CARINFOR_ERROR
        break;

    case ERROR_CODE_BATTERY_OVERHEAT:
        if (code_append)
        {
            lt_carinfo_error.fault_battery = error_code;
            lt_carinfo_error.fault_fuse = error_code;
        }
        else
        {
            lt_carinfo_error.fault_battery = 0;
            lt_carinfo_error.fault_fuse = 0;
        }
        // send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, FRAME_CMD_CARINFOR_ERROR); // FRAME_CMD__CARINFOR_ERROR
        break;

    case ERROR_CODE_LOW_VOLTAGE_PROTECTION:
    case ERROR_CODE_OVER_VOLTAGE_PROTECTION:
        if (code_append)
        {
            lt_carinfo_error.fault_plug = error_code;
            lt_carinfo_error.fault_battery = error_code;
        }
        else
        {
            lt_carinfo_error.fault_plug = 0;
            lt_carinfo_error.fault_battery = 0;
        }
        // send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, FRAME_CMD_CARINFOR_ERROR); // FRAME_CMD__CARINFOR_ERROR
        break;

    case ERROR_CODE_CONTROLLER_OVERHEAT:
    case ERROR_CODE_CONTROLLER_TEMPERATURE_SENSOR_ABNORMALITY:
    case ERROR_CODE_CURRENT_SENSOR_ABNORMALITY:
    case ERROR_CODE_MOTOR_TEMPERATURE_SENSOR_ABNORMALITY:
    case ERROR_CODE_ASSIST_POWER_SENSOR_ABNORMALITY:
    case ERROR_CODE_SPEED_SENSOR_ABNORMALITY:
    case ERROR_CODE_LAMP_SENSOR_ABNORMALITY:
        if (code_append)
        {
            lt_carinfo_error.fault_sensor = error_code;
            lt_carinfo_error.fault_ecu = error_code;
        }
        else
        {
            lt_carinfo_error.fault_sensor = 0;
            lt_carinfo_error.fault_ecu = 0;
        }
        // send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, FRAME_CMD_CARINFOR_ERROR); // FRAME_CMD__CARINFOR_ERROR
        break;

    case ERROR_CODE_HALLSENSOR_ABNORMALITY:
    case ERROR_CODE_LAMP_ABNORMALITY:
    case ERROR_CODE_BMS_ABNORMALITY:
        if (code_append)
        {
            lt_carinfo_error.fault_ecu = error_code;
        }
        else
            lt_carinfo_error.fault_ecu = 0;
        // send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, FRAME_CMD_CARINFOR_ERROR); // FRAME_CMD__CARINFOR_ERROR
        break;

    case ERROR_CODE_COMMUNICATION_ABNORMALITY:
    case ERROR_CODE_CAN_BMS_TIMEOUT:
    case ERROR_CODE_CAN_MOTOR_CONTROLLER_TIMEOUT:
    case ERROR_CODE_CAN_EXTERNAL_DEVICE_ERROR:
#ifdef CAR_EXTEND_ERROR_CODE
        if (CAR_INFOR_CHECK_ERROR(car_error_code_flags, ERROR_CODE_COMMUNICATION_ABNORMALITY) ||
            CAR_INFOR_CHECK_ERROR(car_error_code_flags, ERROR_CODE_CAN_BMS_TIMEOUT) ||
            CAR_INFOR_CHECK_ERROR(car_error_code_flags, ERROR_CODE_CAN_MOTOR_CONTROLLER_TIMEOUT) ||
            CAR_INFOR_CHECK_ERROR(car_error_code_flags, ERROR_CODE_CAN_EXTERNAL_DEVICE_ERROR))
        {
            lt_carinfo_indicator.ready = 1;
        }
        else
        {
            lt_carinfo_indicator.ready = 0;
        }
#else
        if (code_append)
            lt_carinfo_indicator.ready = 1;
        else
            lt_carinfo_indicator.ready = 0;
#endif
        break;
    default:
        break;
    }

    if (update_immediately)
    {
        send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, 0);
        if (update_send)
        {
            send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_ERROR, 1);
        }
    }
}

bool task_carinfo_has_error_code(void)
{
    return (car_error_code[0] > ERROR_CODE_NORMAL);
}
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t task_carinfo_getSpeed(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
    return lt_carinfo_meter.speed_actual;
#else
    return 0;
#endif
}

carinfo_indicator_t *task_carinfo_get_indicator_info(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
    return &lt_carinfo_indicator;
#else
    return NULL;
#endif
}

carinfo_meter_t *task_carinfo_get_meter_info(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
    return &lt_carinfo_meter;
#else
    return NULL;
#endif
}

carinfo_battery_t *task_carinfo_get_battery_info(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
    return &lt_carinfo_battery;
#else
    return NULL;
#endif
}

carinfo_error_t *task_carinfo_get_error_info(void)
{
#ifdef TASK_MANAGER_STATE_MACHINE_CARINFOR
    return &lt_carinfo_error;
#else
    return NULL;
#endif
}
