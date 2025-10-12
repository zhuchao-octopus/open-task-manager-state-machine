
/*****************************************************************************/
#include "octopus_can_2E008.h"
#include "octopus_platform.h"
#include "octopus_tickcounter.h" // Tick Counter: provides timing and delay utilities
#include "octopus_message.h"     // Message IDs: defines identifiers for inter-task communication
#include "octopus_msgqueue.h"    // Message Queue: API for sending/receiving messages between tasks
#include "octopus_uart_ptl.h"    // UART Protocol Layer: handles protocol-level UART operations
#include "octopus_uart_upf.h"

#ifdef CUSTOMER_MODEL_DH_500

CAN_Data_Union data_union_2e008;
	 
void can_update_vehicle_infor(const CanQueueMsg_t *queue_msg);

/**
 * @brief Dispatch a CAN message to appropriate handler based on its ID.
 *        This function may be used for routing logic in receive ISR or task.
 * @param msg Pointer to a CAN_Message_t structure.
 */
bool can_message_dispatcher(const CanQueueMsg_t *queue_msg)
{
    // LOG_BUFF_LEVEL((uint8_t *)queue_msg, sizeof(CanQueueMsg_t));
    // bool updated = false;
    can_update_vehicle_infor(queue_msg);
		return true;
}

bool can_message_sender(const uint16_t message_id)
{		
		return false;
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

void can_update_vehicle_infor(const CanQueueMsg_t *queue_msg)
{
	 CAN_Data_Union data_union;
	 bool difference = false;
    // Copy the raw data from the CAN message into the union (big endian)
    for (int i = 0; i < 8; i++) 
	  {
        data_union.bytes[i] = queue_msg->data[i];	
    }
		 
		for (int i = 0; i < 8; i++) 
	  {
      if (data_union_2e008.bytes[i] != data_union.bytes[i])
			 difference = true;
    } 
		
		if (!difference) return;

		for (int i = 0; i < 8; i++) 
	  {
      data_union_2e008.bytes[i] = data_union.bytes[i];
    } 
		
   switch (queue_msg->std_id)
    {
			  case 0x300: break;
        case 0x301: break;
        case 0x302: break;
        case 0x303: break;
        case 0x304: break;
        case 0x305: break;
        case 0x306: break;
        case 0x307: break;
        case 0x308: break;
        case 0x309: break;
			  case 0x20A:
        case 0x30A: 
						lt_carinfo_battery.voltage = data_union_2e008.battery_pack.voltage;
						lt_carinfo_battery.current = data_union_2e008.battery_pack.current;
						lt_carinfo_battery.soc = data_union_2e008.battery_pack.soc;
						//lt_carinfo_battery.reserve = data_union_2e008.battery_pack.soh;
						carinfo_add_error_code(ERROR_CODE_OVER_VOLTAGE_PROTECTION, data_union_2e008.battery_pack.status2.bits.cell_over_voltage_prot, false);
						carinfo_add_error_code(ERROR_CODE_OVER_VOLTAGE_PROTECTION, data_union_2e008.battery_pack.status2.bits.cell_low_voltage_prot, false);

						//LOG_LEVEL("lt_carinfo_meter.speed_actual: %d\n", lt_carinfo_meter.speed_actual);
						send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_INDICATOR, FRAME_CMD_CARINFOR_INDICATOR);
						send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_BATTERY, FRAME_CMD_CARINFOR_BATTERY);
					break;
				
        case 0x30B: break;
        case 0x30C: break;
        case 0x30D: break;
				
        case 0x20E: 
				case 0x30E: 
						lt_carinfo_battery.abs_charge_state = data_union_2e008.bytes[0];
						lt_carinfo_battery.rel_charge_state = data_union_2e008.bytes[0];	
						send_message(TASK_MODULE_CAR_INFOR, MCU_TO_SOC_MOD_CARINFOR, FRAME_CMD_CARINFOR_BATTERY, FRAME_CMD_CARINFOR_BATTERY);				
					break;
        case 0x30F: break;
        default:
        break; // unknown ID
    }
}


#endif
