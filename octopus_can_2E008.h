
#ifndef __OCTOPUS_CAN_FUNCTION_2E008__
#define __OCTOPUS_CAN_FUNCTION_2E008__


#include "octopus_base.h" //  Base include file for the Octopus project.
#include "octopus_can_queue.h"
#include "octopus_tickcounter.h"
#include "octopus_vehicle.h"

#ifdef CUSTOMER_MODEL_DH_500
// -------- BMS --------
#pragma pack(push, 1)
// Union for data interpretation
typedef union {
    uint8_t bytes[8];      // Raw byte data (can represent 8 bytes of the CAN data)
    uint16_t word[4];      // Interprets the data as 4 words (16-bit integers)
    uint32_t dword[2];     // Interprets the data as 2 double-words (32-bit integers)
    float float_values[2]; // Interprets the data as 2 floats (for voltage or other float data)
	
    struct {
        uint16_t cell1_voltage;
        uint16_t cell2_voltage;
        uint16_t cell3_voltage;
        uint16_t cell4_voltage;
    } cell_voltages;      // Structure to interpret the first 8 bytes as voltages for 4 cells
		
    struct {
			uint8_t error_code1;
			uint8_t error_status1;
			uint8_t error_code2;
			uint8_t error_status2;
			uint8_t error_code3;
			uint8_t error_status3;
			uint8_t error_code4;
			uint8_t error_status4;
    } error_info;         // Structure to interpret the data as error code and status
		
    struct {
        uint16_t max_cell_voltage;
        uint16_t min_cell_voltage;
        uint16_t avg_cell_voltage;
    } cell_voltage_info;  // Structure for max/min/avg cell voltage data
		
    struct {
        uint16_t max_bat_temp;
        uint16_t min_bat_temp;
        uint16_t avg_bat_temp;
    } battery_temperature;  // Structure for max/min/avg battery temperature
		
    struct {
        uint8_t first_error;
        uint8_t second_error;
        uint8_t third_error;
    } error_status;       // Structure for various error statuses
		
		struct {
			uint16_t current;
			uint16_t voltage;
			uint8_t soc;
			uint8_t soh;
      // Byte6: Battery Status Flags 1
			union {
					uint8_t byte;
					struct {
							uint8_t charge_over_temp_prot     :1; // 7.0 充电过温保护
							uint8_t charge_low_temp_prot      :1; // 7.1 充电低温保护
							uint8_t discharge_over_temp_prot  :1; // 7.2 放电过温保护
							uint8_t discharge_low_temp_prot   :1; // 7.3 放电低温保护
							uint8_t signal_wire_broke_prot    :1; // 7.4 信号线断开保护
							uint8_t reserved_5                :1; // 7.5 保留
							uint8_t pack_over_voltage_prot    :1; // 7.6 整组过压保护
							uint8_t mos_over_temp_prot        :1; // 7.7 MOSFET过温保护
					} bits;
			} status1;

			// Byte7: Battery Status Flags 2
			union {
					uint8_t byte;
					struct {
							uint8_t cell_over_voltage_prot    :1; // 8.0 电芯过压保护
							uint8_t pack_low_voltage_prot     :1; // 8.1 整组欠压保护
							uint8_t cell_low_voltage_prot     :1; // 8.2 电芯欠压保护
							uint8_t over_charge_current_prot  :1; // 8.3 过充电流保护
							uint8_t over_discharge_current_prot:1;// 8.4 过放电流保护
							uint8_t short_circuit_prot        :1; // 8.5 短路保护
							uint8_t reserved_6                :1; // 8.6 保留
							uint8_t reserved_7                :1; // 8.7 保留
					} bits;
			} status2;
		}battery_pack;
		
} CAN_Data_Union;

#pragma pack(pop)


/**
 * @brief Dispatch a CAN message to appropriate handler based on its ID.
 * @this function may be used for routing logic in receive ISR or task.
 * @param msg Pointer to a CAN_Message_t structure.
 */
bool can_message_dispatcher(const CanQueueMsg_t *queue_msg);
bool can_message_sender(const uint16_t message_id);

#endif
#endif //__OCTOPUS_CAN_FUNCTION_2E006__
