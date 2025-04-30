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

#include "octopus_platform.h"     // Include platform-specific header for hardware platform details
#include "octopus_log.h"          // Include logging functions for debugging
#include "octopus_task_manager.h" // Include task manager for scheduling tasks
#include "octopus_tickcounter.h"
#include "octopus_carinfor.h"
#include "octopus_msgqueue.h"
#include "octopus_sif.h"
#include "can/can_message_rx.h"
#include "can/can_function.h"

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */
#define CARINFOR_PTL_NO_ACK
/// #define TEST_LOG_DEBUG_SIF  // Uncomment to enable debug logging for SIF module

/*******************************************************************************
 * MACROS
 */
#define CELL_VOL_20 (1058) // Voltage corresponding to 20% battery charge
#define CELL_VOL_30 (1076) // Voltage corresponding to 30% battery charge
#define CELL_VOL_40 (1100) // Voltage corresponding to 40% battery charge
#define CELL_VOL_50 (1120) // Voltage corresponding to 50% battery charge
#define CELL_VOL_60 (1142) // Voltage corresponding to 60% battery charge
#define CELL_VOL_70 (1164) // Voltage corresponding to 70% battery charge
#define CELL_VOL_80 (1184) // Voltage corresponding to 80% battery charge
#define CELL_VOL_90 (1206) // Voltage corresponding to 90% battery charge

#define METER_AVG_BUFF_MAX (8)
#define INDICATOR_TYPE_ARRAY_COUNT ((kIndicatorTypeCount - 1) / 8 + 1)
/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * LOCAL FUNCTIONS DECLARATION
 */
static bool meter_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool meter_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

static bool indicator_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool indicator_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

static bool drivinfo_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff);
static bool drivinfo_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff);

static void app_car_controller_msg_handler(void); // Process messages related to car controller
#ifdef TASK_MANAGER_STATE_MACHINE_SIF
static void app_car_controller_sif_updating(void); // Update the SIF (System Information Frame)
#endif
#ifdef BATTERY_MANAGER
static void get_battery_voltage(void); // Retrieve the current battery voltage
#endif
#ifdef TEST_LOG_DEBUG_SIF
static void log_sif_data(uint8_t *data, uint8_t maxlen); // Log SIF data for debugging purposes
#endif

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * STATIC VARIABLES
 */
#ifdef TASK_MANAGER_STATE_MACHINE_SIF
static uint8_t sif_buff[12] = {0}; // Buffer for storing SIF data
carinfo_sif_t lt_sif = {0};             // Local SIF data structure
#endif

carinfo_meter_t lt_meter = {0};         // Local meter data structure
carinfo_indicator_t lt_indicator = {0}; // Local indicator data structure

//carinfo_drivinfo_t lt_drivinfo = {0};   // Local drivetrain information
// Timer variables
static uint32_t l_t_msg_wait_meter_timer; // Timer for 10 ms message wait (not used currently)
static uint32_t l_t_msg_wait_50_timer;    // Timer for 50 ms message wait
static uint32_t l_t_msg_wait_100_timer;   // Timer for 100 ms message wait

static uint32_t l_t_soc_timer; // Timer for state of charge monitoring

#ifdef TASK_MANAGER_STATE_MACHINE_CAN
// static bool l_t_speed_changed = false;
// static bool l_t_gear_changed = false;
//static uint8_t  l_u8_rpm_index = 0;
static uint16_t l_arr_rpm_buff[METER_AVG_BUFF_MAX] = {0};
static uint8_t  l_u8_speed_index = 0;
static uint16_t l_arr_speed_buff[METER_AVG_BUFF_MAX] = {0};

static uint8_t g_indicatorFlag[INDICATOR_TYPE_ARRAY_COUNT];
static uint8_t g_indicatorFlagChanged[INDICATOR_TYPE_ARRAY_COUNT];
static uint8_t g_indicatorWarnFlag[INDICATOR_TYPE_ARRAY_COUNT];
uint8_t g_hazardFlag = 0;
static uint8_t             lrlight_flag = 0;
#endif

/*******************************************************************************
 * EXTERNAL VARIABLES
 */

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */
void app_carinfo_init_running(void)
{
    LOG_LEVEL("app_carinfo_init_running\r\n");
    ptl_register_module(MCU_TO_SOC_MOD_METER, meter_module_send_handler, meter_module_receive_handler);
    ptl_register_module(MCU_TO_SOC_MOD_INDICATOR, indicator_module_send_handler, indicator_module_receive_handler);
    ptl_register_module(MCU_TO_SOC_MOD_DRIV_INFO, drivinfo_module_send_handler, drivinfo_module_receive_handler);

	  srand(1234); // Seed the random number generator
	
    OTMS(TASK_ID_CAR_INFOR, OTMS_S_INVALID);
}

void app_carinfo_start_running(void)
{
    LOG_LEVEL("app_carinfo_start_running\r\n");
    OTMS(TASK_ID_CAR_INFOR, OTMS_S_ASSERT_RUN);
}

void app_carinfo_assert_running(void)
{
    ptl_reqest_running(MCU_TO_SOC_MOD_METER);
    ptl_reqest_running(MCU_TO_SOC_MOD_INDICATOR);
    ptl_reqest_running(MCU_TO_SOC_MOD_DRIV_INFO);
    StartTickCounter(&l_t_msg_wait_meter_timer);
    StartTickCounter(&l_t_msg_wait_50_timer);
    StartTickCounter(&l_t_msg_wait_100_timer);
    StartTickCounter(&l_t_soc_timer);
    OTMS(TASK_ID_CAR_INFOR, OTMS_S_RUNNING);
}

void app_carinfo_running(void)
{
    /// if(MB_ST_OFF == app_power_state_get_mb_state())
    ///{
    /// OTMS(CAR_INFOR_ID, OTMS_S_POST_RUN);
    ///}

#ifdef TASK_MANAGER_STATE_MACHINE_SIF
    app_car_controller_sif_updating();
#endif

    if (GetTickCounter(&l_t_msg_wait_50_timer) < 5)
        return;
    StartTickCounter(&l_t_msg_wait_50_timer);

#ifdef TASK_MANAGER_STATE_MACHINE_MCU
    app_car_controller_msg_handler();
#endif
}

void app_carinfo_post_running(void)
{
    ptl_release_running(MCU_TO_SOC_MOD_METER);
    ptl_release_running(MCU_TO_SOC_MOD_INDICATOR);
    ptl_release_running(MCU_TO_SOC_MOD_DRIV_INFO);

    /// if(MB_ST_OFF != app_power_state_get_mb_state())
    ///{
    /// OTMS(CAR_INFOR_ID, OTMS_S_ASSERT_RUN);
    /// }
    /// goto app_carinfo_running;
}

void app_carinfo_stop_running(void)
{
    OTMS(TASK_ID_CAR_INFOR, OTMS_S_INVALID);
}

void app_carinfo_on_enter_run(void)
{
    /// if (KCS(AppSetting) > OTMS_S_POST_RUN)
    ///{
    ///     OTMS(CAR_INFOR_ID, OTMS_S_START);
    /// }
}

void app_carinfo_on_exit_post_run(void)
{
    OTMS(TASK_ID_CAR_INFOR, OTMS_S_STOP);
}

uint16_t app_carinfo_getSpeed(void)
{
    return lt_meter.actual_speed;
}

carinfo_indicator_t *app_carinfo_get_indicator_info(void)
{
    return &lt_indicator;
}

carinfo_meter_t *app_carinfo_get_meter_info(void)
{
    return &lt_meter;
}

carinfo_drivinfo_t *app_carinfo_get_drivinfo_info(void)
{
    return NULL;//&lt_drivinfo;
}

/*******************************************************************************
 * LOCAL FUNCTIONS IMPLEMENTATION
 */
bool meter_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    MY_ASSERT(buff);
    //uint8_t tmp[16] = {0};
    if (MCU_TO_SOC_MOD_METER == frame_type)
    {
        switch (param1)
        {
        case CMD_MODMETER_RPM_SPEED:
        case CMD_MODMETER_SOC:
				case CMD_MODINDICATOR_METER:
        default:
					  ptl_build_frame(MCU_TO_SOC_MOD_METER, CMD_MODINDICATOR_METER, (uint8_t*)&lt_meter, sizeof(carinfo_meter_t), buff);
				 return true;
        }
    }
    else if (SOC_TO_MCU_MOD_METER == frame_type)
    {
        switch (param1)
        {
        case CMD_MODMETER_RPM_SPEED:
            return false;
        case CMD_MODMETER_SOC:
            return false;
        default:
            break;
        }
    }
    return false;
}

bool meter_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    MY_ASSERT(payload);
    MY_ASSERT(ackbuff);
#ifndef CARINFOR_PTL_NO_ACK // no ack
    uint8_t tmp = 0;
#endif
    if (SOC_TO_MCU_MOD_METER == payload->frame_type)
    {
        switch (payload->cmd)
        {
        case CMD_MODMETER_RPM_SPEED:
            // ACK, no thing to do
            return false;
        case CMD_MODMETER_SOC:
            // ACK, no thing to do
            return false;
        default:
            break;
        }
    }
    else if (MCU_TO_SOC_MOD_METER == payload->frame_type)
    {
        switch (payload->cmd)
        {
        case CMD_MODMETER_RPM_SPEED:
            // LOGIC
            lt_meter.actual_speed = MK_WORD(payload->data[0], payload->data[1]);
            lt_meter.rpm = MK_WORD(payload->data[2], payload->data[3]);
            lt_meter.speed = lt_meter.actual_speed * 11 / 10;
            // ACK
#ifndef CARINFOR_PTL_NO_ACK // no ack
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_METER, CMD_MODMETER_RPM_SPEED, &tmp, 1, ackbuff);
#endif
            return true;
        case CMD_MODMETER_SOC:
            // LOGIC
            lt_meter.soc = payload->data[0];
            lt_meter.voltage = MK_WORD(payload->data[1], payload->data[2]);
            lt_meter.current = MK_WORD(payload->data[3], payload->data[4]);
            lt_meter.voltage_system = payload->data[5];
            // ACK
#ifndef CARINFOR_PTL_NO_ACK // no ack
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_METER, CMD_MODMETER_SOC, &tmp, 1, ackbuff);
#endif
            return true;
        default:
            break;
        }
    }
    return false;
}

bool indicator_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    MY_ASSERT(buff);
    //uint8_t tmp[16] = {0};
    if (MCU_TO_SOC_MOD_INDICATOR == frame_type)
    {
        switch (param1)
        {
        case CMD_MODINDICATOR_INDICATOR:
        case CMD_MODINDICATOR_ERROR_INFO:
        default:
					 ptl_build_frame(MCU_TO_SOC_MOD_INDICATOR, CMD_MODINDICATOR_INDICATOR, (uint8_t*)&lt_indicator, sizeof(carinfo_indicator_t), buff);
            return true;
        }
    }
    else if (SOC_TO_MCU_MOD_INDICATOR == frame_type)
    {
        switch (param1)
        {
        case CMD_MODINDICATOR_INDICATOR:
            return false;
        case CMD_MODINDICATOR_ERROR_INFO:
            return false;
        default:
            break;
        }
    }
    return false;
}

bool indicator_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    MY_ASSERT(payload);
    MY_ASSERT(ackbuff);
#ifndef CARINFOR_PTL_NO_ACK // no ack
    uint8_t tmp = 0;
#endif
    if (SOC_TO_MCU_MOD_INDICATOR == payload->frame_type)
    {
        switch (payload->cmd)
        {
        case CMD_MODINDICATOR_INDICATOR:
            // ACK, no thing to do
            return false;
        case CMD_MODINDICATOR_ERROR_INFO:
            // ACK, no thing to do
            return false;
        default:
            break;
        }
    }
    else if (MCU_TO_SOC_MOD_INDICATOR == payload->frame_type)
    {
        switch (payload->cmd)
        {
        case CMD_MODINDICATOR_INDICATOR:
            // LOGIC
            lt_indicator.highBeam = GetBit(payload->data[0], 0);    // Զ���
            lt_indicator.lowBeam = GetBit(payload->data[0], 1);     // �����
            lt_indicator.position = GetBit(payload->data[0], 2);    // λ�õ�
            lt_indicator.frontFog = GetBit(payload->data[0], 3);    // ǰ����
            lt_indicator.rearFog = GetBit(payload->data[0], 4);     // ������
            lt_indicator.leftTurn = GetBit(payload->data[0], 5);    // ��ת��
            lt_indicator.rightTurn = GetBit(payload->data[0], 6);   // ��ת��
            lt_indicator.ready = GetBit(payload->data[0], 7);       // Ready��
            lt_indicator.charge = GetBit(payload->data[1], 0);      // ��س�ŵ��
            lt_indicator.parking = GetBit(payload->data[1], 1);     // פ����
            lt_indicator.ecuFault = GetBit(payload->data[1], 2);    // ECU���ϵ�
            lt_indicator.sensorFault = GetBit(payload->data[1], 3); // ���������ϵ�
            lt_indicator.motorFault = GetBit(payload->data[1], 4);  // ������ϵ�
#ifndef CARINFOR_PTL_NO_ACK                                         // no ack
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_INDICATOR, CMD_MODINDICATOR_INDICATOR, &tmp, 1, ackbuff);
#endif
            return true;
        case CMD_MODINDICATOR_ERROR_INFO:
#ifndef CARINFOR_PTL_NO_ACK // no ack
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_INDICATOR, CMD_MODINDICATOR_ERROR_INFO, &tmp, 1, ackbuff);
#endif
            return true;
        default:
            break;
        }
    }
    return false;
}

bool drivinfo_module_send_handler(ptl_frame_type_t frame_type, uint16_t param1, uint16_t param2, ptl_proc_buff_t *buff)
{
    MY_ASSERT(buff);
    uint8_t tmp[16] = {0};
    if (MCU_TO_SOC_MOD_DRIV_INFO == frame_type)
    {
        switch (param1)
        {
        case CMD_MODDRIVINFO_GEAR:
            //tmp[0] = lt_drivinfo.gear;
            //tmp[1] = lt_drivinfo.driveMode;
            ptl_build_frame(MCU_TO_SOC_MOD_DRIV_INFO, CMD_MODDRIVINFO_GEAR, tmp, 2, buff);
            return true;
        default:
            break;
        }
    }
    else if (SOC_TO_MCU_MOD_DRIV_INFO == frame_type)
    {
        switch (param1)
        {
        case CMD_MODDRIVINFO_GEAR:
            return false;
        default:
            break;
        }
    }
    return false;
}

bool drivinfo_module_receive_handler(ptl_frame_payload_t *payload, ptl_proc_buff_t *ackbuff)
{
    MY_ASSERT(payload);
    MY_ASSERT(ackbuff);

    if (SOC_TO_MCU_MOD_DRIV_INFO == payload->frame_type)
    {
        switch (payload->cmd)
        {
        case CMD_MODDRIVINFO_GEAR:
            // ACK, no thing to do
            return false;
        default:
            break;
        }
    }
    else if (MCU_TO_SOC_MOD_DRIV_INFO == payload->frame_type)
    {
        switch (payload->cmd)
        {
        case CMD_MODDRIVINFO_GEAR:
            // LOGIC
            //lt_drivinfo.gear = (carinfo_drivinfo_gear_t)payload->data[0];
            //lt_drivinfo.driveMode = (carinfo_drivinfo_drivemode_t)payload->data[1];
            // ACK
#ifndef CARINFOR_PTL_NO_ACK // no ack
            uint8_t tmp = 0;
            tmp = 0x01;
            ptl_build_frame(SOC_TO_MCU_MOD_DRIV_INFO, CMD_MODDRIVINFO_GEAR, &tmp, 1, ackbuff);
#endif
            return true;
        default:
            break;
        }
    }
    return false;
}

int get_random_0_to_10(void)
{
    return rand() % 11;  // rand() % (max - min + 1) + min, here: 0~10
}

void app_car_controller_msg_handler(void)
{
	static bool test_ =false;
	
	#ifdef TASK_MANAGER_STATE_MACHINE_SIF
    lt_indicator.position = GPIO_PIN_READ_SKD() ? 0 : 1; 
    lt_indicator.highBeam = GPIO_PIN_READ_DDD() ? 0 : 1;  
    lt_indicator.leftTurn = GPIO_PIN_READ_ZZD() ? 0 : 1; 
    lt_indicator.rightTurn = GPIO_PIN_READ_YZD() ? 0 : 1; 

    lt_indicator.ready = !lt_sif.bootGuard;

    lt_indicator.ecuFault = lt_sif.controllerFault;
    lt_indicator.sensorFault = lt_sif.throttleFault;
    lt_indicator.motorFault = lt_sif.motorFault | lt_sif.hallFault;

    lt_indicator.parking = lt_sif.brake;
	#endif
	#ifdef BATTERY_MANAGER
    get_battery_voltage();
	#endif
	

    Msg_t *msg = get_message(TASK_ID_CAR_INFOR);
    if (msg->id != NO_MSG && (MsgId_t)msg->id == MSG_DEVICE_GPIO_EVENT)
    {
        //send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_INDICATOR, CMD_MODINDICATOR_INDICATOR, 0);
    }
 
		if(GetTickCounter(&l_t_msg_wait_100_timer) >= 1000)
    {
			 int r = get_random_0_to_10();
			 lt_meter.actual_speed = r;
		   lt_meter.speed = r;
			 
			 if(test_)
			 {
					 send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_METER, CMD_MODINDICATOR_METER, 0);	 
			 }
			 else
			 {
					if(lt_indicator.leftTurn == 0)
					{
						lt_indicator.leftTurn = 1;
						lt_indicator.rightTurn = 1;
					}
					else
					{
						lt_indicator.leftTurn = 0;
						lt_indicator.rightTurn = 0;
					}
					send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_INDICATOR, CMD_MODINDICATOR_INDICATOR, 0);   
			 }
			 test_ = !test_;
			 
		   StartTickCounter(&l_t_msg_wait_100_timer);
    } 
		 	 
	#ifdef TASK_MANAGER_STATE_MACHINE_SIF
    if(l_t_speed_changed)
		{
			send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_METER, CMD_MODMETER_RPM_SPEED, 0);	
			l_t_speed_changed=false;			
		}
		if(l_t_gear_changed)
		{
			send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_DRIV_INFO, CMD_MODDRIVINFO_GEAR, 0);
			l_t_gear_changed = false;
		}

		static uint8_t l_u8_op_step = 0;  // Operational step variable
    if(GetTickCounter(&l_t_msg_wait_100_timer) >= 300)
    {
        switch(l_u8_op_step)
        {
        case 0:
            send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_METER, CMD_MODMETER_SOC, 0);
            break;
        case 1:
            send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_METER, CMD_MODMETER_RPM_SPEED, 0);		
            break;
        case 2:
            send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_DRIV_INFO, CMD_MODDRIVINFO_GEAR, 0);
            break;
				case 3:
            send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_INDICATOR, CMD_MODINDICATOR_ERROR_INFO, 0);
            break;
        default:
            l_u8_op_step = (uint8_t)-1;
            break;
        }
				
        l_u8_op_step++;
				if(l_u8_op_step >=4)
					l_u8_op_step = 0;
        StartTickCounter(&l_t_msg_wait_100_timer);
    }
	#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef TASK_MANAGER_STATE_MACHINE_CAN

// 指示灯自检
#if 0
static void car_indicator_proc_self_test(void)
{
    car_indicator_set_indicatorflag(kHazardSignal, true);
    car_indicator_set_indicatorflag(kLeftTurnSignal, true);
    car_indicator_set_indicatorflag(kRightTurnSignal, true);
    car_indicator_set_indicatorflag(kLowBeam, true);
    car_indicator_set_indicatorflag(kHighBeam, true);
    car_indicator_set_indicatorflag(kPositionLight, true);
    car_indicator_set_indicatorflag(kDrvSeatBeltRst, true);
    car_indicator_set_indicatorflag(kParkingSignal, true);
    car_indicator_set_indicatorflag(kBrakeFluidLow, true);
    car_indicator_set_indicatorflag(kCharging, true);
    car_indicator_set_indicatorflag(kEpsFault, true);

    car_indicator_set_indicatorflag(kBattSOCUnder, true);
    car_indicator_set_indicatorflag(kBattSOCChrg, true);
    car_indicator_set_indicatorflag(kFrntMotTOver, true);
    car_indicator_set_indicatorflag(kNormal, false);
    car_indicator_set_indicatorflag(kSport, false);
    car_indicator_set_indicatorflag(kEco, true);
    car_indicator_set_indicatorflag(kObcCpValVld, true);
    car_indicator_set_indicatorflag(kBCUChrgSts, true);
    car_indicator_set_indicatorflag(kMotSysPwrLmtLp, true);
    car_indicator_set_indicatorflag(kFrntIpuFltRnk, true);
    car_indicator_set_indicatorflag(kBCUMILReq, true);
    car_indicator_set_indicatorflag(kBCUCellTOver, true);
    car_indicator_set_indicatorflag(kReadyLp, true);
    //data_indicator_set_indicatorflag(kMotTempHight, true);
    //data_indicator_set_indicatorflag(kCurrentG, true);
}
#endif
//转向灯、危险警告灯
static void car_indicator_proc_turn_signal(void)
{
    uint32_t sig_val_elsswitch = 0;
    uint32_t sig_val_left = 0;
    uint32_t sig_val_right = 0;
    
    uint8_t  valid = 0;
    
    valid = (can_function_read_signal_timeout_flag(CAN_MSG_RX_BCM5) == 0);
 
    //应急灯开关
    can_function_read_signal_value(CAN_MSG_RX_BCM5, BCM5_ElsSwitch, &sig_val_elsswitch);
    //左转向灯
    can_function_read_signal_value(CAN_MSG_RX_BCM5, BCM5_LeftTurnLamp, &sig_val_left);
    //右转向灯
    can_function_read_signal_value(CAN_MSG_RX_BCM5, BCM5_RightTurnLamp, &sig_val_right);
    
    if(valid && (sig_val_elsswitch || sig_val_left || sig_val_right))
    {
        lrlight_flag = !lrlight_flag;
    }
    else
    {
        lrlight_flag = 0;
    }
        
    car_indicator_set_indicatorflag(kLeftTurnSignal, lrlight_flag && valid && (sig_val_left || sig_val_elsswitch));
    car_indicator_set_indicatorflag(kRightTurnSignal, lrlight_flag && valid && (sig_val_right || sig_val_elsswitch));
    car_indicator_set_indicatorflag(kHazardSignal, lrlight_flag && valid && sig_val_elsswitch);
    
    car_indicator_set_warnflag(kLeftTurnSignal, lrlight_flag && valid && (sig_val_left || sig_val_elsswitch));
    car_indicator_set_warnflag(kRightTurnSignal, lrlight_flag && valid && (sig_val_right || sig_val_elsswitch));
    car_indicator_set_warnflag(kHazardSignal, lrlight_flag && valid && sig_val_elsswitch);
    
    car_indicator_set_hazard(valid && sig_val_elsswitch);
}
// Process vehicle speed and engine RPM
static void car_meter_proc_speed_rpm(void) {
    uint32_t sig_val = 0;
    uint8_t valid = 0;
    uint32_t vehspd_vld = 0;
    uint8_t flag = 0;
    uint16_t speed = 0;
    uint16_t rpm = 0;
    uint64_t sum = 0;

    //CarInfo* info = data_carinfo_get_carinfo();

    //if (sys_pwr_mark_get_acc() == 1) 
			{ // Only process when ACC is ON
        // Read vehicle speed
        can_function_read_signal_value(CAN_MSG_RX_IPU6, IPU6_IPUVehSpdVld, &vehspd_vld);
        can_function_read_signal_value(CAN_MSG_RX_IPU6, IPU6_IPUVehSpd, &sig_val);
        valid = (can_function_read_signal_timeout_flag(CAN_MSG_RX_IPU6) == 0) && sig_val && (!vehspd_vld);

#if METER_AUTO_RUM
        // Simulated speed for auto run mode
        static uint32_t speed_auto = 0;
        speed_auto += 10;
        if (speed_auto > 1200) speed_auto = 0;
        for (uint8_t i = 0; i < METER_AVG_BUFF_MAX; i++) {
            l_arr_speed_buff[i] = speed_auto;
        }
#else
        // Store speed sample in buffer
        l_arr_speed_buff[l_u8_speed_index] = valid ? (sig_val * 10) : 0;
        l_u8_speed_index++;
        if (l_u8_speed_index >= METER_AVG_BUFF_MAX) {
            l_u8_speed_index = 0;
        }
#endif

        // Read engine RPM
        can_function_read_signal_value(CAN_MSG_RX_IPU2, IPU2_FrntMotSpdVld, &vehspd_vld);
        can_function_read_signal_value(CAN_MSG_RX_IPU2, IPU2_FrntMotSpd, &sig_val);
        valid = (can_function_read_signal_timeout_flag(CAN_MSG_RX_IPU2) == 0) && sig_val && vehspd_vld;

#if METER_AUTO_RUM
        // Simulated RPM for auto run mode
        static uint32_t rpm_auto = 0;
        rpm_auto += 100;
        if (rpm_auto > 9000) rpm_auto = 0;
        for (uint8_t i = 0; i < METER_AVG_BUFF_MAX; i++) {
            l_arr_rpm_buff[i] = rpm_auto;
        }
#else
        // Store RPM sample in buffer
				l_arr_speed_buff[l_u8_speed_index] = valid ? (sig_val * 10) : 0;
        l_u8_speed_index++;
        if(l_u8_speed_index >= METER_AVG_BUFF_MAX)
        {
            l_u8_speed_index = 0;
        }
#endif
    } 

    // Calculate average speed
    sum = 0;
    for (uint8_t i = 0; i < METER_AVG_BUFF_MAX; i++) {
        sum += l_arr_speed_buff[i];
    }
    speed = sum / METER_AVG_BUFF_MAX;

    // Calculate average RPM
    sum = 0;
    for (uint8_t i = 0; i < METER_AVG_BUFF_MAX; i++) {
        sum += l_arr_rpm_buff[i];
    }
    rpm = sum / METER_AVG_BUFF_MAX;

    // Check and update if speed changed
		// if (lt_meter.actual_speed != lt_meter_current_speed)
    if (lt_meter.actual_speed != speed) {
        flag = true;
       lt_meter.actual_speed = speed;
    }

    // Apply different threshold for idle state
    uint8_t threshold = lt_meter.actual_speed ? 50 : 98;
    if (abs(rpm - lt_meter.rpm) >= threshold) {
        if (lt_meter.rpm != rpm) {
            flag = true;
            lt_meter.rpm = rpm;
        }
    }

    if (flag) {
         send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_METER, CMD_MODMETER_RPM_SPEED, 0);
    }
}

void car_indicator_init() {
    uint8_t i = 0;
    for (i = 0; i < INDICATOR_TYPE_ARRAY_COUNT; i++) {
        g_indicatorFlag[i] = 0;
        g_indicatorFlagChanged[i] = 0;
        g_indicatorWarnFlag[i] = 0;
    }
}

uint8_t car_indicator_get_indicator_type_array_count(){
    return INDICATOR_TYPE_ARRAY_COUNT;
}

uint8_t car_indicator_get_indicator(uint8_t index, uint8_t *indicator) {
    uint8_t ret = 0;
    if (index < sizeof(g_indicatorFlag)) {
        ret = 1;
        *indicator = g_indicatorFlag[index];
    } else {
        ret = 0;
    }
    return ret;
}

uint8_t car_indicator_get_indicatorflag(IndicatorType type) {
    return (g_indicatorFlag[type / 8] >> (type % 8)) & 0x01;
}

void car_indicator_set_indicatorflag(IndicatorType type, uint8_t flag) {
    if (flag == car_indicator_get_indicatorflag(type)) {
        return;
    }

    if (flag) {
        g_indicatorFlag[type / 8] |= (0x01 << (type % 8));
    } else {
        g_indicatorFlag[type / 8] &= ~(0x01 << (type % 8));
    }
    g_indicatorFlagChanged[type / 8] |= (0x01 << (type % 8));
}

uint8_t car_indicator_get_indicatorflag_changed() {
    uint8_t i = 0;
    for (i = 0; i < INDICATOR_TYPE_ARRAY_COUNT; i++) {
        if(g_indicatorFlagChanged[i] > 0){
            return 1;
        }
    }
    return 0;
}

void car_indicator_clear_indicatorflag_changed() {
    uint8_t i = 0;
    for (i = 0; i < INDICATOR_TYPE_ARRAY_COUNT; i++) {
        g_indicatorFlagChanged[i] = 0;
    }
}

uint8_t car_indicator_get_warnflag(IndicatorType type)
{
    return (g_indicatorWarnFlag[type / 8] >> (type % 8)) & 0x01;
}

void car_indicator_set_warnflag(IndicatorType type, uint8_t flag)
{
    if (flag) {
        g_indicatorWarnFlag[type / 8] |= (0x01 << (type % 8));
        //StartTickCounter(&lt_warn_wait_timer);
    } else {
        g_indicatorWarnFlag[type / 8] &= ~(0x01 << (type % 8));
    }
}

uint8_t cat_indicator_get_warnflag_event() {
#if 0
    uint8_t i = 0;
    for (i = 0; i < INDICATOR_TYPE_ARRAY_COUNT; i++) {
        if(g_indicatorWarnFlag[i] > 0){
            return 1;
        }
    }
    
    if(IsTimerStart(&lt_warn_wait_timer) && GetTimer(&lt_warn_wait_timer) < 5000)
    {
        return 1;
    }
#endif
    return 0;
}


uint8_t cat_indicator_get_hazard(void )
{
    return g_hazardFlag;
}
void car_indicator_set_hazard(uint8_t flag)
{
    g_hazardFlag = flag;
}
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef TASK_MANAGER_STATE_MACHINE_SIF
void app_car_controller_sif_updating(void)
{
    uint8_t res = SIF_ReadData(sif_buff, sizeof(sif_buff));
    // uint8_t lt_meter_current_gear = 0;
    uint16_t lt_meter_current_speed = 0;

#ifdef TEST_LOG_DEBUG_SIF
    if (res)
        log_sif_data(sif_buff, sizeof(sif_buff));
#endif
    if (res && sif_buff[0] == 0x08 && sif_buff[1] == 0x61)
    {
        lt_sif.sideStand = ((sif_buff[2] & 0x08) ? 1 : 0);                // ���Ŷϵ���  0:��������     1:���ŷ���
        lt_sif.bootGuard = ((sif_buff[2] & 0x02) ? 1 : 0);                // ��������            0:�Ǳ���          1:������
        lt_sif.hallFault = ((sif_buff[3] & 0x40) ? 1 : 0);                // ��������(���)0:����            1:����
        lt_sif.throttleFault = ((sif_buff[3] & 0x20) ? 1 : 0);            // ת�ѹ���
        lt_sif.controllerFault = ((sif_buff[3] & 0x10) ? 1 : 0);          // ����������
        lt_sif.lowVoltageProtection = ((sif_buff[3] & 0x08) ? 1 : 0);     // Ƿѹ����
        lt_sif.cruise = ((sif_buff[3] & 0x04) ? 1 : 0);                   // Ѳ��ָʾ��
        lt_sif.assist = ((sif_buff[3] & 0x02) ? 1 : 0);                   // ����ָʾ��
        lt_sif.motorFault = ((sif_buff[3] & 0x01) ? 1 : 0);               // �������
        lt_sif.gear = ((sif_buff[4] & 0x80) >> 5) | (sif_buff[4] & 0x03); // ��λ//0~7
        lt_sif.motorRunning = ((sif_buff[4] & 0x40) ? 1 : 0);             // ������� 1����
        lt_sif.brake = ((sif_buff[4] & 0x20) ? 1 : 0);                    // ɲ��
        lt_sif.controllerProtection = ((sif_buff[4] & 0x10) ? 1 : 0);     // ����������
        lt_sif.coastCharging = ((sif_buff[4] & 0x08) ? 1 : 0);            // ���г��
        lt_sif.antiSpeedProtection = ((sif_buff[4] & 0x04) ? 1 : 0);      // ���ɳ�����
        lt_sif.seventyPercentCurrent = ((sif_buff[5] & 0x80) ? 1 : 0);    // 70%����
        lt_sif.pushToTalk = ((sif_buff[5] & 0x40) ? 1 : 0);               // ����һ��ͨ
        lt_sif.ekkBackupPower = ((sif_buff[5] & 0x20) ? 1 : 0);           // ����EKK���õ�Դ
        lt_sif.overCurrentProtection = ((sif_buff[5] & 0x10) ? 1 : 0);    // ��������
        lt_sif.motorShaftLockProtection = ((sif_buff[5] & 0x08) ? 1 : 0); // ��ת����
        lt_sif.reverse = ((sif_buff[5] & 0x04) ? 1 : 0);                  // ����
        lt_sif.electronicBrake = ((sif_buff[5] & 0x02) ? 1 : 0);          // ����ɲ��
        lt_sif.speedLimit = ((sif_buff[5] & 0x01) ? 1 : 0);               // ����
        lt_sif.current = ((sif_buff[6] & 0xFF));                          // ���� ��λ��1A
        lt_sif.hallCounter = MK_WORD(sif_buff[7], sif_buff[8]);           // 0.5s�����������仯�ĸ���
        lt_sif.soc = ((sif_buff[9] & 0xFF));                              // ����/���� 0-100% 5��ָʾΪ 90,70,50,30,20���ٷֱȣ������Ӧ�ĵ�ѹ����Ϊ 47V��46V,44.5V,43V,41V)��4 ��ָʾΪ 90,70,50,30
        lt_sif.voltage_system = ((sif_buff[10] & 0xFF));                   // ��ѹϵͳ  0x01:36V  0x02:48V  0x04:60V  0x08:64V  0x10:72V  0x20:80V  0x40:84V   0x80:96V

        double rpm = lt_sif.hallCounter * (2.0 * 60 / 100.0);
        double radius = 0.254 / 2.0;                      // ��̥�뾶
        double w = rpm * (2.0 * 3.14159265358979 / 60.0); // ת�����ٶȣ���λ������/��
        double v = w * radius;                            // ���ٶȣ���λ:��/��

        lt_meter.rpm = rpm + 20000; // offset:-20000
        lt_meter.speed = v * (10.0 * 3600.0 / 1000.0) * 1.1;

        lt_meter_current_speed = v * (10.0 * 3600.0 / 1000.0);

        lt_meter.voltage_system = lt_sif.voltage_system;
        // lt_meter.soc = lt_sif.soc;
        lt_meter.current = lt_sif.current * 10; // test

        if (lt_sif.gear != lt_drivinfo.gear)
        {
            // l_t_gear_changed = true;
            LOG_LEVEL("SIF DATA:lt_drivinfo.gear changed\r\n");
            send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_DRIV_INFO, CMD_MODDRIVINFO_GEAR, 0);
        }
        lt_drivinfo.gear = (carinfo_drivinfo_gear_t)lt_sif.gear;
        if (lt_meter.actual_speed != lt_meter_current_speed)
        {
            // l_t_speed_changed=true;
            LOG_LEVEL("SIF DATA:lt_drivinfo.actual_speed changed\r\n");
            send_message(TASK_ID_PTL, MCU_TO_SOC_MOD_METER, CMD_MODMETER_RPM_SPEED, 0);
        }
        lt_meter.actual_speed = lt_meter_current_speed;
    }
}
#endif


#ifdef TEST_LOG_DEBUG_SIF
void log_sif_data(uint8_t *data, uint8_t maxlen)
{
    LOG_LEVEL("SIF DATA:");
    for (int i = 0; i < maxlen; i++)
    {
        LOG_("0x%02x ", data[i]);
    }
    LOG_("\r\n");
}
#endif

#ifdef BATTERY_MANAGER
void get_battery_voltage(void)
{
    if (GetTickCounter(&l_t_soc_timer) < 1000)
    {
        return;
    }
    StartTickCounter(&l_t_soc_timer);
    uint8_t cellcount = 4; // Ĭ��4�ŵ��
    uint16_t vol = 0;      //(uint32_t)SensorAdc_Get_BatVal() * 274 / 1000;

    bool rise = vol > lt_meter.voltage;
    lt_meter.voltage = vol;
    uint8_t soc = 100;

    if (rise)
    {
        switch (lt_sif.voltage_system)
        {
        case 0x00:
        case 0x02: // 48V
        {
            {
                if (lt_meter.voltage >= 480)
                    soc = 100;
                else if (lt_meter.voltage >= 465)
                    soc = 80;
                else if (lt_meter.voltage >= 445)
                    soc = 40;
                else if (lt_meter.voltage >= 415)
                    soc = 20;
                else
                    soc = 10;
            }

            // if(soc < lt_meter.soc)
            {
                lt_meter.soc = soc;
            }

            return;
        }
        case 0x04: // 60V
        {
            if (lt_meter.voltage >= 600)
                soc = 100;
            else if (lt_meter.voltage >= 574)
                soc = 80;
            else if (lt_meter.voltage >= 550)
                soc = 40;
            else if (lt_meter.voltage >= 526)
                soc = 20;
            else
                soc = 10;
        }

            // if(soc < lt_meter.soc)
            {
                lt_meter.soc = soc;
            }
            return;
        case 0x10: // 72V
        {
            if (lt_meter.voltage >= 719)
                soc = 100;
            else if (lt_meter.voltage >= 690)
                soc = 80;
            else if (lt_meter.voltage >= 660)
                soc = 40;
            else if (lt_meter.voltage >= 630)
                soc = 20;
            else
                soc = 10;
        }

            // if(soc < lt_meter.soc)
            {
                lt_meter.soc = soc;
            }
            return;
        }
    }
    else
    {
        switch (lt_sif.voltage_system)
        {
        case 0x00:
        case 0x02: // 48V
        {
            {
                if (lt_meter.voltage >= 470)
                    soc = 100;
                else if (lt_meter.voltage >= 455)
                    soc = 80;
                else if (lt_meter.voltage >= 435)
                    soc = 40;
                else if (lt_meter.voltage >= 405)
                    soc = 20;
                else
                    soc = 10;
            }

            // if(soc < lt_meter.soc)
            {
                lt_meter.soc = soc;
            }

            return;
        }
        case 0x04: // 60V
        {
            if (lt_meter.voltage >= 590)
                soc = 100;
            else if (lt_meter.voltage >= 564)
                soc = 80;
            else if (lt_meter.voltage >= 540)
                soc = 40;
            else if (lt_meter.voltage >= 516)
                soc = 20;
            else
                soc = 10;
        }

            // if(soc < lt_meter.soc)
            {
                lt_meter.soc = soc;
            }
            return;
        case 0x10: // 72V
        {
            if (lt_meter.voltage >= 709)
                soc = 100;
            else if (lt_meter.voltage >= 680)
                soc = 80;
            else if (lt_meter.voltage >= 650)
                soc = 40;
            else if (lt_meter.voltage >= 620)
                soc = 20;
            else
                soc = 10;
        }

            // if(soc < lt_meter.soc)
            {
                lt_meter.soc = soc;
            }
            return;
        }
    }

    switch (lt_sif.voltage_system)
    {
    case 0x01: // 36V
        cellcount = 3;
        break;
    case 0x02: // 48V
        cellcount = 4;
        break;
    case 0x04: // 60V
        cellcount = 5;
        break;
    case 0x08:         // 64V
        cellcount = 5; //
        break;
    case 0x10: // 72V
        cellcount = 6;
        break;
    case 0x20:         // 80V
        cellcount = 6; // 
        break;
    case 0x40: // 84V
        cellcount = 7;
        break;
    case 0x80: // 96V
        cellcount = 8;
        break;
    default:
        break;
    }

    if (lt_meter.voltage > (CELL_VOL_90 * cellcount / 10))
    {
        soc = 90;
    }
    else if (lt_meter.voltage > (CELL_VOL_80 * cellcount / 10))
    {
        soc = 80;
    }
    else if (lt_meter.voltage > (CELL_VOL_70 * cellcount / 10))
    {
        soc = 70;
    }
    else if (lt_meter.voltage > (CELL_VOL_60 * cellcount / 10))
    {
        soc = 60;
    }
    else if (lt_meter.voltage > (CELL_VOL_50 * cellcount / 10))
    {
        soc = 50;
    }
    else if (lt_meter.voltage > (CELL_VOL_40 * cellcount / 10))
    {
        soc = 40;
    }
    else if (lt_meter.voltage > (CELL_VOL_30 * cellcount / 10))
    {
        soc = 30;
    }
    else if (lt_meter.voltage > (CELL_VOL_20 * cellcount / 10))
    {
        soc = 20;
    }
    else
    {
        soc = 0;
    }

    // if(soc < lt_meter.soc)
    {
        lt_meter.soc = soc;
    }
}
#endif
