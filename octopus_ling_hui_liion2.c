
/*******************************************************************************
 * INCLUDES
 */
#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "octopus_ling_hui_liion2.h"
#include "octopus_uart_hal.h"
#include "octopus_uart_ptl_2.h" // Include UART protocol header
#include "octopus_carinfor.h"

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */
#ifdef TASK_MANAGER_STATE_MACHINE_LING_HUI_LIION2
/*******************************************************************************
 * MACROS
 */
#define PTL_LHLL_I2C_HEADER  (0x01)          //(ICU -> CTRL)FRAME HEADER
#define PTL_LHLL_C2I_HEADER  (0x02)          //(CTRL -> ICU)FRAME HEADER 
#define PTL_LHLL_FRAME_HEADER_SIZE (4)
#define PTL_LHLL_FRAME_MIN_SIZE (4)
/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
uint8_t lhl2_ptl_checksum(uint8_t* data, uint8_t len);
bool lhl2_ptl_receive_handler(ptl_2_proc_buff_t *ptl_2_proc_buff);
void lhl2_ptl_tx_process(void);   
void lhl2_ptl_proc_valid_frame(uint8_t* data, uint16_t len);
/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * STATIC VARIABLES
 */
static uint32_t lhl2_task_interval_ms = 0;
static uint32_t lhl2_task_tx_interval_ms = 0;
/*******************************************************************************
 * EXTERNAL VARIABLES
 */

/*******************************************************************************
 *  GLOBAL FUNCTIONS IMPLEMENTATION
 */

void task_lhl2_ptl_init_running(void)
{
	OTMS(TASK_MODULE_LING_HUI_LIION2, OTMS_S_INVALID);
	LOG_LEVEL("task_bafang_ptl_init_running\r\n");
}

void task_lhl2_ptl_start_running(void)
{
	LOG_LEVEL("task_bafang_ptl_start_running\r\n");
	ptl_2_register_module(PTL2_MODULE_LING_HUI_LIION2, lhl2_ptl_receive_handler);
	OTMS(TASK_MODULE_LING_HUI_LIION2, OTMS_S_ASSERT_RUN);
}

void task_lhl2_ptl_assert_running(void)
{
  OTMS(TASK_MODULE_LING_HUI_LIION2, OTMS_S_RUNNING);
}

void task_lhl2_ptl_running(void)
{
	if (GetTickCounter(&lhl2_task_interval_ms) < 10)
	{
			return;
	}
	StartTickCounter(&lhl2_task_interval_ms);

	OTMS(TASK_MODULE_LING_HUI_LIION2, OTMS_S_POST_RUN);

	lhl2_ptl_tx_process();
}

void task_lhl2_ptl_post_running(void)
{
  OTMS(TASK_MODULE_LING_HUI_LIION2, OTMS_S_ASSERT_RUN);
}

void task_lhl2_ptl_stop_running(void)
{
  LOG_LEVEL("_stop_running\r\n");
  OTMS(TASK_MODULE_LING_HUI_LIION2, OTMS_S_INVALID);
}

/*******************************************************************************
 * LOCAL FUNCTIONS IMPLEMENTATION
 */

void lhl2_ptl_tx_process(void)   
{
  uint8_t lu_tx_buff[32];
	
	if (GetTickCounter(&lhl2_task_tx_interval_ms) < 80)
			return;
	StartTickCounter(&lhl2_task_tx_interval_ms);

	lu_tx_buff[0] = PTL_LHLL_I2C_HEADER;
	lu_tx_buff[1] = 20;
	lu_tx_buff[2] = 0x01;
	//驱动方式
	
	lu_tx_buff[3] = lt_carinfo_indicator.drive_mode;

	//助力档位
	uint8_t pas = 0;
	if (lt_carinfo_meter.max_gear_level == SETTING_MAX_PAS_3_LEVEL)
	{

			switch (lt_carinfo_meter.gear)
			{
			case 0:  pas = 0; break;
			case 1:  pas = 5; break;
			case 2:  pas = 10; break;
			case 3:  pas = 15; break;
			default: pas = 5; break;
			}
	}
	else if (lt_carinfo_meter.max_gear_level == SETTING_MAX_PAS_5_LEVEL)
	{
			switch (lt_carinfo_meter.gear)
			{
			case 0:  pas = 0; break;
			case 1:  pas = 3; break;
			case 2:  pas = 6; break;
			case 3:  pas = 9; break;
			case 4:  pas = 12; break;
			case 5:  pas = 15; break;
			default: pas = 3; break;
			}
	}
	else if (lt_carinfo_meter.max_gear_level == SETTING_MAX_PAS_9_LEVEL)
	{
			switch (lt_carinfo_meter.gear)
			{
			case 0:  pas = 0; break;
			case 1:  pas = 1; break;
			case 2:  pas = 3; break;
			case 3:  pas = 5; break;
			case 4:  pas = 7; break;
			case 5:  pas = 9; break;
			case 6:  pas = 11; break;
			case 7:  pas = 13; break;
			case 8:  pas = 14; break;
			case 9:  pas = 15; break;
			default: pas = 1; break;
			}
	}
	lu_tx_buff[4] = pas;


	//控制器控制设定1
	uint8_t flag = 0x00;
	if (true) { flag |= BIT_7; }
	
	if (lt_carinfo_indicator.start_mode) { flag |= BIT_6; }
	if (lt_carinfo_indicator.highBeam) { flag |= BIT_5; }  //--发送大灯
	
	if (task_carinfo_has_error_code()) { flag |= BIT_4; }    //--发送通讯故障

  if (lt_carinfo_indicator.horn) { flag |= BIT_3; }    //--长按鸣笛键，鸣笛，，瑞碌

	if (lt_carinfo_indicator.rightTurn) { flag |= BIT_2; }      //--改右转向，瑞碌

	if (lt_carinfo_indicator.leftTurn) { flag |= BIT_1; }  	 //--改左转向，瑞碌
	

	if (lt_carinfo_indicator.cruise_control) {flag |= BIT_0; }  //定速巡航模式 true:启动定速巡航
	lu_tx_buff[5] = flag;
									
	//测速磁钢数：（单位：磁钢片数）
	lu_tx_buff[6] = lt_carinfo_indicator.motor_poles;

	//轮径:（单位:0.1英寸）
	uint16_t radius = lt_carinfo_meter.wheel_diameter;
	lu_tx_buff[7] = MK_MSB(radius);
	lu_tx_buff[8] = MK_LSB(radius);

	//助力灵敏度
	lu_tx_buff[9] = lt_carinfo_indicator.start_poles;    //1

	//助力启动强度
	lu_tx_buff[10] = lt_carinfo_indicator.start_mode;

	//内测速磁钢数
	lu_tx_buff[11] = 0;
	
	//限速值:(单位：km/h)
	lu_tx_buff[12] = lt_carinfo_meter.speed_limit;

	//控制器限流值
	lu_tx_buff[13] = lt_carinfo_battery.max_range;

	//控制器欠压值:(单位：0.1V)
	uint16_t vol = lt_carinfo_battery.voltage;
									
	lu_tx_buff[14] = MK_MSB(vol);
	lu_tx_buff[15] = MK_LSB(vol);

	//转把调速PWM
	lu_tx_buff[16] = MK_MSB(lt_carinfo_battery.throttle_pwm);
	lu_tx_buff[17] = MK_LSB(lt_carinfo_battery.throttle_pwm);
 
	flag = 0x00;           
								 //控制器控制设定 2 +助力磁钢盘磁钢个数   
	if (lt_carinfo_indicator.cruise_control) { flag |= BIT_6; }  //巡航状态  
	
	if (lt_carinfo_indicator.drive_mode) { flag |= BIT_4; }  //0:后驱；1：双驱    

	lu_tx_buff[18] = flag|0x06;    //助力磁钢数：6 


	lu_tx_buff[19] = lhl2_ptl_checksum(lu_tx_buff,19);

	ptl_2_send_buffer(PTL2_MODULE_LING_HUI_LIION2,lu_tx_buff, 20);
}

void lhl2_ptl_remove_none_header_data(ptl_2_proc_buff_t *ptl_2_proc_buff)
{
    if (ptl_2_proc_buff->buffer[0] == PTL_LHLL_C2I_HEADER)
    {
        return;
    }

    for (uint16_t i = 0; i < ptl_2_proc_buff->size; i++)
    {
        if (ptl_2_proc_buff->buffer[i] == PTL_LHLL_C2I_HEADER)
        {
            // remove data before header
            for (uint16_t j = i; j < ptl_2_proc_buff->size; j++)
            {
                ptl_2_proc_buff->buffer[j - i] = ptl_2_proc_buff->buffer[j];
            }
            ptl_2_proc_buff->size -= i;
            break;
        }
    }

    //no find A2M_PTL_HEADER,clear all;
    ptl_2_proc_buff->size = 0;
}

bool lhl2_ptl_find_valid_frame(ptl_2_proc_buff_t *ptl_2_proc_buff)
{
    uint16_t offset = 0; // hdr offset
    //uint8_t datalen = 0;
    uint8_t framelen = 0;        // frame len
    uint8_t crc;
    uint8_t crc_read = 0;
    bool frame_crc_ok = false;
    bool find = false;
    uint16_t next_valid_offset = 0;
    bool header_invalid = false;

    for (uint16_t i = 0; i < ptl_2_proc_buff->size; i++)
    {
        if (ptl_2_proc_buff->buffer[i] == PTL_LHLL_C2I_HEADER)
        {
            offset = i;
            framelen = ptl_2_proc_buff->buffer[i + 1];
            //datalen = framelen - PTL_LHLL_FRAME_HEADER_SIZE;
            // min size is PTL_FRAME_MIN_SIZE
            if ((framelen < PTL_LHLL_FRAME_MIN_SIZE) && (ptl_2_proc_buff->size >= PTL_LHLL_FRAME_MIN_SIZE) && (offset == 0))
            {
                next_valid_offset = PTL_LHLL_FRAME_MIN_SIZE;
            }
            else if ((framelen >= PTL_LHLL_FRAME_MIN_SIZE) && (framelen <= (ptl_2_proc_buff->size - offset)))
            {
                //head checksum
                crc = lhl2_ptl_checksum(&ptl_2_proc_buff->buffer[offset], framelen - 1);
               
                crc_read = ptl_2_proc_buff->buffer[offset + framelen - 1];
                frame_crc_ok = (crc == crc_read);

                if (frame_crc_ok)
                {
                    next_valid_offset = offset + framelen;
                    find = true;
                    break;
                }
                else
                {
                    // the first header is invalid
                    if ((offset == 0) || (header_invalid == true))
                    {
                        header_invalid = true;
                        next_valid_offset = framelen + offset;
                    }
                }
            }
            else
            {
                // no enough length, search next
            }
        }
    }

    if (find == true)
    {
        lhl2_ptl_proc_valid_frame(ptl_2_proc_buff->buffer + offset, framelen);
    }

    // todo, get timeout, if timeout, clear all data

    if (next_valid_offset != 0)
    {
        for (uint16_t i = next_valid_offset; i < ptl_2_proc_buff->size; i++)
        {
            ptl_2_proc_buff->buffer[i - next_valid_offset] = ptl_2_proc_buff->buffer[i];
        }

        ptl_2_proc_buff->size = ptl_2_proc_buff->size - next_valid_offset;
    }
    return find;
}


//#define  YONGJIU_WHEEL_Inch   284    //284  ,  700C50C,700C的车圈直径为622毫米,理论直径?：622mm+(50mm×2)=722mm（约28.4英寸）

// data is payload, len is payload length
void lhl2_ptl_proc_valid_frame(uint8_t* data, uint16_t len)   //RX
{
    uint8_t state1 = data[3];
    
    uint8_t state2 = data[4];
    uint16_t cur = MK_WORD(data[5], data[6]);
    //uint8_t cur_pre = data[7];
    uint16_t round = MK_WORD(data[8], data[9]);
    uint8_t soc = data[10];
    uint8_t range = MK_WORD(data[11], data[12]);
   
    //实际电流数据
    uint32_t cur_tmp = cur & 0x3FFF;

    //判断是否为1A单位
    if (0 == (cur & 0x4000))
    {
        cur_tmp = cur_tmp * 10;
    }
   
    
    lt_carinfo_battery.current = cur_tmp;
    lt_carinfo_battery.power = lt_carinfo_battery.voltage * lt_carinfo_battery.current / 100;
    lt_carinfo_battery.soc = soc;

		if (range)
		{
		  lt_carinfo_battery.range = range;  
		}
		else
		{
		  lt_carinfo_battery.range = (lt_carinfo_battery.max_range * soc) / 100;
		}
		
    if (round <= 10 || round >= 3500) //如果一圈的时间大于等于30S，认为车辆已经停止
    {
        lt_carinfo_meter.rpm = 0;
        lt_carinfo_meter.speed = 0;
    }
    else
    {		
        uint16_t rpm = 60.0 * 1000 / round;   //245
        double radius = lt_carinfo_meter.wheel_diameter; //轮毂半径，单位：米  //0.33
        double w = rpm * (2.0 * PI_FLOAT / 60.0);//转换角速度，单位：弧度/秒  //25
        double v = w * radius; //线速度,单位：米/秒   //8.25
        double kph = (v * 3600.0 / 1000.0) * 10;   //297
        if (kph > 990)   //theSettingInfo.speed_limit
        {
            kph = 990;
        }
		
        lt_carinfo_meter.rpm = (uint32_t)rpm;

		double speed = kph * 1.618;  //实物调试修正
		
		lt_carinfo_meter.speed = (uint32_t)speed;  
    }
    
    //6KM巡航状态
    lt_carinfo_indicator.walk_assist = (state1 & BIT_7) ? 1 : 0;     //------ 接收车机来的助推   
    //巡航状态
    lt_carinfo_indicator.cruise_control = (state1 & BIT_2) ? 1 : 0;
    //车辆水平状态
    lt_carinfo_indicator.horizontal_position = (state2 & BIT_7) ? 1 : 0;
    //断电刹把
    lt_carinfo_indicator.brake = (state2 & BIT_5) ? 1 : 0;
    //充电状态
    lt_carinfo_battery.rel_charge_state = (state2 & BIT_3) ? 1 : 0;

    lt_carinfo_indicator.ready = (state2 & BIT_6) ? 1 : 0;    //---ready灯

	//mingnuo_4chin
	if(((state1 & BIT_6) ==0)&&((state1 & BIT_5) ==0)&&((state1 & BIT_4) ==0)	 \
		&&((state1 & BIT_3) ==0)&&((state1 & BIT_0) ==0)&&((state2 & BIT_6) ==0)   \
		&&((state2 & BIT_4) ==0))
	{
		 return;
	}

	if (state1 & BIT_6)    //霍尔传感器状态
	{
		task_carinfo_add_error_code(ERROR_CODE_HALLSENSOR_ABNORMALITY);
	}

	//转把故障状态
	if (state1 & BIT_5)
	{
			task_carinfo_add_error_code(ERROR_CODE_THROTTLE_HALLSENSOR_ABNORMALITY);
	}

	//控制器故障状态
	if (state1 & BIT_4)
	{
			task_carinfo_add_error_code(ERROR_CODE_CONTROLLER_ABNORMALITY);
	}

	//欠压保护状态
	if (state1 & BIT_3)
	{
			task_carinfo_add_error_code(ERROR_CODE_LOW_VOLTAGE_PROTECTION);
	}	
    //电机缺相
  if (state1 & BIT_0)
  {
     task_carinfo_add_error_code(ERROR_CODE_MOTOR_ABNORMALITY);
  }
	
  //助力传感器状态
  if (state2 & BIT_6)
  {
    if (lt_carinfo_error.sensorFault)
    {
       task_carinfo_add_error_code(ERROR_CODE_ASSIST_POWER_SENSOR_ABNORMALITY);
    }
  }
  //通讯故障
  if (state2 & BIT_4)
  {
      task_carinfo_add_error_code(ERROR_CODE_BMS_ABNORMALITY);
  }	
}

uint8_t lhl2_ptl_checksum(uint8_t* data, uint8_t len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum ^= data[i];
    }
    return sum;
}

bool lhl2_ptl_receive_handler(ptl_2_proc_buff_t *ptl_2_proc_buff)
{
    lhl2_ptl_remove_none_header_data(ptl_2_proc_buff);
    return lhl2_ptl_find_valid_frame(ptl_2_proc_buff);	
}
#endif
