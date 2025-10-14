/******************************************************************************
 * File Name    : sif_driver.c
 * Author       : Octopus Embedded Systems Team
 * Description  : Custom SIF (Serial Interface Frame) communication driver
 *                for single-wire GPIO-based data transmission and reception.
 *
 *                This driver implements a proprietary single-wire serial
 *                communication protocol using precise timing control via
 *                GPIO and timer interrupts. It supports frame synchronization,
 *                bit-level encoding/decoding, checksum verification, and
 *                non-blocking send/receive states.
 *
 * Target MCU   : CH32 / STM32 / HK32 series
 * Dependency   : TIMx timer interrupt @ 50 µs interval
 *                GPIO configured for input/output (open-drain or push-pull)
 *
 * Protocol Summary:
 *    - Single wire interface
 *    - Synchronization frame:
 *         Low: ~30 ms, High: ~1 ms
 *    - Bit encoding (per bit ≈ 1.5 ms total):
 *         Logic '0' → Low 1.0 ms, High 0.5 ms
 *         Logic '1' → Low 0.5 ms, High 1.0 ms
 *    - Frame end: Low 5 ms
 *
 * Example timing (oscilloscope):
 *    [Sync] ___‾‾‾ | 0 | 1 | 0 | 1 | ___ (End)
 *
 * Created on   : 2024-07-18
 * Updated on   : 2025-10-10
 *****************************************************************************/

/******************************************************************************/
/* Header file contains */

#include "octopus_sif.h"
#include "octopus_gpio_hal.h"
#include "octopus_system.h"

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */
#ifdef TASK_MANAGER_STATE_MACHINE_SIF
/*==============================================================================
 *                                CONSTANTS
 *============================================================================*/

#define SIF_LOW   0   ///< Logic low level
#define SIF_HIGH  1   ///< Logic high level

/*------------------- Receive Timing Parameters -------------------*/

// Synchronization low period: 30 ms = 30000 µs / 50 µs = 600 counts
#define SIF_REV_SYNC_L_TIME_NUM       500

// Synchronization high period (acceptable range)
#define SIF_REV_SYNC_H_TIME_NUM_MIN   18  ///< Min 900 µs (1000 µs - 100 µs)
#define SIF_REV_SYNC_H_TIME_NUM_MAX   42  ///< Max 2100 µs (1000 µs + 1100 µs)

// Full logic cycle: ~3 ms ± 10%
#define SIF_REV_LOGIC_CYCLE_NUM_MIN   54  ///< Min 2700 µs / 50 µs = 54
#define SIF_REV_LOGIC_CYCLE_NUM_MAX   66  ///< Max 3300 µs / 50 µs = 66

// Half logic cycle: ~1.5 ms ± 10%
#define SIF_REV_HALF_LOGIC_CYCLE_MIN  27  ///< Min 1350 µs / 50 µs = 27
#define SIF_REV_HALF_LOGIC_CYCLE_MAX  33  ///< Max 1650 µs / 50 µs = 33

// End signal low level period after one complete frame (~5 ms)
#define SIF_REV_END_SIGNAL_TIME_NUM   60

// Bit and data configuration
#define SIF_REV_BIT_NUM               8    ///< 8 bits per byte
#define SIF_REV_DATA_NUM              12   ///< Maximum 12 bytes per frame

/*------------------- Send Timing Parameters -------------------*/

// Sync pulse for transmission: 30 ms low + 0.5 ms high
#define SIF_SEND_SYNC_L_TIME_NUM      600  ///< 30 ms
#define SIF_SEND_SYNC_H_TIME_NUM      10   ///< 0.5 ms

// Bit timing definitions
#define SIF_SEND_SHORT_TIME_NUM       10   ///< 0.5 ms
#define SIF_SEND_LONG_TIME_NUM        20   ///< 1.0 ms

#define SIF_SEND_BIT_NUM              8    ///< 8 bits per byte
#define SIF_SEND_DATA_MAX_NUM         12   ///< Maximum bytes per frame

/*==============================================================================
 *                                ENUMERATIONS
 *============================================================================*/

/**
 * @brief SIF receive state machine states
 */
typedef enum
{
    SIF_REV_STATE_INITIAL     = 0,  ///< Initial state, waiting for sync
    SIF_REV_STATE_SYNC_L      = 1,  ///< Receiving sync low period
    SIF_REV_STATE_SYNC_H      = 2,  ///< Receiving sync high period
    SIF_REV_STATE_REV_BIT     = 3,  ///< Receiving data bits
    SIF_REV_STATE_BUILD_DATA  = 4,  ///< Building received bytes
    SIF_REV_STATE_END_SIGNAL  = 5,  ///< Receiving end-of-frame signal
    SIF_REV_STATE_IDLE_SIGNAL = 6,  ///< Idle line detected
    SIF_REV_STATE_RESTART     = 7,  ///< Restart reception
    SIF_REV_STATE_STOP        = 8   ///< Reception complete or aborted
} SIF_REV_STATE_T;

/**
 * @brief SIF send state machine states
 */
typedef enum
{
    SIF_SEND_STATE_INITIAL     = 0, ///< Initial state, waiting for send request
    SIF_SEND_STATE_SYNC_L      = 1, ///< Transmitting sync low signal
    SIF_SEND_STATE_SYNC_H      = 2, ///< Transmitting sync high signal
    SIF_SEND_STATE_GET_BIT     = 3, ///< Load next bit to transmit
    SIF_SEND_STATE_BIT_L       = 4, ///< Output low period of current bit
    SIF_SEND_STATE_BIT_H       = 5, ///< Output high period of current bit
    SIF_SEND_STATE_CHECK_BIT   = 6, ///< Check bit timing consistency
    SIF_SEND_STATE_END_SIGNAL  = 7  ///< Transmitting end-of-frame low signal
} SIF_SEND_STATE_T;

/*==============================================================================
 *                              GLOBAL VARIABLES
 *=============================================================================*/

/*----------------------------- Receive Variables -----------------------------*/
uint32_t SIF_receive_H_L_Level_time_cnt = 0;           ///< Timing counter for H/L signal durations
uint8_t  SIF_receive_start_H_L_Level_timming_flag = 0; ///< Timing start flag

uint8_t  SIF_receive_state = SIF_REV_STATE_IDLE_SIGNAL; ///< Current receive state
uint8_t  SIF_receive_bit_num = 0;                       ///< Bit counter for current byte
uint8_t  SIF_receive_data_num = 0;                      ///< Byte counter for received data

uint8_t  SIF_receive_data_buf[SIF_REV_DATA_NUM] = {0};  ///< Temporary buffer during reception
uint8_t  SIF_receive_buf[SIF_REV_DATA_NUM] = {0};       ///< Final buffer after frame completion

uint8_t  SIF_receive_has_read_bit = 0;                  ///< 1 = bit already read
uint8_t  SIF_receive_check_OK = 0;                      ///< 1 = checksum valid
uint8_t  SIF_receive_read_success = 0;                  ///< 1 = frame successfully received

/*------------------- Send Variables -------------------*/
uint32_t SIF_send_H_L_Level_time_cnt = 0;               ///< Timing counter for send bit waveform
uint8_t  SIF_send_start_H_L_Level_timming_flag = 0;     ///< Start timing flag

uint8_t  SIF_send_state = 0;                            ///< Send state machine status
uint8_t  SIF_send_bit_num = 0;                          ///< Bit counter
uint8_t  SIF_send_data_num = 0;                         ///< Sent byte counter
uint8_t  SIF_send_bit = 0;                              ///< Current bit value being transmitted

uint8_t  SIF_send_data_buf[SIF_SEND_DATA_MAX_NUM] = {0};///< Transmit data buffer
uint8_t  SIF_send_data_num_target = 0;                  ///< Total bytes to be sent
uint8_t  SIF_send_req_flag = 0;                         ///< Send request flag

static bool SIF_has_inited = false;                     ///< Initialization flag

/*==============================================================================
 *                         FUNCTION DECLARATIONS (STATIC)
 *============================================================================*/

/**
 * @brief Initialize SIF GPIO pin(s)
 */
static void SIF_GPIO_Init(void);

/**
 * @brief Read SIF input GPIO level (0 = Low, 1 = High)
 * @return uint8_t Current logic level
 */
static uint8_t SIF_Get_Input_Pin_Data(void);

/**
 * @brief Receive data handler (state machine)
 */
static void SIF_Receive_Data_Handle(void);

/**
 * @brief Send data handler (state machine)
 */
static void SIF_Send_Data_Handle(void);

/**
 * @brief Frame checksum calculation and validation
 */
static void SIF_Checksum_Handle(void);

/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/
/**
 * @brief Initialize the SIF interface (GPIO + timer setup).
 */
void otsm_sif_init(void)
{
    if (!SIF_has_inited)
    {
        SIF_GPIO_Init();
        // SIF_TIM2_Int_Init(50 - 1, 96 - 1); // Timer config (50us)
        SIF_has_inited = true;
        LOG_LEVEL("sif init\r\n");
    }
}

/**
 * @brief Check if the SIF module has been initialized.
 */
bool sif_is_init(void)
{
    return SIF_has_inited;
}

/**
 * @brief Initialize the SIF GPIO line.
 *        Sets data output line to low initially.
 */
void SIF_GPIO_Init(void)
{
    SIF_SEND_DATA_BIT_LOW();
}

/**
 * @brief Deinitialize the SIF module, disabling timer interrupts.
 */
void sif_de_init(void)
{
    if (SIF_has_inited)
    {
        // TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
        // TIM_Cmd(TIM2, DISABLE);
        SIF_has_inited = false;
    }
}

/**
 * @brief Restart SIF receiver state machine.
 */
void SIF_ReStart(void)
{
    SIF_receive_state = SIF_REV_STATE_INITIAL;
}

/**
 * @brief Check if SIF is in idle state.
 */
bool sif_is_idle(void)
{
    return (SIF_receive_state == SIF_REV_STATE_STOP);
}

/**
 * @brief Delay for a specified number of microseconds using 50 µs tick counter
 * @param delay_us Desired delay in microseconds
 */
void sif_delay_50_us(uint32_t delay_us)
{
    if (delay_us == 0)
        return;

    // Convert to 50 µs ticks (round up)
    uint32_t ticks = (delay_us + 49) / 50;  // 向上取整，确保不小于目标时间
    uint32_t start = system_timer_tick_50us;
		
		//LOG_LEVEL("start test sif_delay_50_us %u\r\n",system_timer_tick_50us);
    while ((system_timer_tick_50us - start) < ticks)
    {
        // busy wait
			__NOP();
    }
		//LOG_LEVEL("start test sif_delay_50_us %u\r\n",system_timer_tick_50us);
}

/**
 * @brief Main SIF interrupt handler.
 *        This should be called every 50us by timer interrupt.
 */
void SIF_IO_IRQHandler(void)
{
    /* Handle receiver timing */
    if (SIF_receive_start_H_L_Level_timming_flag)
        SIF_receive_H_L_Level_time_cnt++;

    SIF_Receive_Data_Handle();  // Process RX state machine

    /* Handle sender timing */
#if 1
    if (SIF_send_start_H_L_Level_timming_flag)
        SIF_send_H_L_Level_time_cnt++;

    SIF_Send_Data_Handle(); // Process TX state machine
#endif
}

/**
 * @brief Process received waveform and build bytes.
 *
 * State machine overview:
 *  - INITIAL: Wait for sync low.
 *  - SYNC_L:  Measure sync low duration.
 *  - SYNC_H:  Measure sync high duration.
 *  - REV_BIT: Decode bits based on pulse width.
 *  - BUILD_DATA: Assemble bytes.
 *  - END_SIGNAL: Detect frame end.
 *  - RESTART/IDLE: Reset and prepare for next frame.
 */
void SIF_Receive_Data_Handle(void)
{
    switch (SIF_receive_state)
    {
        /* Initial: wait for sync low pulse */
        case SIF_REV_STATE_INITIAL:
            if (SIF_Get_Input_Pin_Data() == SIF_LOW)
            {
                SIF_receive_bit_num = 0;
                SIF_receive_data_num = 0;
                SIF_receive_H_L_Level_time_cnt = 0;
                SIF_receive_start_H_L_Level_timming_flag = 1;
                SIF_receive_state = SIF_REV_STATE_SYNC_L;
                memset(SIF_receive_data_buf, 0, SIF_REV_DATA_NUM);
            }
            break;

        /* Detect sync low → high transition */
        case SIF_REV_STATE_SYNC_L:
            if (SIF_Get_Input_Pin_Data() == SIF_HIGH)
            {
                if (SIF_receive_H_L_Level_time_cnt >= SIF_REV_SYNC_L_TIME_NUM)
                {
                    SIF_receive_H_L_Level_time_cnt = 0;
                    SIF_receive_state = SIF_REV_STATE_SYNC_H;
                }
                else
                {
                    SIF_receive_state = SIF_REV_STATE_IDLE_SIGNAL;
                }
            }
            break;

        /* Detect sync high → low transition */
        case SIF_REV_STATE_SYNC_H:
            if (SIF_Get_Input_Pin_Data() == SIF_LOW)
            {
                if (SIF_receive_H_L_Level_time_cnt >= SIF_REV_SYNC_H_TIME_NUM_MIN &&
                    SIF_receive_H_L_Level_time_cnt <= SIF_REV_SYNC_H_TIME_NUM_MAX)
                {
                    SIF_receive_H_L_Level_time_cnt = 0;
                    SIF_receive_has_read_bit = 0;
                    SIF_receive_state = SIF_REV_STATE_REV_BIT;
                }
                else
                {
                    SIF_receive_state = SIF_REV_STATE_IDLE_SIGNAL;
                }
            }
            else if (SIF_receive_H_L_Level_time_cnt >= SIF_REV_LOGIC_CYCLE_NUM_MAX)
            {
                SIF_receive_state = SIF_REV_STATE_IDLE_SIGNAL;
            }
            break;

        /* Decode bit based on pulse length */
        case SIF_REV_STATE_REV_BIT:
            if (SIF_Get_Input_Pin_Data() == SIF_HIGH)
            {
                if (SIF_receive_H_L_Level_time_cnt >= SIF_REV_LOGIC_CYCLE_NUM_MAX)
                {
                    SIF_receive_state = SIF_REV_STATE_IDLE_SIGNAL;
                }
                else if (!SIF_receive_has_read_bit)
                {
                    if (SIF_receive_H_L_Level_time_cnt < SIF_REV_HALF_LOGIC_CYCLE_MIN)
                        SIF_receive_data_buf[SIF_receive_data_num] |= 0x01;

                    SIF_receive_has_read_bit = 1;
                    SIF_receive_state = SIF_REV_STATE_BUILD_DATA;
                }
            }
            else if (SIF_receive_H_L_Level_time_cnt >= SIF_REV_LOGIC_CYCLE_NUM_MAX)
            {
                SIF_receive_state = SIF_REV_STATE_IDLE_SIGNAL;
            }
            break;

        /* Build bytes from bits */
        case SIF_REV_STATE_BUILD_DATA:
            if (SIF_Get_Input_Pin_Data() == SIF_LOW)
            {
                if (SIF_receive_has_read_bit)
                {
                    SIF_receive_H_L_Level_time_cnt = 0;
                    SIF_receive_has_read_bit = 0;
                    SIF_receive_bit_num++;

                    if (SIF_receive_bit_num == SIF_REV_BIT_NUM)
                    {
                        SIF_receive_data_num++;
                        SIF_receive_bit_num = 0;

                        if (SIF_receive_data_num == SIF_REV_DATA_NUM)
                            SIF_receive_state = SIF_REV_STATE_END_SIGNAL;
                        else
                            SIF_receive_state = SIF_REV_STATE_REV_BIT;
                    }
                    else
                    {
                        SIF_receive_data_buf[SIF_receive_data_num] <<= 1;
                        SIF_receive_state = SIF_REV_STATE_REV_BIT;
                    }
                }
            }
            else if (SIF_receive_H_L_Level_time_cnt >= SIF_REV_LOGIC_CYCLE_NUM_MAX)
            {
                SIF_receive_state = SIF_REV_STATE_IDLE_SIGNAL;
            }
            break;

        /* Detect end of frame */
        case SIF_REV_STATE_END_SIGNAL:
            if (SIF_Get_Input_Pin_Data() == SIF_LOW)
            {
                if (SIF_receive_H_L_Level_time_cnt >= SIF_REV_END_SIGNAL_TIME_NUM)
                {
                    memcpy(SIF_receive_buf, SIF_receive_data_buf, SIF_REV_DATA_NUM);
                    SIF_Checksum_Handle();
                    SIF_receive_read_success = 1;
                    SIF_receive_start_H_L_Level_timming_flag = 0;
                    SIF_receive_H_L_Level_time_cnt = 0;
                    SIF_receive_state = SIF_REV_STATE_INITIAL;
                }
            }
            else if (SIF_receive_H_L_Level_time_cnt >= SIF_REV_LOGIC_CYCLE_NUM_MAX)
            {
                SIF_receive_state = SIF_REV_STATE_IDLE_SIGNAL;
            }
            break;

        /* Idle and restart handling */
        case SIF_REV_STATE_IDLE_SIGNAL:
        case SIF_REV_STATE_RESTART:
            SIF_receive_start_H_L_Level_timming_flag = 0;
            SIF_receive_H_L_Level_time_cnt = 0;
            SIF_receive_has_read_bit = 0;
            SIF_receive_bit_num = 0;
            SIF_receive_data_num = 0;
            SIF_receive_state = SIF_REV_STATE_INITIAL;
            break;

        default:
            break;
    }
}
/********************************************************************************
 * @brief  从接收缓存区读取完整的一帧 SIF 数据
 * @param  data   外部传入的接收缓冲区指针
 * @param  maxlen 最多读取的字节数
 * @retval 实际读取的字节数；若未接收到完整数据则返回 0
 *******************************************************************************/
uint8_t sif_read_data(uint8_t *data, uint8_t maxlen)
{
	MY_ASSERT(data); // 参数断言
	if (!SIF_has_inited)
	{
		return 0; // 未初始化直接返回
	}

	// 检查接收完成并通过校验
	if (SIF_receive_read_success && SIF_receive_check_OK)
	{
		// 限制拷贝长度不超过缓冲区
		uint8_t len = maxlen < SIF_REV_DATA_NUM ? maxlen : SIF_REV_DATA_NUM;
		for (int i = 0; i < len; i++)
		{
			data[i] = SIF_receive_buf[i];
		}
		SIF_receive_read_success = false; // 清除读取标志，防止重复读
		return len;						// 返回拷贝长度
	}
	return 0; // 尚未有新数据可读
}

/********************************************************************************
 * @brief  SIF 同步方式发送函数（阻塞式）
 * @param  data  发送的数据指针
 * @param  len   数据长度
 * @retval 实际发送的字节数
 * @note   使用同步延时方式发送，一般用于调试或时序验证
 *******************************************************************************/
uint8_t SIF_SendData_Sync(uint8_t *data, uint8_t len)
{
	MY_ASSERT(data);
	MY_ASSERT(len);

	if (!SIF_has_inited)
	{
		return 0;
	}

	uint8_t byte = 0;
	uint8_t flag = 0;

	// 发送同步信号：低电平 20ms + 高电平 0.5ms
	SIF_SEND_DATA_BIT_LOW();
	sif_delay_50_us(20000);
	SIF_SEND_DATA_BIT_HIGH();
	sif_delay_50_us(500);

	// 循环发送每个字节
	for (int i = 0; i < len; i++)
	{
		byte = data[i];
		// 循环发送每一位
		for (int j = 0; j < 8; j++)
		{
			flag = byte & (0x80 >> j); // 提取当前 bit
			if (flag)
			{
				// 发送逻辑“1” ：低 500us + 高 1000us
				SIF_SEND_DATA_BIT_LOW();
				sif_delay_50_us(500);
				SIF_SEND_DATA_BIT_HIGH();
				sif_delay_50_us(1000);
			}
			else
			{
				// 发送逻辑“0” ：低 1000us + 高 500us
				SIF_SEND_DATA_BIT_LOW();
				sif_delay_50_us(1000);
				SIF_SEND_DATA_BIT_HIGH();
				sif_delay_50_us(500);
			}
		}
	}

	// 结束信号：低电平 5ms
	SIF_SEND_DATA_BIT_LOW();
	sif_delay_50_us(5000);
	return len;
}

/********************************************************************************
 * @brief  异步请求式发送接口
 * @param  data  要发送的数据缓冲区
 * @param  len   数据长度
 * @retval 0=发送忙或失败，>0=成功加入发送队列
 * @note   实际发送在 SIF_Send_Data_Handle() 中由状态机执行
 *******************************************************************************/
uint8_t SIF_SendData(uint8_t *data, uint8_t len)
{
	MY_ASSERT(data);
	MY_ASSERT(len);
	MY_ASSERT(len <= SIF_SEND_DATA_MAX_NUM);

	if (!SIF_has_inited)
	{
		return 0;
	}

	// 若当前未在发送，则可进入发送准备
	if (SIF_send_req_flag == false)
	{
		for (int i = 0; i < len; i++)
		{
			SIF_send_data_buf[i] = data[i]; // 拷贝待发送数据
		}
		SIF_send_data_num_target = len; // 保存目标长度
		SIF_send_req_flag = true;		  // 设置发送请求标志
		return len;
	}
	return 0; // 若正在发送，则直接返回 0
}

/********************************************************************************
 * @brief  发送状态机（需周期调用，比如在定时中断或主循环中）
 * @note   控制 SIF 的逐位时序输出，依赖定时计数器 SIF_send_H_L_Level_time_cnt
 *******************************************************************************/
void SIF_Send_Data_Handle(void)
{
	switch (SIF_send_state)
	{
	case SIF_SEND_STATE_INITIAL: // 初始状态：等待发送请求
		if (SIF_send_req_flag == true)
		{
			// 初始化计数和标志
			SIF_send_bit_num = 0;
			SIF_send_data_num = 0;
			SIF_send_H_L_Level_time_cnt = 0;
			SIF_send_start_H_L_Level_timming_flag = 1;
			SIF_send_state = SIF_SEND_STATE_SYNC_L; // 进入同步低电平阶段
			SIF_SEND_DATA_BIT_LOW();
		}
		break;

	case SIF_SEND_STATE_SYNC_L: // 发送同步低电平阶段
		if (SIF_send_H_L_Level_time_cnt >= SIF_SEND_SYNC_L_TIME_NUM)
		{
			SIF_send_H_L_Level_time_cnt = 0;
			SIF_SEND_DATA_BIT_HIGH();				 // 切换高电平
			SIF_send_state = SIF_SEND_STATE_SYNC_H; // 进入同步高阶段
		}
		break;

	case SIF_SEND_STATE_SYNC_H: // 发送同步高电平阶段
		if (SIF_send_H_L_Level_time_cnt >= SIF_SEND_SYNC_H_TIME_NUM)
		{
			SIF_send_H_L_Level_time_cnt = 0;
			SIF_SEND_DATA_BIT_LOW();
			SIF_send_state = SIF_SEND_STATE_GET_BIT; // 进入取 bit 状态
		}
		break;

	case SIF_SEND_STATE_GET_BIT: // 获取当前待发送 bit
		SIF_send_H_L_Level_time_cnt = 0;
		SIF_send_bit = SIF_send_data_buf[SIF_send_data_num] & (0x80 >> SIF_send_bit_num);
		SIF_send_state = SIF_SEND_STATE_BIT_L;
		break;

	case SIF_SEND_STATE_BIT_L: // 输出低电平阶段（按bit类型决定长短）
		if (SIF_send_bit)
		{
			if (SIF_send_H_L_Level_time_cnt >= SIF_SEND_SHORT_TIME_NUM)
			{
				SIF_send_H_L_Level_time_cnt = 0;
				SIF_SEND_DATA_BIT_HIGH();
				SIF_send_state = SIF_SEND_STATE_BIT_H;
			}
		}
		else
		{
			if (SIF_send_H_L_Level_time_cnt >= SIF_SEND_LONG_TIME_NUM)
			{
				SIF_send_H_L_Level_time_cnt = 0;
				SIF_SEND_DATA_BIT_HIGH();
				SIF_send_state = SIF_SEND_STATE_BIT_H;
			}
		}
		break;

	case SIF_SEND_STATE_BIT_H: // 输出高电平阶段
		if (SIF_send_bit)
		{
			if (SIF_send_H_L_Level_time_cnt >= SIF_SEND_LONG_TIME_NUM)
			{
				SIF_send_H_L_Level_time_cnt = 0;
				SIF_SEND_DATA_BIT_LOW();
				SIF_send_state = SIF_SEND_STATE_CHECK_BIT;
			}
		}
		else
		{
			if (SIF_send_H_L_Level_time_cnt >= SIF_SEND_SHORT_TIME_NUM)
			{
				SIF_send_H_L_Level_time_cnt = 0;
				SIF_SEND_DATA_BIT_LOW();
				SIF_send_state = SIF_SEND_STATE_CHECK_BIT;
			}
		}
		break;

	case SIF_SEND_STATE_CHECK_BIT: // 校验当前字节 bit 是否发送完毕
		SIF_send_bit_num++;
		if (SIF_send_bit_num == SIF_SEND_BIT_NUM)
		{
			SIF_send_data_num++;
			SIF_send_bit_num = 0;

			if (SIF_send_data_num == SIF_send_data_num_target)
			{
				SIF_send_state = SIF_SEND_STATE_END_SIGNAL; // 数据全部发完
			}
			else
			{
				SIF_SEND_DATA_BIT_LOW();
				SIF_send_state = SIF_SEND_STATE_GET_BIT; // 继续下一个字节
			}
		}
		else
		{
			SIF_SEND_DATA_BIT_LOW();
			SIF_send_state = SIF_SEND_STATE_GET_BIT; // 继续下一个 bit
		}
		break;

	case SIF_SEND_STATE_END_SIGNAL: // 发送结束信号
		SIF_send_req_flag = false;
		SIF_send_start_H_L_Level_timming_flag = 0;
		SIF_send_H_L_Level_time_cnt = 0;
		SIF_send_bit_num = 0;
		SIF_send_data_num = 0;
		SIF_send_state = SIF_SEND_STATE_INITIAL; // 回到初始状态
		break;
	}
}

/********************************************************************************
 * @brief  校验接收到的一帧数据（异或校验）
 *******************************************************************************/
void SIF_Checksum_Handle(void)
{
	uint8_t checkByte = 0;
	uint64_t checkXor = 0;

	// 从第 0~N-2 字节进行异或
	for (int i = 0; i < (SIF_REV_DATA_NUM - 1); i++)
	{
		checkXor ^= SIF_receive_buf[i];
	}

	checkByte = (uint8_t)checkXor;

	// 对比最后一个字节是否为校验结果
	if (checkByte == SIF_receive_buf[SIF_REV_DATA_NUM - 1])
	{
		SIF_receive_check_OK = 1; // 校验通过
	}
	else
	{
		SIF_receive_check_OK = 0; // 校验失败
	}
}

/********************************************************************************
 * @brief  读取 SIF 输入引脚状态（含简单消抖）
 * @retval 当前稳定的电平值（SIF_LOW 或 SIF_HIGH）
 *******************************************************************************/
uint8_t SIF_Get_Input_Pin_Data(void)
{
	static uint8_t flag = 0;  // 稳定计数
	static uint8_t value = 0; // 上次读值
	static uint8_t ret = 0;   // 返回值

	uint8_t cur = SIF_RECEIVE_DATA_BIT(); // 读取当前引脚状态

	if (value != cur)
	{
		flag = 3;  // 检测到变化则启动稳定计数
		value = cur;
	}
	if (flag)
	{
		flag--;
		if (flag == 0)
		{
			ret = value; // 当稳定若干次后才更新输出值
		}
	}
	return ret;
}

#endif
