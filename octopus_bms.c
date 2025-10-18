/********************************** (C) COPYRIGHT *******************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************
 */

/******************************************************************************/
/* Header file contains */

#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "octopus_bms.h"
#include "octopus_system.h"

/*******************************************************************************
 * DEBUG SWITCH MACROS
 */
#ifdef TASK_MANAGER_STATE_MACHINE_BMS

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// CONSTANTS
//
// 1ms interrupter
// sync ______________________________________________________________|--| 62ms+2ms
// 0 ____|-- 4ms+2ms
// 1 --|____ 2ms+4ms
//////////////////////////////////////////////////////////////////////////////////////////////////////////
#define BMS_LOW 0  // �͵�ƽ
#define BMS_HIGH 1 // �ߵ�ƽ

#define BMS_REV_SYNC_L_TIME_NUM 60 // ͬ���źŵ͵�ƽʱ�䣺62ms = 30000us / 1ms = 62

#define BMS_REV_SYNC_H_TIME_NUM_MIN 1 // ͬ���źŸߵ�ƽ��Сʱ��
#define BMS_REV_SYNC_H_TIME_NUM_MAX 2 // ͬ���źŸߵ�ƽ���ʱ��

// #define BMS_REV_SHORT_TIME_NUM_MIN      18      //һ���߼������ж̵�ʱ����Сֵ��1000-100us = 900us / 50us = 18
// #define BMS_REV_SHORT_TIME_NUM_MAX      22      //һ���߼������ж̵�ʱ�����ֵ��1000+100us = 1000us / 50us = 22

// #define BMS_REV_LONG_TIME_NUM_MIN       36      //һ���߼������г���ʱ����Сֵ��2ms-200us = 1800us / 50us = 36
// #define BMS_REV_LONG_TIME_NUM_MAX       44      //һ���߼������г���ʱ�����ֵ��2ms+200us = 2200us / 50us = 44

// #define BMS_REV_LOGIC_CYCLE_NUM_MIN     54      //һ���߼�������Сʱ�䣺3ms-300us = 2700us / 50us = 54
#define BMS_REV_LOGIC_CYCLE_NUM_MAX 12 // һ���߼��������ʱ�䣺3ms+300us = 3300us / 50us = 66

#define BMS_REV_HALF_LOGIC_CYCLE_MIN 5 // һ���߼����ڵ�1/2��Сʱ�䣺1.5ms-150us = 1350us / 50us = 27
// #define BMS_REV_HALF_LOGIC_CYCLE_MAX    33      //һ���߼����ڵ�1/2���ʱ�䣺1.5ms+150us = 1650us / 50us = 33

#define BMS_REV_END_SIGNAL_TIME_NUM 200 // �����źŵ�ƽʱ�䣺5ms�͵�ƽ + Nms�ߵ�ƽ��ʵ�ʼ��5ms�͵�ƽ���У�һ֡���ݷ�����ɺ���5ms�͵�ƽ�ʹ�������ˣ��������ݵ�ʱ����������������

#define BMS_REV_BIT_NUM 8 // ���յ�bitλ������1�ֽ�=8bit

#define BMS_REV_DATA_NUM 6 // ���յ����ݸ���

#define BMS_SEND_SYNC_L_TIME_NUM 600 // ͬ���źŵ͵�ƽʱ�䣺30ms = 30000us / 50us = 600
#define BMS_SEND_SYNC_H_TIME_NUM 10	 // ͬ���źŸߵ�ƽʱ�䣺500us = 500us / 50us = 10
#define BMS_SEND_SHORT_TIME_NUM 10	 // һ���߼������ж̵�ʱ�䣺500us = 500us / 50us = 10
#define BMS_SEND_LONG_TIME_NUM 20	 // һ���߼������г���ʱ�䣺1ms = 1000us / 50us = 20
#define BMS_SEND_BIT_NUM 8			 // ���͵�bitλ������1�ֽ�=8bit
#define BMS_SEND_DATA_MAX_NUM 12	 // ���͵�����������

/***************************************************************************************************
 *                                           GLOBAL VARIABLES
 ***************************************************************************************************/

/* ���Ͷ��� -------------------------------------------------------------------*/
typedef enum
{
	BMS_REV_STATE_INITIAL = 0,	  // ��ʼ״̬���ȴ�����ͬ���ź�
	BMS_REV_STATE_SYNC_L = 1,	  // ����ͬ���͵�ƽ�ź�״̬
	BMS_REV_STATE_SYNC_H = 2,	  // ����ͬ���ߵ�ƽ�ź�״̬
	BMS_REV_STATE_REV_BIT = 3,	  // ��ȡ�������ƽ״̬
	BMS_REV_STATE_BUILD_DATA = 4, // �����ֽ�����
	BMS_REV_STATE_END_SIGNAL = 5, // ���ս�����ƽ�ź�״̬
	BMS_REV_STATE_IDLE_SIGNAL = 6,
	BMS_REV_STATE_RESTART = 7, // ���չ��̳������½���״̬
	BMS_REV_STATE_STOP = 8
} BMS_REV_STATE_T; // ��������״̬ö��

typedef enum
{
	BMS_SEND_STATE_INITIAL = 0,	   // ��ʼ״̬���ȴ��������ݱ�־
	BMS_SEND_STATE_SYNC_L = 1,	   // ����ͬ���͵�ƽ�ź�״̬
	BMS_SEND_STATE_SYNC_H = 2,	   // ����ͬ���ߵ�ƽ�ź�״̬
	BMS_SEND_STATE_GET_BIT = 3,	   // ��ȡ����λ
	BMS_SEND_STATE_BIT_L = 4,	   // ����������͵�ƽ�ź�״̬
	BMS_SEND_STATE_BIT_H = 5,	   // ����������ߵ�ƽ�ź�״̬
	BMS_SEND_STATE_CHECK_BIT = 6,  // �������λ״̬
	BMS_SEND_STATE_END_SIGNAL = 7, // ���ͽ�����ƽ�ź�״̬

} BMS_SEND_STATE_T; // ��������״̬ö��

/* ���ձ������� -------------------------------------------------------------------*/
uint32_t BMS_receive_H_L_Level_time_cnt = 0;		  // �ߵ͵�ƽʱ�����
uint8_t BMS_receive_start_H_L_Level_timming_flag = 0; // ��ʼ�ߵ͵�ƽ��ʱ���

uint8_t BMS_receive_state = BMS_REV_STATE_IDLE_SIGNAL; // ��������״̬
uint8_t BMS_receive_bit_num = 0;					   // ���յ�bitλ����
uint8_t BMS_receive_data_num = 0;					   // ���յ����ݸ���

// �������ݻ�������-��һ���������������ݣ�51�������ֽ�
uint8_t BMS_receive_data_buf[BMS_REV_DATA_NUM] = {0};
uint8_t BMS_receive_buf[BMS_REV_DATA_NUM] = {0};

uint8_t BMS_receive_has_read_bit = 0; // 1-�Ѿ���ȡһ��bitλ
uint8_t BMS_receive_check_OK = 0;	  // 1-У�����ȷ��0-У���ʧ��
uint8_t BMS_receive_read_success = 0; // һ֡�����Ƿ��ȡ�ɹ���0-���ɹ���1-�ɹ�

#ifdef BMS_SEND_HANDLER
/* ���ͱ������� -------------------------------------------------------------------*/
uint32_t BMS_send_H_L_Level_time_cnt = 0;		   // �ߵ͵�ƽʱ�����
uint8_t BMS_send_start_H_L_Level_timming_flag = 0; // ��ʼ�ߵ͵�ƽ��ʱ���

uint8_t BMS_send_state = 0;	   // ��������״̬
uint8_t BMS_send_bit_num = 0;  // ���յ�bitλ����
uint8_t BMS_send_data_num = 0; // ���յ����ݸ���

uint8_t BMS_send_bit = 0; // 1-�Ѿ���ȡһ��bitλ

uint8_t BMS_send_data_buf[BMS_SEND_DATA_MAX_NUM] = {0};
uint8_t BMS_send_data_num_target = 0;
uint8_t BMS_send_req_flag = 0; // ���ڷ�������
#endif

static bool BMS_has_inited = false;
/*************************************************************************************************
 * GLOBAL FUNCTIONS DECLEAR
 */
/*************************************************************************************************/

static void BMS_GPIO_Init(void);			 // GPIO��ʼ������
static uint8_t BMS_Get_Input_Pin_Data(void); // ��ȡBMS���ն˿ڵ�ƽ������������
static void BMS_Receive_Data_Handle(void);	 // �������ݴ�������У׼λ��������������Ӧ
#ifdef BMS_SEND_HANDLER
static void BMS_Send_Data_Handle(void); // �������ݴ�������У׼λ��������������Ӧ
#endif
static void BMS_Checksum_Handle(void); // У��ʹ���

bool BMS_Is_Idle(void);
/***************************************************************************************************
 *                                            LOCAL FUNCTION
 ***************************************************************************************************/
void otsm_bms_init(void)
{
	if (!BMS_has_inited)
	{
		BMS_GPIO_Init();
		/// BMS_TIM2_Int_Init(50 - 1, 96 - 1);
		BMS_has_inited = true;
	}
}

bool BMS_IsInit(void)
{
	return BMS_has_inited;
}

void BMS_GPIO_Init(void)
{
	BMS_SEND_DATA_BIT_LOW();
}

void BMS_DeInit(void)
{
	if (BMS_has_inited)
	{
		/// TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
		/// TIM_Cmd(TIM2, DISABLE);
		BMS_has_inited = false;
	}
}

void BMS_ReStart()
{
	BMS_receive_state = BMS_REV_STATE_INITIAL;
}

bool BMS_Is_Idle(void)
{
	if (BMS_receive_state == BMS_REV_STATE_STOP)
		return 1;
	else
		return 0;
}

void BMS_Delay_us(uint32_t us)
{
	DELAY_US(us);
}

uint8_t BMS_ReadData(uint8_t *data, uint8_t maxlen)
{
	MY_ASSERT(data);
	if (!BMS_has_inited)
	{
		return 0;
	}

	if (BMS_receive_read_success && BMS_receive_check_OK)
	{
		uint8_t len = maxlen < BMS_REV_DATA_NUM ? maxlen : BMS_REV_DATA_NUM;
		for (int i = 0; i < len; i++)
		{
			data[i] = BMS_receive_buf[i];
		}
		BMS_receive_read_success = false;
		return len;
	}
	return 0;
}
/*********************************************************************
 * @fn      TIM2_IRQHandler
 *
 * @brief   This function handles TIM2 global interrupt request.
 *
 * @return  none
 */

void BMS_IO_IRQHandler(void)
{
	// BMS ���մ���
	if (BMS_receive_start_H_L_Level_timming_flag == 1) // ��ʼ�ߵ͵�ƽ��ʱ���
	{
		BMS_receive_H_L_Level_time_cnt++; // �ߵ͵�ƽά��ʱ���������
	}
	/////////////////////////////////////////////////////////////////////////////////////

	BMS_Receive_Data_Handle(); // �������ݴ���

/////////////////////////////////////////////////////////////////////////////////////
// BMS ���ʹ���
#ifdef BMS_SEND_HANDLER
	if (BMS_send_start_H_L_Level_timming_flag == 1)
	{
		BMS_send_H_L_Level_time_cnt++; // �ߵ͵�ƽά��ʱ���������
	}
	BMS_Send_Data_Handle(); // �������ݴ���
#endif
	/////////////////////////////////////////////////////////////////////////////////////
}

///__attribute__((section(".highcode")))
void BMS_Receive_Data_Handle(void)
{

	switch (BMS_receive_state) // ��⵱ǰ��������״̬
	{
	case BMS_REV_STATE_INITIAL:					 // ��ʼ״̬��δ���յ�ͬ����Ϣ������ͬ���ж�
		if (BMS_Get_Input_Pin_Data() == BMS_LOW) // �жϽ������ŵĵ�ƽ״̬���������͵�ƽʱ����ʼ��ʱ
		{
			BMS_receive_bit_num = 0;					  // ����bitλ������
			BMS_receive_data_num = 0;					  // ���ý������ݸ���
			BMS_receive_H_L_Level_time_cnt = 0;			  // �ߵ͵�ƽ��ʱ������0
			BMS_receive_start_H_L_Level_timming_flag = 1; // ��ʼ�ߵ͵�ƽ��ʱ
			BMS_receive_state = BMS_REV_STATE_SYNC_L;	  // �����ȡͬ���͵�ƽ�ź�״̬

			memset(BMS_receive_data_buf, 0, BMS_REV_DATA_NUM);
		}
		break;

	case BMS_REV_STATE_SYNC_L:					  // �ڶ�ȡͬ���͵�ƽ�ź��ڼ�
		if (BMS_Get_Input_Pin_Data() == BMS_HIGH) // ͬ���źŵ͵�ƽ����ڼ�����ߵ�ƽ
		{
			if (BMS_receive_H_L_Level_time_cnt >= BMS_REV_SYNC_L_TIME_NUM) // ���ͬ���źŵ͵�ƽʱ��>=SYNC_L_TIME_NUM
			{															   // ͬ���źŵ͵�ƽʱ��Ҫ>=10ms
				BMS_receive_H_L_Level_time_cnt = 0;						   // �ߵ͵�ƽ��ʱ������0
				BMS_receive_state = BMS_REV_STATE_SYNC_H;				   // �����ȡͬ���źŸߵ�ƽ״̬
			}
			else
			{
				BMS_receive_state = BMS_REV_STATE_IDLE_SIGNAL; // �������½���״̬
			}
		}
		break;

	case BMS_REV_STATE_SYNC_H:					 // �ڶ�ȡͬ���źŸߵ�ƽ�ڼ�
		if (BMS_Get_Input_Pin_Data() == BMS_LOW) // ͬ���źŸߵ�ƽ����ڼ�����͵�ƽ
		{
			// �ж�ͬ���źŸߵ�ƽʱ���Ƿ���1ms��100us֮��
			if (BMS_receive_H_L_Level_time_cnt >= BMS_REV_SYNC_H_TIME_NUM_MIN && BMS_receive_H_L_Level_time_cnt <= BMS_REV_SYNC_H_TIME_NUM_MAX)
			{
				BMS_receive_H_L_Level_time_cnt = 0; // �ߵ͵�ƽ��ʱ������0
				BMS_receive_has_read_bit = 0;
				BMS_receive_state = BMS_REV_STATE_REV_BIT; // �����ȡ����״̬
			}
			else
			{
				BMS_receive_state = BMS_REV_STATE_IDLE_SIGNAL; // �������½���״̬
			}
		}
		else // �����ͬ���źŸߵ�ƽ����ڼ䣬ʱ�䳬��2ms��200us����Ϊ��ʱ
		{
			// �ж�ʱ���Ƿ�ʱ 2ms��200us
			if (BMS_receive_H_L_Level_time_cnt >= BMS_REV_LOGIC_CYCLE_NUM_MAX)
			{
				BMS_receive_state = BMS_REV_STATE_IDLE_SIGNAL; // �������½���״̬
			}
		}
		break;

	case BMS_REV_STATE_REV_BIT:					  // �ڶ�ȡ�������ƽ�ڼ�
		if (BMS_Get_Input_Pin_Data() == BMS_HIGH) // ͬ���źŵ͵�ƽ����ڼ�����ߵ�ƽ
		{
			// �ж�ʱ���Ƿ�ʱ 2ms��200us
			if (BMS_receive_H_L_Level_time_cnt >= BMS_REV_LOGIC_CYCLE_NUM_MAX)
			{
				BMS_receive_state = BMS_REV_STATE_IDLE_SIGNAL; // �������½���״̬
			}
			else
			{
				if (BMS_receive_has_read_bit == 0)
				{
					if ((BMS_receive_H_L_Level_time_cnt < (BMS_REV_HALF_LOGIC_CYCLE_MIN)))
					{
						BMS_receive_data_buf[BMS_receive_data_num] |= 0x01;
					}
					BMS_receive_has_read_bit = 1;
					BMS_receive_state = BMS_REV_STATE_BUILD_DATA;
				}
			}
		}
		else
		{
			if (BMS_receive_H_L_Level_time_cnt >= BMS_REV_LOGIC_CYCLE_NUM_MAX)
			{
				BMS_receive_state = BMS_REV_STATE_IDLE_SIGNAL; // �������½���״̬
			}
		}
		break;

	case BMS_REV_STATE_BUILD_DATA:
		if (BMS_Get_Input_Pin_Data() == BMS_LOW) // ͬ���źŸߵ�ƽ����ڼ�����͵�ƽ
		{
			// �ж�ͬ���źŸߵ�ƽʱ���Ƿ���1ms��100us֮��
			if (BMS_receive_has_read_bit == 1)
			{
				BMS_receive_H_L_Level_time_cnt = 0; // �ߵ͵�ƽ��ʱ������0

				BMS_receive_has_read_bit = 0; // ��0����ȡ��һ��bitλ
				BMS_receive_bit_num++;		  // ���յ�bit��++

				if (BMS_receive_bit_num == BMS_REV_BIT_NUM) // ���һ���ֽ�8��bitλ�������
				{
					BMS_receive_data_num++;						  // ���յ����ݸ���++
					BMS_receive_bit_num = 0;					  // ����bitλ������0���½���
					if (BMS_receive_data_num == BMS_REV_DATA_NUM) // ������ݲɼ����
					{
						BMS_receive_state = BMS_REV_STATE_END_SIGNAL; // ������ս����͵�ƽ�ź�״̬
					}
					else
					{
						BMS_receive_state = BMS_REV_STATE_REV_BIT; // �����ȡ����״̬
					}
				}
				else // ���һ���ֽ�8��bitλ��û�н������
				{
					// ���������ݻ�������һλ�����ݴӵ�bitλ��ʼ����
					BMS_receive_data_buf[BMS_receive_data_num] =
						BMS_receive_data_buf[BMS_receive_data_num] << 1;
					BMS_receive_state = BMS_REV_STATE_REV_BIT; // �����ȡ����״̬
				}
			}
		}
		else // �����ͬ���źŸߵ�ƽ����ڼ䣬ʱ�䳬��2ms��200us����Ϊ��ʱ
		{
			// �ж�ʱ���Ƿ�ʱ 2ms��200us
			if (BMS_receive_H_L_Level_time_cnt >= BMS_REV_LOGIC_CYCLE_NUM_MAX)
			{
				BMS_receive_state = BMS_REV_STATE_IDLE_SIGNAL; // �������½���״̬
			}
		}
		break;

	case BMS_REV_STATE_END_SIGNAL: // �ڽ��ս����źŵ͵�ƽ�ڼ�
		if (BMS_Get_Input_Pin_Data() == BMS_LOW)
		{
			if (BMS_receive_H_L_Level_time_cnt >= BMS_REV_END_SIGNAL_TIME_NUM) // ��������͵�ƽʱ��>=5ms
			{

				memcpy(BMS_receive_buf, BMS_receive_data_buf, BMS_REV_DATA_NUM);
				BMS_Checksum_Handle();

				BMS_receive_read_success = 1;				  // һ֡���ݶ�ȡ�ɹ�
				BMS_receive_start_H_L_Level_timming_flag = 0; // ֹͣ�ߵ͵�ƽ��ʱ
				BMS_receive_H_L_Level_time_cnt = 0;			  // ��ʱ������ֵ��0
				BMS_receive_state = BMS_REV_STATE_INITIAL;	  // ����״̬��0
			}
		}
		else // �����źŵ͵�ƽ����ڼ�һֱΪ��
		{
			if (BMS_receive_H_L_Level_time_cnt >= BMS_REV_LOGIC_CYCLE_NUM_MAX) // ��������͵�ƽʱ��>=10ms����Ϊ��ʱ
			{																   // һ֡���ݷ�����ɺ���Ҫ���50ms�ŷ��͵ڶ�֡���ݣ��ڼ�϶��ᱻ����
				BMS_receive_state = BMS_REV_STATE_IDLE_SIGNAL;				   // �������½���״̬
			}
		}
		break;
#if 1
	case BMS_REV_STATE_IDLE_SIGNAL:
		// BMS_receive_state = BMS_REV_STATE_RESTART;
		// break;
	case BMS_REV_STATE_RESTART:						  // ���½�������״̬
		BMS_receive_start_H_L_Level_timming_flag = 0; // ֹͣ�ߵ͵�ƽ��ʱ
		BMS_receive_H_L_Level_time_cnt = 0;			  // ��ʱ������ֵ��0
		BMS_receive_has_read_bit = 0;				  // ��0����ȡ��һ��bitλ
		BMS_receive_bit_num = 0;
		BMS_receive_data_num = 0;
		BMS_receive_state = BMS_REV_STATE_INITIAL; // ����״̬��0
		break;
#endif
	}
}

#ifdef BMS_SEND_HANDLER
uint8_t BMS_SendData_Sync(uint8_t *data, uint8_t len)
{
	MY_ASSERT(data);
	MY_ASSERT(len);

	if (!BMS_has_inited)
	{
		return 0;
	}

	uint8_t byte = 0;
	uint8_t flag = 0;
	// ͬ���ź�
	BMS_SEND_DATA_BIT_LOW();
	BMS_Delay_us(20000);
	BMS_SEND_DATA_BIT_HIGH();
	BMS_Delay_us(500);
	for (int i = 0; i < len; i++)
	{
		byte = data[i];
		for (int j = 0; j < 8; j++)
		{
			flag = byte & (0x80 >> j);
			if (flag)
			{
				BMS_SEND_DATA_BIT_LOW();
				BMS_Delay_us(500);
				BMS_SEND_DATA_BIT_HIGH();
				BMS_Delay_us(1000);
			}
			else
			{
				BMS_SEND_DATA_BIT_LOW();
				BMS_Delay_us(1000);
				BMS_SEND_DATA_BIT_HIGH();
				BMS_Delay_us(500);
			}
		}
	}
	BMS_SEND_DATA_BIT_LOW();
	BMS_Delay_us(5000);
	return len;
}

uint8_t BMS_SendData(uint8_t *data, uint8_t len)
{
#if 1
	LOG_LEVEL("BMS_SendData:");
	for (int i = 0; i < len; i++)
	{
		LOG_LEVEL("%02x ", data[i]);
	}
	LOG_LEVEL("\r\n");
#endif
	MY_ASSERT(data);
	MY_ASSERT(len);
	MY_ASSERT(len <= BMS_SEND_DATA_MAX_NUM);
	if (!BMS_has_inited)
	{
		return 0;
	}

	if (BMS_send_req_flag == false)
	{
		for (int i = 0; i < len; i++)
		{
			BMS_send_data_buf[i] = data[i];
		}
		BMS_send_data_num_target = len;
		BMS_send_req_flag = true;
		return len;
	}
	return 0;
}
///__attribute__((section(".highcode")))
void BMS_Send_Data_Handle(void)
{
	switch (BMS_send_state) // ��⵱ǰ��������״̬
	{
	case BMS_SEND_STATE_INITIAL:	   // ��ʼ״̬��δ���յ�ͬ����Ϣ������ͬ���ж�
		if (BMS_send_req_flag == true) // �жϽ������ŵĵ�ƽ״̬���������͵�ƽʱ����ʼ��ʱ
		{
			BMS_send_bit_num = 0;					   // ����bitλ������
			BMS_send_data_num = 0;					   // ���ý������ݸ���
			BMS_send_H_L_Level_time_cnt = 0;		   // �ߵ͵�ƽ��ʱ������0
			BMS_send_start_H_L_Level_timming_flag = 1; // ��ʼ�ߵ͵�ƽ��ʱ
			BMS_send_state = BMS_SEND_STATE_SYNC_L;	   // �����ȡͬ���͵�ƽ�ź�״̬
			BMS_SEND_DATA_BIT_LOW();
		}
		break;

	case BMS_SEND_STATE_SYNC_L:										 // �ڶ�ȡͬ���͵�ƽ�ź��ڼ�
		if (BMS_send_H_L_Level_time_cnt >= BMS_SEND_SYNC_L_TIME_NUM) // ���ͬ���źŵ͵�ƽʱ��>=SYNC_L_TIME_NUM
		{															 // ͬ���źŵ͵�ƽʱ��Ҫ>=10ms
			BMS_send_H_L_Level_time_cnt = 0;						 // �ߵ͵�ƽ��ʱ������0
			BMS_SEND_DATA_BIT_HIGH();
			BMS_send_state = BMS_SEND_STATE_SYNC_H; // �����ȡͬ���źŸߵ�ƽ״̬
		}
		break;

	case BMS_SEND_STATE_SYNC_H: // �ڶ�ȡͬ���źŸߵ�ƽ�ڼ�
		// �ж�ͬ���źŸߵ�ƽʱ���Ƿ���1ms��100us֮��
		if (BMS_send_H_L_Level_time_cnt >= BMS_SEND_SYNC_H_TIME_NUM)
		{
			BMS_send_H_L_Level_time_cnt = 0; // �ߵ͵�ƽ��ʱ������0
			BMS_SEND_DATA_BIT_LOW();
			BMS_send_state = BMS_SEND_STATE_GET_BIT; // �����ȡ����״̬
		}
		break;

	case BMS_SEND_STATE_GET_BIT: // �ڶ�ȡ�������ƽ�ڼ�
		BMS_send_H_L_Level_time_cnt = 0;
		BMS_send_bit = BMS_send_data_buf[BMS_send_data_num] & (0x80 >> BMS_send_bit_num);
		BMS_send_state = BMS_SEND_STATE_BIT_L; // �����ȡ����״̬
		break;

	case BMS_SEND_STATE_BIT_L:
		if (BMS_send_bit)
		{
			if (BMS_send_H_L_Level_time_cnt >= BMS_SEND_SHORT_TIME_NUM)
			{
				BMS_send_H_L_Level_time_cnt = 0;
				BMS_SEND_DATA_BIT_HIGH();
				BMS_send_state = BMS_SEND_STATE_BIT_H; // �������½���״̬
			}
		}
		else
		{
			if (BMS_send_H_L_Level_time_cnt >= BMS_SEND_LONG_TIME_NUM)
			{
				BMS_send_H_L_Level_time_cnt = 0;
				BMS_SEND_DATA_BIT_HIGH();
				BMS_send_state = BMS_SEND_STATE_BIT_H; // �������½���״̬
			}
		}
		break;

	case BMS_SEND_STATE_BIT_H:
		if (BMS_send_bit)
		{
			if (BMS_send_H_L_Level_time_cnt >= BMS_SEND_LONG_TIME_NUM)
			{
				BMS_send_H_L_Level_time_cnt = 0;
				BMS_SEND_DATA_BIT_LOW();
				BMS_send_state = BMS_SEND_STATE_CHECK_BIT; // �������½���״̬
			}
		}
		else
		{
			if (BMS_send_H_L_Level_time_cnt >= BMS_SEND_SHORT_TIME_NUM)
			{
				BMS_send_H_L_Level_time_cnt = 0;
				BMS_SEND_DATA_BIT_LOW();
				BMS_send_state = BMS_SEND_STATE_CHECK_BIT; // �������½���״̬
			}
		}
		break;

	case BMS_SEND_STATE_CHECK_BIT:
		BMS_send_bit_num++;
		if (BMS_send_bit_num == BMS_SEND_BIT_NUM) // ���һ���ֽ�8��bitλ�������
		{
			BMS_send_data_num++;							   // ���յ����ݸ���++
			BMS_send_bit_num = 0;							   // ����bitλ������0���½���
			if (BMS_send_data_num == BMS_send_data_num_target) // ������ݲɼ����
			{
				BMS_send_state = BMS_SEND_STATE_END_SIGNAL; // ������ս����͵�ƽ�ź�״̬
			}
			else
			{
				BMS_SEND_DATA_BIT_LOW();
				BMS_send_state = BMS_SEND_STATE_GET_BIT; // �����ȡ����״̬
			}
		}
		else // ���һ���ֽ�8��bitλ��û�н������
		{
			BMS_SEND_DATA_BIT_LOW();
			BMS_send_state = BMS_SEND_STATE_GET_BIT; // �����ȡ����״̬
		}
		break;

	case BMS_SEND_STATE_END_SIGNAL: // �ڽ��ս����źŵ͵�ƽ�ڼ�
		BMS_send_req_flag = false;
		BMS_send_start_H_L_Level_timming_flag = 0; // ֹͣ�ߵ͵�ƽ��ʱ
		BMS_send_H_L_Level_time_cnt = 0;		   // ��ʱ������ֵ��0
		BMS_send_bit_num = 0;
		BMS_send_data_num = 0;
		BMS_send_state = BMS_SEND_STATE_INITIAL; // ����״̬��0
		break;
	}
}
#endif
/*******************************************************************************
 *�������� : Check_Sum_Handle
 *�������� : У��ʹ���
 *������� : void
 *������� : void
 *******************************************************************************/
///__attribute__((section(".highcode")))
void BMS_Checksum_Handle(void)
{
	uint8_t checkByte = 0;
	uint64_t checkXor = 0;
	for (int i = 0; i < (BMS_REV_DATA_NUM - 1); i++)
	{
		checkXor = checkXor ^ BMS_receive_buf[i];
	}

	checkByte = (unsigned char)checkXor;
	if (checkByte == BMS_receive_buf[BMS_REV_DATA_NUM - 1]) // У�����ȷ
	{
		BMS_receive_check_OK = 1; // ���У��ɹ�
	}
	else
	{
		BMS_receive_check_OK = 0; // ���У��ʧ��
	}
}

///__attribute__((section(".highcode")))
uint8_t BMS_Get_Input_Pin_Data(void)
{
	static uint8_t flag = 0;
	static uint8_t value = 0;
	static uint8_t ret = 0;
	uint8_t cur = BMS_RECEIVE_DATA_BIT();
	if (value != cur)
	{
		flag = 3;
		value = cur;
	}
	if (flag)
	{
		flag--;
		if (flag == 0)
		{
			ret = value;
		}
	}
	return ret;
}

#endif
