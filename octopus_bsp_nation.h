
/**
 * @file	bsp.h
 * @author	chipsea
 * @brief
 * @version	0.1
 * @date	2020-11-30
 * @copyright Copyright (c) 2020, CHIPSEA Co., Ltd.
 * @note
 */

/*******************************************************************************
 * @file		bsp.h
 * @brief	Contains all functions support for uart driver
 * @version	0.0
 *
 *******************************************************************************/
#ifndef __OCTOPUS_TASK_MANAGER_NATION_BSP_H__
#define __OCTOPUS_TASK_MANAGER_NATION_BSP_H__

#include <stddef.h>	 // Standard definitions for NULL and size_t
#include <stdint.h>	 // Standard integer type definitions
#include <stdbool.h> // Boolean type definitions
#include <stdio.h>	 // Standard input/output functions

#include "n32l40x.h"

#define HOST_COMM_UART USART1
#define HOST_COMM_UART_IRQ USART1_IRQn
#define GPIO_HOST_UART_TX_GRP GPIOA
#define GPIO_HOST_UART_TX_PIN GPIO_PIN_9
#define GPIO_HOST_UART_RX_GRP GPIOA
#define GPIO_HOST_UART_RX_PIN GPIO_PIN_10

#define CAN_COMM_UART USART2
#define CAN_COMM_UART_IRQ USART2_IRQn
#define GPIO_CAN_UART_TX_GRP GPIOA
#define GPIO_CAN_UART_TX_PIN GPIO_PIN_2
#define GPIO_CAN_UART_RX_GRP GPIOA
#define GPIO_CAN_UART_RX_PIN GPIO_PIN_3

#define GPIO_REAL_SYS_PWR_GRP GPIOC
#define GPIO_REAL_SYS_PWR_PIN GPIO_PIN_4
#define REAL_SYS_PWR_ON                                             \
	{                                                               \
		GPIO_SetBits(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN); \
	}
#define REAL_SYS_PWR_OFF                                              \
	{                                                                 \
		GPIO_ResetBits(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN); \
	}
#define REAL_SYS_PWR_IS_ON() (Bit_SET == GPIO_ReadOutputDataBit(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN))

#define GPIO_DEVICE_5V_PWR_GRP GPIOB
#define GPIO_DEVICE_5V_PWR_PIN GPIO_PIN_14
#define SYSTEM_POWER_CTRL_ON                                          \
	{                                                                 \
		GPIO_SetBits(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN); \
	}
#define SYSTEM_POWER_CTRL_OFF                                           \
	{                                                                   \
		GPIO_ResetBits(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN); \
	}
#define SYSTEM_POWER_IS_ON() (Bit_SET == GPIO_ReadOutputDataBit(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN))

#define GPIO_AUDIO_PWR_GRP GPIOA
#define GPIO_AUDIO_PWR_PIN GPIO_PIN_6

#define GPIO_FCAM_PWR_GRP GPIOB
#define GPIO_FCAM_PWR_PIN GPIO_PIN_5

#define GPIO_LCD_EN_GRP GPIOC
#define GPIO_LCD_EN_PIN GPIO_PIN_13
#define IS_PANEL_PWR_ON (Bit_SET == GPIO_ReadOutputDataBit(GPIO_LCD_EN_GRP, GPIO_LCD_EN_PIN))

#define GPIO_BKL_EN_GRP GPIOA
#define GPIO_BKL_EN_PIN GPIO_PIN_5

#define GPIO_NISSAN_XTRAIL_CAM_SW_GRP GPIOB
#define GPIO_NISSAN_XTRAIL_CAM_SW_PIN GPIO_PIN_13

#define GPIO_ANT_CTRL_GRP GPIOA
#define GPIO_ANT_CTRL_PIN GPIO_PIN_12

#define GPIO_ACC_GRP GPIOA
#define GPIO_ACC_PIN GPIO_PIN_0

#define GPIO_BRAKE_GRP GPIOC
#define GPIO_BRAKE_PIN GPIO_PIN_15

#define GPIO_ILL_GRP GPIOB
#define GPIO_ILL_PIN GPIO_PIN_2

#define GPIO_REVERSE_GRP GPIOB
#define GPIO_REVERSE_PIN GPIO_PIN_12

#define GPIO_TEL_MUTE_DET_GRP GPIOC
#define GPIO_TEL_MUTE_DET_PIN GPIO_PIN_14

#define TIMER_IR_RX TIM5
#define TIMER_CH_IR_RX TIM_CH_2
#define GPIO_IR_RX_GRP GPIOA
#define GPIO_IR_RX_PIN GPIO_PIN_1

#define TIMER_CH_CANIR_RX TIM_CH_4

#define GPIO_MUTE_GRP GPIOB
#define GPIO_MUTE_PIN GPIO_PIN_9
#define AUDIO_HW_MUTE GPIO_SetBits(GPIO_MUTE_GRP, GPIO_MUTE_PIN);
#define AUDIO_HW_UNMUTE GPIO_ResetBits(GPIO_MUTE_GRP, GPIO_MUTE_PIN);

#define GPIO_I2C_GRP GPIOB
#define GPIO_I2C_SCL_PIN GPIO_PIN_10
#define GPIO_I2C_SDA_PIN GPIO_PIN_11

#define GPIO_DVD_RST_GRP GPIOB
#define GPIO_DVD_RST_PIN GPIO_PIN_3

#define GPIO_DVD_PWR_CTRL_GRP GPIOB
#define GPIO_DVD_PWR_CTRL_PIN GPIO_PIN_4

#define GPIO_DVD_DISC_DET_GRP GPIOD
#define GPIO_DVD_DISC_DET_PIN GPIO_PIN_2

#define GPIO_DVD_LOAD_P_GRP GPIOC
#define GPIO_DVD_LOAD_P_PIN GPIO_PIN_10
#define GPIO_DVD_LOAD_N_GRP GPIOA
#define GPIO_DVD_LOAD_N_PIN GPIO_PIN_15

#define GPIO_DVD_TRAY_OUT_GRP GPIOC
#define GPIO_DVD_TRAY_OUT_PIN GPIO_PIN_11
#define GPIO_DVD_TRAY_IN_GRP GPIOC
#define GPIO_DVD_TRAY_IN_PIN GPIO_PIN_12

#define GPIO_HOST_REC_KEY_GRP GPIOA
#define GPIO_HOST_REC_KEY_PIN GPIO_PIN_11

#define GPIO_HOST_PWR_KEY_GRP GPIOB
#define GPIO_HOST_PWR_KEY_PIN GPIO_PIN_15

#define GPIO_DTV_IR_TX_GRP GPIOB
#define GPIO_DTV_IR_TX_PIN GPIO_PIN_6

#define GPIO_SWC_KEY_CTL_2K2_GRP GPIOB
#define GPIO_SWC_KEY_CTL_2K2_PIN GPIO_PIN_0
#define GPIO_SWC_KEY_CTL_470R_GRP GPIOB
#define GPIO_SWC_KEY_CTL_470R_PIN GPIO_PIN_1
#define CAR_SWC_KEY_PULLUP_LARGE                                            \
	do                                                                      \
	{                                                                       \
		GPIO_SetBits(GPIO_SWC_KEY_CTL_2K2_GRP, GPIO_SWC_KEY_CTL_2K2_PIN);   \
		GPIO_SetBits(GPIO_SWC_KEY_CTL_470R_GRP, GPIO_SWC_KEY_CTL_470R_PIN); \
	} while (0);
#define CAR_SWC_KEY_PULLUP_SMALL                                            \
	do                                                                      \
	{                                                                       \
		GPIO_ResetBits(GPIO_SWC_KEY_CTL_2K2_GRP, GPIO_SWC_KEY_CTL_2K2_PIN); \
		GPIO_SetBits(GPIO_SWC_KEY_CTL_470R_GRP, GPIO_SWC_KEY_CTL_470R_PIN); \
	} while (0);
#define CAR_SWC_KEY_PULLUP_TINY                                               \
	do                                                                        \
	{                                                                         \
		GPIO_SetBits(GPIO_SWC_KEY_CTL_2K2_GRP, GPIO_SWC_KEY_CTL_2K2_PIN);     \
		GPIO_ResetBits(GPIO_SWC_KEY_CTL_470R_GRP, GPIO_SWC_KEY_CTL_470R_PIN); \
	} while (0);

#define AD_PANEL_KEY_DET_GRP ADC
#define AD_PANEL_KEY_DET_1 ADC_CH_13
#define AD_PANEL_KEY_DET_2 ADC_CH_14
#define GPIO_AD_PANEL_KEY_DET_GRP GPIOC
#define GPIO_AD_PANEL_KEY_DET_1_PIN GPIO_PIN_2
#define GPIO_AD_PANEL_KEY_DET_2_PIN GPIO_PIN_3

#define AD_CAR_SWC_DET_GRP ADC
#define AD_CAR_SWC_DET_1 ADC_CH_11
#define AD_CAR_SWC_DET_2 ADC_CH_12
#define GPIO_AD_SWC_KEY_DET_GRP GPIOC
#define GPIO_AD_SWC_KEY_DET_1_PIN GPIO_PIN_0
#define GPIO_AD_SWC_KEY_DET_2_PIN GPIO_PIN_1

#define AD_VOL_ENCODER_GRP ADC
#define AD_VOL_ENCODER ADC_CH_16
#define GPIO_AD_VOL_GRP GPIOC
#define GPIO_AD_VOL_PIN GPIO_PIN_5

#define AD_TUNE_ENCODER_GRP ADC
#define AD_TUNE_ENCODER ADC_CH_5
#define GPIO_AD_TUNE_GRP GPIOA
#define GPIO_AD_TUNE_PIN GPIO_PIN_4

#define AD_BATT_DET_GRP ADC
#define AD_BATT_DET ADC_CH_8
#define GPIO_AD_BATT_DET_GRP GPIOA
#define GPIO_AD_BATT_DET_PIN GPIO_PIN_7

#define TIMER_LED TIM8
#define TIMER_CH_LED_B TIM_CH_1
#define TIMER_CH_LED_G TIM_CH_2
#define TIMER_CH_LED_R TIM_CH_3
#define GPIO_LED_GRP GPIOC
#define GPIO_LED_B_PIN GPIO_PIN_6
#define GPIO_LED_G_PIN GPIO_PIN_7
#define GPIO_LED_R_PIN GPIO_PIN_8
#define GPIO_LED_EN_PIN GPIO_PIN_9

#define TIMER_VCOM TIM1
#define TIMER_CH_VCOM TIM_CH_1
#define GPIO_VCOM_GRP GPIOA
#define GPIO_VCOM_PIN GPIO_PIN_8

#define TIMER_BEEP TIM4
#define TIMER_CH_BEEP TIM_CH_3
#define GPIO_BEEP_GRP GPIOB
#define GPIO_BEEP_PIN GPIO_PIN_8

#ifdef __cplusplus
extern "C"
{
#endif

	void SYS_Config(void);
	void GPIO_Config(void);
	void RCC_Config(void);

	void UART1_Config_IRQ(void);
	void UART2_Config_IRQ(void);
	void UART3_Config_IRQ(void);
	void UART4_Config_IRQ(void);
	void UART5_Config_IRQ(void);

	void LUART_Config(void);

	void UART1_Send_Buffer(const uint8_t *buffer, uint16_t length);
	void UART2_Send_Buffer(const uint8_t *buffer, uint16_t length);
	void UART3_Send_Buffer(const uint8_t *buffer, uint16_t length);
	void UART4_Send_Buffer(const uint8_t *buffer, uint16_t length);
	void LUART_Send_Buffer(const uint8_t *buffer, size_t length);

	void PTL_1_UART_Send_Buffer(const uint8_t *buffer, uint16_t length);
	void PTL_2_UART_Send_Buffer(const uint8_t *buffer, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
