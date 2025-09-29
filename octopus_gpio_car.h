/******************************************************************************
 * @file    octopus_gpio_car.h
 * @brief   Hardware IO definitions for MCU board
 *          Defines GPIOs, UARTs, ADC channels, timers, and power controls.
 * @note    Categorized by function: UART, Power, Panel, ADC, Timers, LEDs, IR, Audio, DVD, Car SWC, etc.
 * @author
 * @date
 ******************************************************************************/

#ifndef __OCTOPUS_GIO_CAR_H__
#define __OCTOPUS_GIO_CAR_H__

/*==============================================================================
 * UART Definitions
 *============================================================================*/
#define SOC_HOST_UART USART1
#define SOC_HOST_UART_IRQ USART1_IRQn

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

/*==============================================================================
 * Power Control
 *============================================================================*/
#define GPIO_REAL_SYS_PWR_GRP GPIOC
#define GPIO_REAL_SYS_PWR_PIN GPIO_PIN_4
#define REAL_SYS_PWR_ON()                                           \
    {                                                               \
        GPIO_SetBits(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN); \
    }
#define REAL_SYS_PWR_OFF()                                            \
    {                                                                 \
        GPIO_ResetBits(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN); \
    }
#define REAL_SYS_PWR_IS_ON() (Bit_SET == GPIO_ReadOutputDataBit(GPIO_REAL_SYS_PWR_GRP, GPIO_REAL_SYS_PWR_PIN))

#define GPIO_DEVICE_5V_PWR_GRP GPIOB
#define GPIO_DEVICE_5V_PWR_PIN GPIO_PIN_14
#define SYSTEM_POWER_CTRL_ON()                                        \
    {                                                                 \
        GPIO_SetBits(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN); \
    }
#define SYSTEM_POWER_CTRL_OFF()                                         \
    {                                                                   \
        GPIO_ResetBits(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN); \
    }
#define SYSTEM_POWER_IS_ON() (Bit_SET == GPIO_ReadOutputDataBit(GPIO_DEVICE_5V_PWR_GRP, GPIO_DEVICE_5V_PWR_PIN))

#define GPIO_AUDIO_PWR_GRP GPIOA
#define GPIO_AUDIO_PWR_PIN GPIO_PIN_6

#define GPIO_FCAM_PWR_GRP GPIOB
#define GPIO_FCAM_PWR_PIN GPIO_PIN_5

/*==============================================================================
 * Display / Panel
 *============================================================================*/
#define GPIO_LCD_EN_GRP GPIOC
#define GPIO_LCD_EN_PIN GPIO_PIN_13
#define IS_PANEL_PWR_ON() (Bit_SET == GPIO_ReadOutputDataBit(GPIO_LCD_EN_GRP, GPIO_LCD_EN_PIN))

#define GPIO_BKL_EN_GRP GPIOC
#define GPIO_BKL_EN_PIN GPIO_PIN_13

#define GPIO_8844_EN_GRP GPIOA
#define GPIO_8844_EN_PIN GPIO_PIN_5

#define IS_PANEL_ROOLER_KEY1_ON (Bit_SET == GPIO_ReadOutputDataBit(GPIOA, GPIO_PIN_15))
#define IS_PANEL_ROOLER_KEY2_ON (Bit_SET == GPIO_ReadOutputDataBit(GPIOB, GPIO_PIN_3))

#define GPIO_ANT_CTRL_GRP GPIOA
#define GPIO_ANT_CTRL_PIN GPIO_PIN_12

/*==============================================================================
 * Car Signals / Inputs
 *============================================================================*/
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

/*==============================================================================
 * IR / Audio Control
 *============================================================================*/
#define TIMER_IR_RX TIM5
#define TIMER_CH_IR_RX TIM_CH_2
#define GPIO_IR_RX_GRP GPIOA
#define GPIO_IR_RX_PIN GPIO_PIN_1

#define TIMER_CH_CANIR_RX TIM_CH_4

#define GPIO_MUTE_GRP GPIOB
#define GPIO_MUTE_PIN GPIO_PIN_9
#define AUDIO_HW_MUTE() GPIO_SetBits(GPIO_MUTE_GRP, GPIO_MUTE_PIN)
#define AUDIO_HW_UNMUTE() GPIO_ResetBits(GPIO_MUTE_GRP, GPIO_MUTE_PIN)

/*==============================================================================
 * I2C Bus
 *============================================================================*/
#define GPIO_I2C_GRP GPIOB
#define GPIO_I2C_SCL_PIN GPIO_PIN_10
#define GPIO_I2C_SDA_PIN GPIO_PIN_11

/*==============================================================================
 * DVD / Media
 *============================================================================*/
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

/*==============================================================================
 * Host Keys
 *============================================================================*/
#define GPIO_HOST_REC_KEY_GRP GPIOA
#define GPIO_HOST_REC_KEY_PIN GPIO_PIN_11

#define GPIO_HOST_PWR_KEY_GRP GPIOB
#define GPIO_HOST_PWR_KEY_PIN GPIO_PIN_15

/*==============================================================================
 * DTV / IR TX
 *============================================================================*/
#define GPIO_DTV_IR_TX_GRP GPIOB
#define GPIO_DTV_IR_TX_PIN GPIO_PIN_6

/*==============================================================================
 * Steering Wheel Control (SWC)
 *============================================================================*/
#define GPIO_SWC_KEY_CTL_2K2_GRP GPIOB
#define GPIO_SWC_KEY_CTL_2K2_PIN GPIO_PIN_0
#define GPIO_SWC_KEY_CTL_470R_GRP GPIOB
#define GPIO_SWC_KEY_CTL_470R_PIN GPIO_PIN_1

#define CAR_SWC_KEY_PULLUP_LARGE()                                          \
    do                                                                      \
    {                                                                       \
        GPIO_SetBits(GPIO_SWC_KEY_CTL_2K2_GRP, GPIO_SWC_KEY_CTL_2K2_PIN);   \
        GPIO_SetBits(GPIO_SWC_KEY_CTL_470R_GRP, GPIO_SWC_KEY_CTL_470R_PIN); \
    } while (0)

#define CAR_SWC_KEY_PULLUP_SMALL()                                          \
    do                                                                      \
    {                                                                       \
        GPIO_ResetBits(GPIO_SWC_KEY_CTL_2K2_GRP, GPIO_SWC_KEY_CTL_2K2_PIN); \
        GPIO_SetBits(GPIO_SWC_KEY_CTL_470R_GRP, GPIO_SWC_KEY_CTL_470R_PIN); \
    } while (0)

#define CAR_SWC_KEY_PULLUP_TINY()                                             \
    do                                                                        \
    {                                                                         \
        GPIO_SetBits(GPIO_SWC_KEY_CTL_2K2_GRP, GPIO_SWC_KEY_CTL_2K2_PIN);     \
        GPIO_ResetBits(GPIO_SWC_KEY_CTL_470R_GRP, GPIO_SWC_KEY_CTL_470R_PIN); \
    } while (0)

/*==============================================================================
 * ADC Channels / Panel Keys / Encoders
 *============================================================================*/
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

/*==============================================================================
 * Timers / LED / Beep / VCOM
 *============================================================================*/
#define TIMER_LED TIM8
#define TIMER_CH_LED_B TIM_CH_1
#define TIMER_CH_LED_G TIM_CH_2
#define TIMER_CH_LED_R TIM_CH_3
#define GPIO_LED_GRP GPIOC
#define GPIO_LED_EN_PIN GPIO_PIN_9

#define TIMER_VCOM TIM1
#define TIMER_CH_VCOM TIM_CH_1
#define GPIO_VCOM_GRP GPIOA
#define GPIO_VCOM_PIN GPIO_PIN_8

#define TIMER_BEEP TIM4
#define TIMER_CH_BEEP TIM_CH_3
#define GPIO_BEEP_GRP GPIOB
#define GPIO_BEEP_PIN GPIO_PIN_8

/*==============================================================================
 * TV / HDMI
 *============================================================================*/
#define GPIO_TV_PWR_GRP GPIOC
#define GPIO_TV_PWR_PIN GPIO_PIN_7
#define GPIO_HDMI_PWR_GRP GPIOC
#define GPIO_HDMI_PWR_PIN GPIO_PIN_8

/*==============================================================================
 * Radio / DVD Reset
 *============================================================================*/
#define GPIO_RADIO_RST_GRP GPIOB
#define GPIO_RADIO_RST_PIN GPIO_PIN_7

#endif // _IO_H_
