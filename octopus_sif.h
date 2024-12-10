/********************************** (C) COPYRIGHT *******************************
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/******************************************************************************/
#ifndef __OCTOPUS_TASK_MANAGER_SIF_H__
#define __OCTOPUS_TASK_MANAGER_SIF_H__

#include "octopus_platform.h"
#include "octopus_gpio_hal.h"
#include "octopus_gpio.h"


#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************
 * CONSTANTS
 */
 
 /********************************************************************
 * MACROS
 */
//#define SIF_REV_DATA_APB2               (RCC_APB2Periph_GPIOB)
//#define SIF_REV_DATA_PORT               (GPIOB)     //定义数据接收引脚(根据实际项目进行更改)
//#define SIF_REV_DATA_PIN                (GPIO_Pin_11)
#define SIF_RECEIVE_DATA_BIT()             GPIO_PIN_READ_SIF()

//#define SIF_SEND_DATA_APB2              (RCC_APB2Periph_GPIOB)
//#define SIF_SEND_DATA_PORT              (GPIOB)     //定义数据接收引脚(根据实际项目进行更改)
//#define SIF_SEND_DATA_PIN               (GPIO_Pin_3)
#define SIF_SEND_DATA_BIT_LOW()         GPIO_PIN_SIF_SET_LOW() //(GPIO_WriteBit(SIF_SEND_DATA_PORT, SIF_SEND_DATA_PIN, SIF_HIGH))
#define SIF_SEND_DATA_BIT_HIGH()        GPIO_PIN_SIF_SET_HIGH() //(GPIO_WriteBit(SIF_SEND_DATA_PORT, SIF_SEND_DATA_PIN, SIF_LOW))

/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */
 
/*******************************************************************************
 * extrn FUNCTIONS DECLEAR
 */
extern void SIF_IO_IRQHandler(void);
extern uint8_t IsSIFIdle(void);
/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
extern void Sif_Init(void);
extern void Sif_DeInit(void);
extern bool Sif_IsInit(void);
//void Sif_Receive_Data_Handle(void);      									//接收数据处理—带校准位，即波特率自适应
//void Sif_Send_Data_Handle(void);
//void RestartSIF(void);
extern uint8_t Sif_ReadData(uint8_t* data, uint8_t maxlen);

/*******************************************************************************
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
