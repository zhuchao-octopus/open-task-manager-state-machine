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
#define BMS_RECEIVE_DATA_BIT()             GPIO_PIN_READ_SIF()
#define BMS_SEND_DATA_BIT_LOW()         GPIO_PIN_SIF_SET_LOW() //(GPIO_WriteBit(SIF_SEND_DATA_PORT, SIF_SEND_DATA_PIN, SIF_HIGH))
#define BMS_SEND_DATA_BIT_HIGH()        GPIO_PIN_SIF_SET_HIGH() //(GPIO_WriteBit(SIF_SEND_DATA_PORT, SIF_SEND_DATA_PIN, SIF_LOW))

/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */
 
/*******************************************************************************
 * extrn FUNCTIONS DECLEAR
 */
extern void BMS_IO_IRQHandler(void);
extern uint8_t IsBMSIdle(void);
/*******************************************************************************
 * LOCAL FUNCTIONS DECLEAR
 */
extern void BMS_Init(void);
extern void BMS_DeInit(void);
extern bool BMS_IsInit(void);
//void Sif_Receive_Data_Handle(void);      									//接收数据处理—带校准位，即波特率自适应
//void Sif_Send_Data_Handle(void);
//void RestartSIF(void);
extern uint8_t BMS_ReadData(uint8_t* data, uint8_t maxlen);

/*******************************************************************************
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
