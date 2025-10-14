
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
#ifndef __OCTOPUS_TASK_MANAGER_BSP_H__
#define __OCTOPUS_TASK_MANAGER_BSP_H__

#include <stddef.h> // Standard definitions for NULL and size_t
#include <stdint.h> // Standard integer type definitions

#include "octopus_can.h"
#include "hk32l0xx.h"
#include "hk32l0xx_eeprom.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void SYS_Config(void);
    void GPIO_Config(void);
    void RCC_Config(void);
    void CAN_Config(void);
    void TIM2_Config(void);
    void ADC_DMA_Config(void);
    void IWDG_Init(uint8_t prer, uint16_t reload);
    void IWDG_Feed(void);
    void ADC_Channel_8_Config(void);
			
    void UART1_Config_IRQ(void);
    void UART2_Config_DMA(void);
    void UART2_Config_IRQ(void);

    void UART3_Config_IRQ(void);
    void UART3_Config_DMA(void);
    void UART4_Config_IRQ(void);
    void LPUART_Config(void);
    void LPUART_WakeStop_Config(void);

    void UART1_SendStr(const char *str);
    void UART1_Send_Buffer(const uint8_t *buffer, uint16_t length);

    void UART2_Send_Buffer(const uint8_t *buffer, uint16_t length);
    void UART2_Send_Buffer_DMA(const uint8_t *buffer, uint16_t length);

    void UART3_Send_Buffer(const uint8_t *buffer, uint16_t length);
    void UART3_Send_Buffer_DMA(const uint8_t *buffer, uint16_t length);

    void UART4_Send_Buffer(const uint8_t *buffer, uint16_t length);
    void LPUART_Send_Buffer(const uint8_t *buffer, size_t length);

    void TIM3_PWM_Backlight_Init(uint16_t pwm_freq, uint8_t duty_cycle);

    // void EEPROM_write_buffer(uint32_t addr, uint8_t *buffer, uint8_t length);
    // void EEPROM_read_buffer(uint32_t addr, uint8_t *buffer, uint8_t length);

    void native_enter_sleep_mode(void);

    void PTL_1_UART_Send_Buffer(const uint8_t *buffer, uint16_t length);

    CAN_Status_t CAN_Send_Data(uint32_t id, uint8_t ide, const uint8_t *buffer, uint8_t length);

    uint8_t hal_iic1_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic2_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic3_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic4_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic5_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic6_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);

    uint8_t hal_iic1_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic2_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic3_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic4_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic5_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);
    uint8_t hal_iic6_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length);

    void platform_delay_us(uint32_t us);
    void platform_delay_ms(uint32_t us);
		
	uint16_t adc_get_value_v(void);

#ifdef __cplusplus
}
#endif

#endif
