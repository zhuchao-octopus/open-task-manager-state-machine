/**
 * ****************************************************************************
 * @copyright Copyright (c) XXX
 * All rights reserved.
 *
 * C file for the Octopus Task Manager module.
 * Defines macros, includes required libraries, and implements functions.
 */

/*********************************************************************
 * INCLUDES
 */
#include "octopus_gpio_hal.h" // Include the GPIO hardware abstraction layer header

#define GPIO_MODE_INPUT  1

bool hal_gpio_read(GPIO_GROUP *gpiox, uint16_t pin)
{
// Macros for writing to and reading from GPIO pins
#ifdef PLATFORM_STM32_RTOS
    return (GPIO_ReadInputDataBit(gpiox, pin)); // Read the state of the specified GPIO pin
#elif defined(PLATFORM_CST_OSAL_RTOS)
    return HalGpioGet((GpioPin_t)pin);
#elif defined(PLATFORM_ITE_OPEN_RTOS)
    return 0;
#else
    return 0;
#endif
}

bool hal_gpio_write(GPIO_GROUP *gpiox, uint16_t pin, uint8_t value)
{
    // Macros for writing to and reading from GPIO pins
#ifdef PLATFORM_STM32_RTOS
    GPIO_WriteBit(gpiox, pin, (BitAction)value); // Write to the specified GPIO pin
#elif defined(PLATFORM_CST_OSAL_RTOS)
    HalGpioSet((GpioPin_t)pin, (bit_action_e)value);
#elif defined(PLATFORM_ITE_OPEN_RTOS)
#else
#endif
    return value;
}

void hal_gpio_set_mode(GPIO_GROUP *gpiox, uint16_t pin, uint8_t io_mode)
{
	
}

void hal_gpio_power_on(void)
{
#ifdef PLATFORM_STM32_RTOS
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        // Alternate function mode
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;      // Open-drain output//GPIO_OType_PP;// Push-pull output
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    // GPIO_PuPd_UP;// Pull-up resistor enabled
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3; // High speed

    // PA15 - 3.3V power enable for MCU (MCU_3V3_EN)
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    // PB4 - SWB+ power enable (SWB+_EN)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    hal_gpio_write(GPIO_POWER_ENABLE_GROUP, GPIO_POWER_ENABLE_PIN, BIT_SET);
    hal_gpio_write(GPIO_POWER_SWITCH_GROUP, GPIO_POWER_SWITCH_PIN, BIT_SET);
#endif
}

void hal_gpio_init(uint8_t task_id)
{
    LOG_LEVEL("hal gpio init\r\n"); // Optional log for GPIO initialization (disabled here)
}


