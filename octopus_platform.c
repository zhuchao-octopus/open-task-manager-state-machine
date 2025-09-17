/*******************************************************************************
 * File Name: octopus_platform.c
 * @version  1.0.0
 * @date     2024-12-11
 * @author   Octopus Team
 * Description:
 *   This file provides platform-specific configurations and utility macros
 *   for the Octopus Task Manager project. It supports multiple platforms
 *   and RTOS environments, allowing developers to switch between different
 *   setups based on project requirements.
 *
 * Features:
 *   - Platform selection macros to switch between ITE and CST platforms.
 *   - Inclusion of standard and platform-specific libraries.
 *   - Definition of general-purpose macros for bit manipulation, system timing,
 *     and mathematical operations.
 *   - Abstracted delay and system tick retrieval for supported platforms.
 *
 * Usage:
 *   1. Define the appropriate platform macro (e.g., PLATFORM_ITE_OPEN_RTOS)
 *      before including this file.
 *   2. Use the provided macros and utility functions as needed for platform-
 *      specific development.
 *
 * Notes:
 *   - Ensure all required libraries and headers are available for the target
 *     platform.
 *   - Validate compatibility between the selected platform and RTOS.
 *   - This file is designed for internal use within the Octopus Task Manager
 *     project.
 *
 * License:
 * Copyright (c) 2024 Octopus Team
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 *

 ******************************************************************************/

#include "octopus_platform.h" ///< Platform-specific settings (e.g. clock, GPIO defs)

// #define MCU_CPU_CLOCK_MHZ       64// Modify for your MCU clock speed (e.g., 168MHz for STM32F4)

// Initialize DWT cycle counter for microsecond resolution
void platform_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// uint32_t platform_dwt_get_us(void) {
//     return DWT->CYCCNT / MCU_CPU_CLOCK_MHZ;
// }
void dwt_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks)
        ;
}
// Delay Function ---
// Platform-specific delay using DWT if available
// Delay Wrapper
void delay_us(uint32_t us)
{
#ifdef DWT_DELAY_FUNCTION
    dwt_delay_us(us);
#else
    volatile uint32_t i;
    while (us--)
    {
        for (i = 0; i < 10; i++)
            __asm("nop");
    }
#endif
}

void delay_ms(uint32_t ms)
{
    delay_us(ms * 1000);
}
