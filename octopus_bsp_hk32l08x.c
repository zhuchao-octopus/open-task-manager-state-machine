////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Includes ------------------------------------------------------------------*/
#include "octopus_platform.h"

#ifdef TASK_MANAGER_STATE_MACHINE_MCU
#include <string.h>	  // Include string functions for string manipulation
#include "hk32l0xx.h" // Include the HK32L0xx library for MCU-specific definitions and functions
#include "octopus_bsp_hk32l08x.h"

#include "octopus_uart_hal.h"
#include "octopus_log.h"
#include "octopus_gpio_hal.h"
#include "octopus_system.h"
#include "octopus_flash.h"
#include "octopus_uart_upf.h"
#include "octopus_ling_hui_liion2.h"
#include "octopus_bafang.h"
#include "octopus_bt.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Exported macro ------------------------------------------------------------*/
#define LPUARTx_TXIO_PORT GPIOB
#define LPUARTx_TX_PIN GPIO_Pin_6
#define LPUARTx_AF_TX_PIN GPIO_PinSource6
#define LPUARTx_TXIO_CLK_CMD RCC_AHBPeriphClockCmd /* TX IO clock Enable */
#define LPUARTx_TXIO_CLK RCC_AHBPeriph_GPIOB

#define LPUARTx_RXIO_PORT GPIOB
#define LPUARTx_RX_PIN GPIO_Pin_7
#define LPUARTx_AF_RX_PIN GPIO_PinSource7
#define LPUARTx_RXIO_CLK_CMD RCC_AHBPeriphClockCmd /* RX IO clock Enable */
#define LPUARTx_RXIO_CLK RCC_AHBPeriph_GPIOB

#define LPUARTx_AF_SELECT GPIO_AF_3 /* AFIO SELECT Reference datasheet 6.8 table*/

#define LPUARTx_IRQn LPUART_IRQn
#define LPUARTx_IRQHandler LPUART_IRQHandler   /* LPUART1 interrupt handle */
#define LPUARTx_CLK_CMD RCC_APB1PeriphClockCmd /* LPUART1 clock Enable */
#define LPUARTx_CLK RCC_APB1Periph_LPUART1

/* LPUART1 STOP mode config clock */
#define LPUARTx_STOPCLK_COFIG RCC_LPUART1CLKConfig
#define LPUARTx_STOPCLK RCC_SELECTIONCLK_LSE

/* Private defines -----------------------------------------------------------*/
#define USART2_RX_BUF_SIZE 128
#define USART2_TX_BUF_SIZE 128

#define UART3_RX_BUF_SIZE 128
#define UART3_TX_BUF_SIZE 128

/* Macros to get peripheral addresses */
#define USART_TDR_ADDRESS(x) ((uint32_t)&((x)->TDR)) // Get the address of USART transmit data register
#define USART_RDR_ADDRESS(x) ((uint32_t)&((x)->RDR)) // Get the address of USART receive data register

// #define DMA_REMAP_MASK(channel)    (0xF << (channel * 4))  // Simplified mask
// #define DMA_REMAP_REG              DMA1->CSELR

#define USART2_RX_DMA_CHANNEL DMA_Channel6
#define USART2_TX_DMA_CHANNEL DMA_Channel7

#define UART3_RX_DMA_CHANNEL DMA_Channel2
#define UART3_TX_DMA_CHANNEL DMA_Channel3

#define USART2_RX_REMAP DMA_CSELR_CH6_USART2_RX
#define USART2_TX_REMAP DMA_CSELR_CH7_USART2_TX

#define UART3_RX_REMAP DMA_CSELR_CH2_UART3_RX
#define UART3_TX_REMAP DMA_CSELR_CH3_UART3_TX

#define UART2_RX_DMA_FLAG DMA1_FLAG_TC6
#define UART2_TX_DMA_FLAG DMA1_FLAG_TC7

#define UART3_RX_DMA_FLAG DMA1_FLAG_TC2
#define UART3_TX_DMA_FLAG DMA1_FLAG_TC3

/* Global buffers -----------------------------------------------------------*/
#ifdef UART2_DMA_MODE
uint8_t UART2_TxRingBuffer[USART2_TX_BUF_SIZE] = {0};	   // Ring buffer for USART2 transmit
uint8_t UART2_RxDoubleBuffer[2][USART2_RX_BUF_SIZE] = {0}; // Double buffer for USART2 receive (for high-speed reception)
#endif

#ifdef UART3_DMA_MODE
uint8_t UART3_TxBuffer1[UART3_TX_BUF_SIZE] = {0}; // Transmit buffer for USART1
// uint8_t UART3_RxBuffer1[UART3_RX_BUF_SIZE] = {0};    // Receive buffer for USART1
uint8_t UART3_RxDoubleBuffer[2][UART3_RX_BUF_SIZE] = {0}; // Double buffer for USART2 receive (for high-speed reception)
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef TASK_MANAGER_STATE_MACHINE_CAN
// Define the maximum length of data that can be transmitted or received in a CAN frame.
// According to the CAN protocol, each data frame can carry up to 8 bytes.
#define CAN_DATA_LENGTH 8

// Define a global variable to hold the CAN transmit message structure.
// This structure contains all necessary fields to describe a CAN frame to be transmitted,
// including ID, data length, identifier type, remote frame flag, and the actual data payload.
CanTxMsg CanTxMessage = {0};
// CanTxMsg TxMessage = {0};
/*
 * CanTxMsg structure members:
 * - StdId: Standard Identifier (11-bit) used when sending standard frames.
 * - ExtId: Extended Identifier (29-bit) used when sending extended frames.
 * - IDE: Identifier Type (CAN_ID_STD or CAN_ID_EXT) indicating standard or extended frame.
 * - RTR: Remote Transmission Request (CAN_RTR_DATA or CAN_RTR_REMOTE) - data or remote frame.
 * - DLC: Data Length Code, indicates number of valid bytes in the data field (0~8).
 * - Data[8]: Data payload to be transmitted (maximum 8 bytes).
 */

// Define a global variable to hold the CAN receive message structure.
// This structure will be filled when a CAN message is received from the FIFO.
// It holds all the relevant fields to interpret the received message.
CanRxMsg CanRxMessage = {0};

/*
 * CanRxMsg structure members:
 * - StdId: Standard Identifier (11-bit) of the received frame.
 * - ExtId: Extended Identifier (29-bit) of the received frame.
 * - IDE: Identifier Type indicating whether the message is a standard or extended frame.
 * - RTR: Remote Transmission Request flag indicating if the frame is a data or remote frame.
 * - DLC: Data Length Code, number of valid data bytes in the Data[] array (0~8).
 * - Data[8]: Actual data bytes received from the CAN bus.
 * - FMI: Filter Match Index indicating which configured filter accepted this frame.
 */
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile uint32_t system_tick_counter_ms = 0;
volatile uint32_t system_timer_tick_50us = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* USART/DMA Structures -----------------------------------------------------*/
// USART_InitTypeDef USART_InitStructure;    			 // USART initialization structure
// DMA_InitTypeDef  DMA_InitStructure;       			 // DMA initialization structure

/* Flags --------------------------------------------------------------------*/
volatile uint8_t uart2_rx_processing = 0;
volatile uint8_t uart2_tx_busy = 0;		 // Flag indicating if USART2 TX is busy (DMA in progress)
volatile uint8_t uart2_rx_buf_index = 0; // Index to toggle between RX double buffers

volatile uint8_t uart3_rx_processing = 0;
volatile uint8_t uart3_tx_busy = 0;		 // Flag indicating if USART2 TX is busy (DMA in progress)
volatile uint8_t uart3_rx_buf_index = 0; // Index to toggle between RX double buffers

/* Function Prototypes ------------------------------------------------------*/
void USART2_IRQHandler(void); // USART2 interrupt handler
void UART3_IRQHandler(void);  // UART3 interrupt handler

void DMA1_Channel4_5_IRQHandler(void); // DMA interrupt handler for UART3 TX
void DMA1_Channel6_7_IRQHandler(void); // DMA interrupt handler for USART2 TX and RX

void UART2_Config_IRQ_STOP_Mode(void);

extern void hal_timer_interrupt_callback(uint8_t event);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Initialize DWT cycle counter for microsecond resolution
void platform_dwt_init(void)
{
	// CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// uint32_t platform_dwt_get_us(void) {
//     return DWT->CYCCNT / MCU_CPU_CLOCK_MHZ;
// }
void dwt_delay_us(uint32_t us)
{
	// uint32_t start = DWT->CYCCNT;
	// uint32_t ticks = us * (SystemCoreClock / 1000000);
	// while ((DWT->CYCCNT - start) < ticks)
	//     ;
}

// Delay Function ---
// Platform-specific delay using DWT if available
// Delay Wrapper
void platform_delay_us(uint32_t us)
{

#ifdef DWT_DELAY_FUNCTION
	dwt_delay_us(us);
#else

#endif
}

void platform_delay_ms(uint32_t ms)
{
	platform_delay_us(ms * 1000);
}
/**
 * @brief  This function handles SysTick Handler.
 * @retval None
 */
void SysTick_Handler(void)
{
	system_tick_counter_ms++; // Increment the millisecond counter
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////BT/debug
__weak void UART1_RX_Callback(uint8_t *buffer, uint16_t length) ///////BT
{
#ifdef TASK_MANAGER_STATE_MACHINE_BT_MUSIC
	upf_receive_callback(upf_module_info_BT_MUSIC, buffer, length);
#endif
}

// Weak callback function for USART2 RX  ///////BLE-MCU
__weak void UART2_RX_Callback(uint8_t *buffer, uint16_t length)
{
	// UART2_Send_Buffer(buffer,length);
	hal_com_uart_receive_callback_ptl_1(buffer, length);
}

// Weak callback function for UART3 RX
///////SOC
__weak void UART3_RX_Callback(uint8_t *buffer, uint16_t length)
{
	// LOG_LEVEL("UART3_RX_Callback bytes:%d ",length);
	// UART3_Send_Buffer(buffer,length);
	hal_com_uart_receive_callback_ptl_1(buffer, length); ////SOC
}

///////4G/GPS
__weak void UART4_RX_Callback(uint8_t *buffer, uint16_t length)
{
	// UART4_Send_Buffer(buffer,length);
#ifdef TASK_MANAGER_STATE_MACHINE_LOT4G
	upf_receive_callback(upf_module_info_LOT4G, buffer, length);
#endif
}

// Weak callback function for LPUART RX
///////bafang/the third protocol
__weak void LPUART_RX_Callback(uint8_t *buffer, uint16_t length)
{
	/// LPUART_Send_Buffer(buffer,length);
	/// UART1_Send_Buffer(buffer,length);
#ifdef TASK_MANAGER_STATE_MACHINE_BAFANG
	upf_receive_callback(upf_module_info_BAFANG, buffer, length);
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_LING_HUI_LIION2
	upf_receive_callback(upf_module_info_LING_HUI_LIION2, buffer, length);
#endif
}

void IWDG_Init(uint8_t prer, uint16_t reload)
{
	// Enable write access to IWDG_PR and IWDG_RLR registers
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	// Set prescaler value (PR)
	// Example: IWDG_Prescaler_64, valid values: 4/8/16/32/64/128/256
	IWDG_SetPrescaler(prer);

	// Set reload value (RL)
	// Value range: 0x000 to 0xFFF (12-bit)
	IWDG_SetReload(reload);

	// Reload the counter (reset watchdog timer)
	IWDG_ReloadCounter();

	// Enable the watchdog
	IWDG_Enable();
}

void IWDG_Feed(void)
{
	IWDG_ReloadCounter(); // Reset watchdog timer
}

void SYS_Config(void)
{
	// Configure SysTick timer to interrupt every 1 ms
	SysTick->LOAD = (SystemCoreClock / 1000) - 1; // Set reload register for 1ms interval
	SysTick->VAL = 0;							  // Clear current value
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |  // Use processor clock
					SysTick_CTRL_TICKINT_Msk |	  // Enable SysTick interrupt
					SysTick_CTRL_ENABLE_Msk;	  // Enable SysTick timer

	// Set the priority of the SysTick interrupt
	NVIC_SetPriority(SysTick_IRQn, 0x3); // 0x0 is the highest priority
}
/**
 * @brief  Configures GPIO pins for USART1 and USART2.
 * @param  None
 * @retval None
 */
void GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		// Alternate function mode
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		// Open-drain output//GPIO_OType_PP;// Push-pull output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	// GPIO_PuPd_UP;// Pull-up resistor enabled
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3; // High speed

	///////////////////////////////////////////////////////////////////////////
	// USART1 Configuration: PA2 (TX), PA3 (RX)
	// Used for debug or external communication(e.g., debug)
	///////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // PA2 - USART1 TX
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1); // AF1 for USART1 TX

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // PA3 - USART1 RX
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1); // AF1 for USART1 RX

	///////////////////////////////////////////////////////////////////////////
	// USART2 Configuration: PA9 (TX), PA10 (RX)
	// Used for general serial communication
	///////////////////////////////////////////////////////////////////////////
	// PA9 - USART2 TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_9); // AF9 for USART2 TX

	// PA10 - USART2 RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_9); // AF9 for USART2 RX

	///////////////////////////////////////////////////////////////////////////
	// UART3 Configuration: PA4 (TX), PA5 (RX)
	// Used for F113 or other MCU communication
	///////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // PA4 - UART3 TX
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_5); // AF5 for UART3 TX

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // PA5 - UART3 RX
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5); // AF5 for UART3 RX

	///////////////////////////////////////////////////////////////////////////////
	// UART4 Configuration: PA0 (TX), PA1 (RX)
	// Used for GPS module or other serial device communication
	///////////////////////////////////////////////////////////////////////////////
	// PA0 - UART4 TX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_9); // AF9 for UART4 TX

	// PA1 - UART4 RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_9); // AF9 for UART4 RX

#if 0
	/////////////////////////////////////////////////////////////////////////////
	// LPUART Configuration: PB6 (TX), PB7 (RX)
	// Used for communication via LPUART4
	/////////////////////////////////////////////////////////////////////////////	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // Open-drain output;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = LPUARTx_TX_PIN;
	GPIO_Init(LPUARTx_TXIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LPUARTx_RX_PIN;
	GPIO_Init(LPUARTx_RXIO_PORT, &GPIO_InitStructure);

	/* Connect PXx to LPUART1 Tx Rx */
	GPIO_PinAFConfig(LPUARTx_TXIO_PORT, LPUARTx_AF_TX_PIN, LPUARTx_AF_SELECT);
	GPIO_PinAFConfig(LPUARTx_RXIO_PORT, LPUARTx_AF_RX_PIN, LPUARTx_AF_SELECT);
#endif
	///////////////////////////////////////////////////////////////////////////
	// CAN Bus Configuration: PA6 (CAN_RX), PA7 (CAN_TX)
	///////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // PA6 - CAN RX
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_4); // AF4 for CAN RX

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // PA7 - CAN TX
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_4); // AF4 for CAN TX

	///////////////////////////////////////////////////////////////////////////
	// Output Pins: Power enable/control GPIOs
	///////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;		// Output mode
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		// GPIO_OType_OD;		// Push-pull output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	// No pull-up/down
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3; // Medium speed

	// PA11 - LED power enable (LEDPW_EN)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PA15 - 3.3V power enable for MCU (MCU_3V3_EN)
	// GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_0);
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PB4 - SWB+ power enable (SWB+_EN)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	// hal_gpio_write(GPIO_POWER_SWITCH_GROUP, GPIO_POWER_SWITCH_PIN, BIT_SET);

	// PB5 - CAN standby control (MCU_CTL_CAN_STBY)
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	///////////////////////////////////////////////////////////////////////////
	// BL_EN_PWM1 (PB1): Backlight enable or PWM control
	// NOTE: If using PWM, reconfigure as alternate function and assign TIMx
	///////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; // Output mode (default on/off control)
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	///////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////
	// BAT_DET (PA8): Battery voltage detect input (ADC channel)
	///////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;	 // Analog mode for ADC
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; // No pull-up/down
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	///////////////////////////////////////////////////////////////////////////
	// KEY_POW_DET (PA12): Power key status input
	///////////////////////////////////////////////////////////////////////////
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 // Input mode
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 // Input mode
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/**
 * @brief  Configures the RCC (clock) for peripherals.
 * @param  None
 * @retval None
 */
void RCC_Config(void)
{
	// Enable clock for GPIOA and GPIOB (required for USART1, USART2, UART3 pins)
	// GPIOA and GPIOB are used for peripheral pins such as USART1 TX/RX, USART2 TX/RX, etc.
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC, ENABLE);

	// Enable the clock for USART1 (on APB2)
	// USART1 is used for debugging purposes, e.g., via UART over USB or serial communication
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); // Enable USART1 clock (Debug UART)

	// Enable the clock for USART2 (both on APB1)
	// USART2 is used for communication with F133 (e.g., for data transmission)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// Enable the clock for UART3 (both on APB1)
	// UART3 is used for communication with BLE module (Bluetooth Low Energy communication)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART3, ENABLE);

	// Enable the clock for UART4 (on APB1)
	// UART4 is used for communication with GPS module
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE); // Enable UART4 clock (GPS)

	// Enable the clock for LPUART1 (Low Power UART)
	// LPUART1 is used for low-power communication, often used in low-power modes or for energy-efficient communication
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_LPUART1, ENABLE); // Enable LPUART1 clock

	// Enable the clock for CAN (Controller Area Network)
	// CAN is used for communication in automotive or industrial environments
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN, ENABLE); // Enable CAN clock (Controller Area Network)

	// Enable the clock for PWR (Power Control)
	// The PWR module is responsible for power management, including low-power modes and voltage regulation
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); // Enable PWR clock (Power Control)

	// Enable the clock for DAC (Digital-to-Analog Converter)
	// DAC is used for generating analog signals from digital data
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); // Enable DAC clock (Digital to Analog Converter)

	// Enable DMA clock (Direct Memory Access) for USART2 RX/TX
	// DMA is used to offload data transfer tasks, reducing CPU load and improving efficiency for high-speed data transfers
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA, ENABLE); // Enable DMA clock (for USART2 DMA RX/TX)

	// TIM3 clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	/* Enable the LSE */
	RCC_LSEConfig(RCC_LSE_ON);
}

void Sleep_Mode_Wakeup_Config(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configure Button pin as input */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Generate software interrupt: simulate a falling edge applied on EXTI2 line */
	EXTI_GenerateSWInterrupt(EXTI_Line2);
	/* Connect Button EXTI Line to Button GPIO Pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource12);

	/* Configure Button EXTI line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set Button EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// NVIC_SetPriority(EXTI4_15_IRQn,3);
#ifdef HARDWARE_BSP_UART_2
	UART2_Config_IRQ_STOP_Mode();
#endif
}

////extern  void TaskManagerStateMachineInit(void);
void native_enter_sleep_mode(void)
{
	LOG_LEVEL("SYSTEM & TASK MANAGER SCADULER ENTER SLEEP LOW POWER MODE.\r\n");
	Sleep_Mode_Wakeup_Config();
	/* Request to enter STOP mode with regulator in low power mode */
	PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);
	// RCC_HSI16Cmd(ENABLE);
	// while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	RCC_USART2CLKConfig(RCC_SELECTIONCLK_PCLK);

	USART_STOPModeCmd(USART2, DISABLE);
	USART_Cmd(USART2, DISABLE);
	USART_DeInit(USART2);
	USART_ClearITPendingBit(USART2, USART_IT_WU);
	USART_ClearFlag(USART2, USART_FLAG_WU);
	USART_ClearFlag(USART2, USART_FLAG_RXNE);

	/// while(1){}
	SystemInit();
	GPIO_Config();
	RCC_Config();
	EEPROM_Init();

#ifdef HARDWARE_BSP_UART_1
	UART1_Config_IRQ();
#endif
#ifdef HARDWARE_BSP_UART_2
	UART2_Config_IRQ();
#endif
#ifdef HARDWARE_BSP_UART_3
	UART3_Config_IRQ();
#endif
#ifdef HARDWARE_BSP_UART_4
	UART4_Config_IRQ();
#endif

#ifdef HARDWARE_BSP_LUART_1
	LPUART_WakeStop_Config();
#endif
	dbg_log_set_channel(TASK_MANAGER_STATE_MACHINE_LOG_CHANNEL);
#ifdef TASK_MANAGER_STATE_MACHINE_CAN
	CAN_Config();
#endif
	LOG_NONE("\r\n");
	LOG_LEVEL("SYSTEM & TASK MANAGER SCADULER EXIT SLEEP MODE.\r\n");
	// NVIC_SystemReset();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef HARDWARE_BSP_UART_1
void UART1_Config_IRQ(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);

	// NVIC_SetPriority(USART1_IRQn, 1);
	// Configure NVIC for USART1 interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);
}
#endif

#ifdef HARDWARE_BSP_UART_2
/* USART + DMA Init Function -------------------------------------------------*/
#ifdef UART2_DMA_MODE
void UART2_Config_DMA(void)
{
	/* USART2 Configuration */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	/* DMA Common Configuration */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	/* USART2 RX (Double Buffer DMA) */
	DMA_InitStructure.DMA_BufferSize = USART2_RX_BUF_SIZE;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART2_RxDoubleBuffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART_RDR_ADDRESS(USART2);
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(USART2_RX_DMA_CHANNEL, &DMA_InitStructure);
	DMA_RemapConfig(DMA, USART2_RX_REMAP);
	// DMA_ITConfig(USART2_RX_DMA_CHANNEL, DMA_IT_HT, ENABLE);
	USART_ClearFlag(USART2, USART_FLAG_TC | USART_FLAG_RXNE);

	/* USART2 TX (Ring Buffer DMA) */
	DMA_InitStructure.DMA_BufferSize = USART2_TX_BUF_SIZE;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART2_TxRingBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART_TDR_ADDRESS(USART2);
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(USART2_TX_DMA_CHANNEL, &DMA_InitStructure);
	DMA_RemapConfig(DMA, USART2_TX_REMAP);
	DMA_ITConfig(USART2_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);

	/* Enable DMA Request and Channels */
	USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(USART2_RX_DMA_CHANNEL, ENABLE);
	DMA_Cmd(USART2_TX_DMA_CHANNEL, DISABLE);

	/* Enable USART2 and its interrupts */
	USART_Cmd(USART2, ENABLE);
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	NVIC_EnableIRQ(USART2_IRQn);

	/* Enable DMA Interrupts */
	NVIC_SetPriority(DMA_CH4_7_IRQn, 2);
	NVIC_EnableIRQ(DMA_CH4_7_IRQn);
}
#endif

void UART2_Config_IRQ_STOP_Mode(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStr;
	// Configure UART2
	USART_InitStructure.USART_BaudRate = 115200; // BLE
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* Enable PWR APB clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	RCC_HSI16Cmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_HSI16RDY) == 0)
	{
	}
	PWR_BackupAccessCmd(DISABLE);
	RCC_USART2CLKConfig(RCC_SELECTIONCLK_HSI16);

	EXTI_InitStr.EXTI_Line = EXTI_Line26;
	EXTI_InitStr.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStr.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStr.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStr);

	USART_STOPModeCmd(USART2, ENABLE);
	USART_StopModeWakeUpSourceConfig(USART2, USART_WakeUpSource_StartBit);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Enable RX interrupt
	USART_ITConfig(USART2, USART_IT_WU, ENABLE);

	USART_ClearFlag(USART2, USART_FLAG_RXNE);
	USART_ClearFlag(USART2, USART_FLAG_WU);

	// NVIC_SetPriority(USART2_IRQn, 3);
	NVIC_EnableIRQ(USART2_IRQn);
	// Enable USART2
	USART_Cmd(USART2, ENABLE);
}

void UART2_Config_IRQ(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Configure UART2
	USART_InitStructure.USART_BaudRate = 115200; // BLE
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // Enable RX interrupt
	// NVIC_SetPriority(USART2_IRQn, 3);
	NVIC_EnableIRQ(USART2_IRQn);
	// Enable USART2
	USART_Cmd(USART2, ENABLE);
}
#endif

#ifdef HARDWARE_BSP_UART_3
void UART3_Config_IRQ(void)
{
	USART_InitTypeDef USART_InitStructure;
	// Configure UART3
	USART_InitStructure.USART_BaudRate = 115200; // SOC
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(UART3, &USART_InitStructure);

	// Enable UART3
	USART_Cmd(UART3, ENABLE);
	USART_ITConfig(UART3, USART_IT_RXNE, ENABLE); // Enable RX interrupt
	NVIC_SetPriority(UART3_4_IRQn, 2);
	NVIC_EnableIRQ(UART3_4_IRQn);
}

#ifdef UART3_DMA_MODE
void UART3_Config_DMA(void)
{
	/* USART3 Configuration */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(UART3, &USART_InitStructure);

	/* DMA Common Configuration */
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	/* USART3 RX (Double Buffer DMA) */
	DMA_InitStructure.DMA_BufferSize = UART3_RX_BUF_SIZE;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART3_RxDoubleBuffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART_RDR_ADDRESS(UART3);
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(UART3_RX_DMA_CHANNEL, &DMA_InitStructure);
	DMA_RemapConfig(DMA, UART3_RX_REMAP);
	// DMA_ITConfig(UART3_RX_DMA_CHANNEL, DMA_IT_HT, ENABLE);
	USART_ClearFlag(UART3, USART_FLAG_TC | USART_FLAG_RXNE);

	/* UART3 TX (Ring Buffer DMA) */
	DMA_InitStructure.DMA_BufferSize = UART3_TX_BUF_SIZE;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART3_TxBuffer1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART_TDR_ADDRESS(UART3);
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(UART3_TX_DMA_CHANNEL, &DMA_InitStructure);
	DMA_RemapConfig(DMA, UART3_TX_REMAP);
	DMA_ITConfig(UART3_TX_DMA_CHANNEL, DMA_IT_TC, ENABLE);

	/* Enable DMA Request and Channels */
	USART_DMACmd(UART3, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
	DMA_Cmd(UART3_RX_DMA_CHANNEL, ENABLE);
	// DMA_Cmd(UART3_TX_DMA_CHANNEL, ENABLE);

	/* Enable USART3 and its interrupts */
	USART_Cmd(UART3, ENABLE);
	USART_ITConfig(UART3, USART_IT_IDLE, ENABLE);
	NVIC_EnableIRQ(UART3_4_IRQn);

	/* Enable DMA Interrupts */
	NVIC_SetPriority(DMA_CH2_3_IRQn, 1);
	NVIC_EnableIRQ(DMA_CH2_3_IRQn);
}
#endif
#endif

#ifdef HARDWARE_BSP_UART_4
void UART4_Config_IRQ(void)
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 57600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(UART4, &USART_InitStructure);
	USART_Cmd(UART4, ENABLE);

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

	NVIC_SetPriority(UART3_4_IRQn, 2);
	NVIC_EnableIRQ(UART3_4_IRQn);
}
#endif

#ifdef HARDWARE_BSP_LUART_1
void LPUART_WakeStop_Config(void)
{
	/* Enable PWR APB clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

	LPUART_InitTypeDef LPUART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	/// EXTI_InitTypeDef EXTI_InitStructure;

	/* Enable GPIO clock and Enable LPUART Clock,LPUARTx,LPUARTy */
	LPUARTx_TXIO_CLK_CMD(LPUARTx_TXIO_CLK, ENABLE);
	LPUARTx_RXIO_CLK_CMD(LPUARTx_RXIO_CLK, ENABLE);
	LPUARTx_CLK_CMD(LPUARTx_CLK, ENABLE);

	/* Enable LSE clock */
	// PWR_BackupAccessCmd(ENABLE);
	// RCC_LSEConfig(RCC_LSE_ON);
	// PWR_BackupAccessCmd(DISABLE);
	/* Configure the LSE as LPUART clock */
	// LPUARTx_STOPCLK_COFIG(LPUARTx_STOPCLK);
	// RCC_HSI16Cmd(ENABLE);
	/* Configure the LSE as LPUART clock */
	// RCC_LPUART1CLKConfig(RCC_SELECTIONCLK_HSI16);

	RCC_LSICmd(ENABLE);
	RCC_LPUART1CLKConfig(RCC_SELECTIONCLK_LSI);

	/* GPIO Configure RX TX */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; // Open-drain output;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = LPUARTx_TX_PIN;
	GPIO_Init(LPUARTx_TXIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LPUARTx_RX_PIN;
	GPIO_Init(LPUARTx_RXIO_PORT, &GPIO_InitStructure);

	/* Connect PXx to LPUART1 Tx Rx */
	GPIO_PinAFConfig(LPUARTx_TXIO_PORT, LPUARTx_AF_TX_PIN, LPUARTx_AF_SELECT);
	GPIO_PinAFConfig(LPUARTx_RXIO_PORT, LPUARTx_AF_RX_PIN, LPUARTx_AF_SELECT);

/* LPUARTx configured */
#if defined(TASK_MANAGER_STATE_MACHINE_LING_HUI_LIION2)
	LPUART_InitStructure.LPUART_BaudRate = 9600;
#elif defined(TASK_MANAGER_STATE_MACHINE_BAFANG)
	LPUART_InitStructure.LPUART_BaudRate = 1200;
#else
	LPUART_InitStructure.LPUART_BaudRate = 115200;
#endif

	LPUART_InitStructure.LPUART_HardwareFlowControl = LPUART_HardwareFlowControl_None; /* Hardware flow control disabled (RTS and CTS signals) */
	LPUART_InitStructure.LPUART_Mode = LPUART_Mode_Rx | LPUART_Mode_Tx;				   /* Receive and transmit enabled */

	/* When using Parity the word length must be configured to 9 bits */
	LPUART_InitStructure.LPUART_Parity = LPUART_Parity_No;		   /* No parity */
	LPUART_InitStructure.LPUART_StopBits = LPUART_StopBits_1;	   /* One Stop Bit */
	LPUART_InitStructure.LPUART_WordLength = LPUART_WordLength_8b; /*Word Length = 8 Bits */
	LPUART_Init(LPUART, &LPUART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = LPUARTx_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	LPUART_ITConfig(LPUART, LPUART_IT_RXNE, ENABLE);
	/* Enable LPUART */
	LPUART_Cmd(LPUART, ENABLE);

	/* NVIC configuration */
	/* Enable the LPUARTx Interrupt. The interrupt of LPUART is shared with EXTI28, so EXTI_Line28 is enabled */
	/// EXTI_InitStructure.EXTI_Line = EXTI_Line28;
	/// EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	/// EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	/// EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	/// EXTI_Init(&EXTI_InitStructure);
	/* Configure the wake up Method = Start bit */
	/// LPUART_StopModeWakeUpSourceConfig(LPUART, LPUART_WakeUpSource_StartBit);
	/* Before entering the LPUART in STOP mode the REACK flag must be checked to ensure the LPUART RX is ready */
	/// while (LPUART_GetFlagStatus(LPUART, LPUART_FLAG_REACK) == RESET)
	///{
	/// }
	/* Enable LPUART STOP mode by setting the UESM bit in the CR1 register */
	// LPUART_STOPModeCmd(LPUART, ENABLE);
	/* Enable the wake up from stop Interrupt WUF */
	// LPUART_ITConfig(LPUART, LPUART_IT_WU, ENABLE);
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef HARDWARE_BSP_UART_1
void USART1_IRQHandler(void)
{
	// Non-DMA mode: use RXNE interrupt
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(USART1); // Read received byte
		UART1_RX_Callback(&data, 1);					   // Pass it to callback
	}

	// Check for and clear overrun error (ORE)
	// if (USART_GetITStatus(USART1, USART_IT_ORE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_ORE);
		// Optional: log or handle overrun error if needed
	}

	// Check for and clear framing error (FE)
	// if (USART_GetITStatus(USART1, USART_IT_FE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_FE);
		// Optional: log or handle framing error if needed
	}

	// if (USART_GetITStatus(USART1, USART_IT_WU) == SET)
	{
		/* Clear The USART WU flag */
		USART_ClearITPendingBit(USART1, USART_IT_WU);
	}
}
#endif
/* USART2 Interrupt Handler --------------------------------------------------*/
#ifdef HARDWARE_BSP_UART_2
void USART2_IRQHandler(void)
{
#ifdef UART2_DMA_MODE
	// Check if IDLE line interrupt occurred (indicates RX completion)
	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		// 1. Clear the IDLE interrupt flag by reading the ISR and RDR
		// volatile uint32_t temp;
		// temp = USART2->ISR;  // Read ISR register to clear the interrupt flag
		// temp = USART2->RDR;  // Read RDR register to clear the idle flag (and retrieve data)
		USART_ClearITPendingBit(USART2, USART_IT_IDLE);
		if (uart2_rx_processing)
			return;
		uart2_rx_processing = 1;

		// 2. Disable DMA temporarily to process the received data
		DMA_Cmd(USART2_RX_DMA_CHANNEL, DISABLE);

		// 3. Calculate how many bytes were received (based on remaining DMA counter)
		uint16_t received_len = USART2_RX_BUF_SIZE - DMA_GetCurrDataCounter(USART2_RX_DMA_CHANNEL);

		// 4. Get the pointer to the received buffer (current buffer in use)
		uint8_t *rx_buf = UART2_RxDoubleBuffer[uart2_rx_buf_index];

		// 5. Switch to the other buffer (for double-buffering mechanism)
		uart2_rx_buf_index ^= 1; // Toggle between buffer 0 and buffer 1

		// 6. Reconfigure the DMA to point to the new buffer for continuous reception
		USART2_RX_DMA_CHANNEL->CHMAR = (uint32_t)UART2_RxDoubleBuffer[uart2_rx_buf_index]; // Set new memory address
		DMA_SetCurrDataCounter(USART2_RX_DMA_CHANNEL, USART2_RX_BUF_SIZE);				   // Reset DMA data counter to buffer size

		// 7. Re-enable DMA to continue receiving data
		DMA_Cmd(USART2_RX_DMA_CHANNEL, ENABLE);

		// 8. Call the user-defined callback function to process the received data
		UART2_RX_Callback(rx_buf, received_len);
		uart2_rx_processing = 0;
	}
#else
	// Non-DMA mode: use RXNE interrupt
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(USART2); // Read received byte
		UART2_RX_Callback(&data, 1);					   // Pass it to callback
	}

	// Check for and clear overrun error (ORE)
	// if (USART_GetITStatus(USART2, USART_IT_ORE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_ORE);
		// Optional: log or handle overrun error if needed
	}

	// Check for and clear framing error (FE)
	// if (USART_GetITStatus(USART2, USART_IT_FE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_FE);
		// Optional: log or handle framing error if needed
	}

	// if (USART_GetITStatus(USART2, USART_IT_WU) == SET)
	{
		/* Clear The USART WU flag */
		USART_ClearITPendingBit(USART2, USART_IT_WU);
	}
#endif
}
#endif

/* DMA Interrupt Handlers for USART2 ---------------------------*/
#ifdef UART2_DMA_MODE
void DMA_CH4_7_IRQHandler(void) // DMA interrupt for USART2 TX and RX
{
	uint8_t next_buf_index = 0;
	if (DMA_GetITStatus(UART2_TX_DMA_FLAG)) // Check for TX transfer complete interrupt
	{
		DMA_ClearITPendingBit(UART2_TX_DMA_FLAG); // Clear interrupt flag
		uart2_tx_busy = 0;						  // Reset TX busy flag
	}

	if (DMA_GetITStatus(DMA1_IT_HT6)) // Check for RX half-transfer interrupt
	{
		DMA_ClearITPendingBit(DMA1_IT_HT6);		 // Clear interrupt flag
												 // Before processing the data, switch to the other buffer
		next_buf_index = uart2_rx_buf_index ^ 1; // Toggle between 0 and 1

		// Now process the data in the newly selected buffer
		uint16_t received_len = USART2_RX_BUF_SIZE / 2; // Entire buffer filled
		UART2_RX_Callback(UART2_RxDoubleBuffer[next_buf_index], received_len);
	}

	// Check for DMA1 Channel 6 Transfer Complete (TC) interrupt (RX complete)
	if (DMA_GetITStatus(UART2_RX_DMA_FLAG))
	{
		// 1. Clear the IDLE interrupt flag by reading the ISR and RDR
		DMA_ClearITPendingBit(UART2_RX_DMA_FLAG); // Clear interrupt flag
		if (uart2_rx_processing)
			return;

		uart2_rx_processing = 1;

		// 2. Disable DMA temporarily to process the received data
		DMA_Cmd(USART2_RX_DMA_CHANNEL, DISABLE);

		// 3. Calculate how many bytes were received (based on remaining DMA counter)
		uint16_t received_len = USART2_RX_BUF_SIZE;

		// 4. Get the pointer to the received buffer (current buffer in use)
		uint8_t *rx_buf = UART2_RxDoubleBuffer[uart2_rx_buf_index];

		// 5. Switch to the other buffer (for double-buffering mechanism)
		uart2_rx_buf_index ^= 1; // Toggle between buffer 0 and buffer 1

		// 6. Reconfigure the DMA to point to the new buffer for continuous reception
		USART2_RX_DMA_CHANNEL->CHMAR = (uint32_t)UART2_RxDoubleBuffer[uart2_rx_buf_index]; // Set new memory address
		DMA_SetCurrDataCounter(USART2_RX_DMA_CHANNEL, USART2_RX_BUF_SIZE);				   // Reset DMA data counter to buffer size

		// 7. Re-enable DMA to continue receiving data
		DMA_Cmd(USART2_RX_DMA_CHANNEL, ENABLE);

		// 8. Call the user-defined callback function to process the received data
		UART2_RX_Callback(rx_buf, received_len);
		uart2_rx_processing = 0;
	}
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* UART3 Interrupt Handler --------------------------------------------------*/
void UART3_4_IRQHandler(void)
{
#ifdef UART3_DMA_MODE
	// Check if IDLE line interrupt occurred (indicates RX completion)
	if (USART_GetITStatus(UART3, USART_IT_IDLE) != RESET)
	{
		// 1. Clear the IDLE interrupt flag by reading ISR and RDR
		volatile uint32_t temp;
		temp = UART3->ISR; // Read ISR to clear IDLE flag
		temp = UART3->RDR; // Read RDR to complete flag clearing

		if (uart3_rx_processing)
			return; // Prevent re-entry
		uart3_rx_processing = 1;

		// 2. Disable DMA temporarily to process the received data
		DMA_Cmd(UART3_RX_DMA_CHANNEL, DISABLE);

		// 3. Calculate how many bytes were received
		uint16_t received_len = UART3_RX_BUF_SIZE - DMA_GetCurrDataCounter(UART3_RX_DMA_CHANNEL);

		// 4. Get the current buffer pointer
		uint8_t *rx_buf = UART3_RxDoubleBuffer[uart3_rx_buf_index];

		// 5. Switch to the other buffer
		uart3_rx_buf_index ^= 1;

		// 6. Configure DMA to new buffer
		UART3_RX_DMA_CHANNEL->CHMAR = (uint32_t)UART3_RxDoubleBuffer[uart3_rx_buf_index];
		DMA_SetCurrDataCounter(UART3_RX_DMA_CHANNEL, UART3_RX_BUF_SIZE);

		// 7. Re-enable DMA
		DMA_Cmd(UART3_RX_DMA_CHANNEL, ENABLE);

		// 8. Call user-defined function to handle data
		UART3_RX_Callback(rx_buf, received_len);

		uart3_rx_processing = 0;
	}

#else
	// Non-DMA mode: use RXNE interrupt
	if (USART_GetITStatus(UART3, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(UART3); // Read received byte
		UART3_RX_Callback(&data, 1);					  // Pass it to callback
	}

	// Optional: Clear overrun and framing error flags
	// if (USART_GetITStatus(UART3, USART_IT_ORE) != RESET)
	{
		USART_ClearITPendingBit(UART3, USART_IT_ORE);
	}

	// if (USART_GetITStatus(UART3, USART_IT_FE) != RESET)
	{
		USART_ClearITPendingBit(UART3, USART_IT_FE);
	}

	////////////////////////////////////////////////////////////////////////////////////
	// Non-DMA mode: use RXNE interrupt
	if (USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(UART4); // Read received byte
		UART4_RX_Callback(&data, 1);					  // Pass it to callback
	}
	// Optional: Clear overrun and framing error flags
	// if (USART_GetITStatus(UART4, USART_IT_ORE) != RESET)
	{
		USART_ClearITPendingBit(UART4, USART_IT_ORE);
	}

	// if (USART_GetITStatus(UART4, USART_IT_FE) != RESET)
	{
		USART_ClearITPendingBit(UART4, USART_IT_FE);
	}
#endif
}

/* DMA Interrupt Handlers for UART3  ---------------------------*/
#ifdef UART3_DMA_MODE
void DMA_CH2_3_IRQHandler(void) // DMA interrupt for UART3 TX and RX
{
	// uint8_t next_buf_index = 0;

	// === UART3 RX (e.g., DMA1 Channel 3) Transfer Complete ===
	if (DMA_GetITStatus(UART3_RX_DMA_FLAG))
	{
		DMA_ClearITPendingBit(UART3_RX_DMA_FLAG); // Clear RX complete flag

		if (uart3_rx_processing)
			return;
		uart3_rx_processing = 1;

		// Temporarily disable RX DMA to safely process buffer
		DMA_Cmd(UART3_RX_DMA_CHANNEL, DISABLE);

		// Use the full buffer size
		uint16_t received_len = UART3_RX_BUF_SIZE;

		// Pointer to current buffer
		uint8_t *rx_buf = UART3_RxDoubleBuffer[uart3_rx_buf_index];

		// Switch to next buffer
		uart3_rx_buf_index ^= 1;

		// Reconfigure DMA for the new buffer
		UART3_RX_DMA_CHANNEL->CHMAR = (uint32_t)UART3_RxDoubleBuffer[uart3_rx_buf_index];
		DMA_SetCurrDataCounter(UART3_RX_DMA_CHANNEL, UART3_RX_BUF_SIZE);

		// Restart DMA
		DMA_Cmd(UART3_RX_DMA_CHANNEL, ENABLE);

		// Callback with received data
		UART3_RX_Callback(rx_buf, received_len);

		uart3_rx_processing = 0;
	}

	// === UART3 TX (DMA1 Channel 2) Transfer Complete ===
	if (DMA_GetITStatus(UART3_TX_DMA_FLAG))
	{
		DMA_ClearITPendingBit(UART3_TX_DMA_FLAG); // Clear TX complete flag
		uart3_tx_busy = 0;						  // Mark TX as ready
	}
}
#endif

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#ifdef HARDWARE_BSP_LUART_1
/**
 * @brief  This function handles LPUART Handler.
 * @retval None
 */
void LPUART_IRQHandler(void)
{
	if (LPUART_GetITStatus(LPUART, LPUART_IT_WU) == SET)
	{
		/* Clear The LPUART WU flag */
		LPUART_ClearITPendingBit(LPUART, LPUART_IT_WU);
	}

	/* Check if Receive Data Register Not Empty (RXNE) interrupt is set */
	if (LPUART_GetITStatus(LPUART, LPUART_IT_RXNE) == SET)
	{
		uint8_t data = LPUART_ReceiveData(LPUART); // Read received byte
		// In receiver mode, store received data to both Cmd and Ack buffer
		LPUART_RX_Callback(&data, 1);
	}

	// Optionally: Clear TXE interrupt if enabled accidentally (defensive)
	if (LPUART_GetITStatus(LPUART, LPUART_IT_TXE) == SET)
	{
		// Disable transmit interrupt as we are only using receive mode
		LPUART_ITConfig(LPUART, LPUART_IT_TXE, DISABLE);
	}

	LPUART_ClearFlag(LPUART, LPUART_FLAG_ORE);
}
#endif
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef HARDWARE_BSP_UART_1
/**
 * @brief  Send a buffer of data through USART1 (blocking)
 * @param  buffer: pointer to the data to be sent
 * @param  len: length of the data in bytes
 * @retval None
 */
void UART1_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
	uint16_t i;
	uint32_t Timeout;
	FlagStatus Status;

	for (i = 0; i < length; i++)
	{
		// Send one byte
		USART_SendData(USART1, buffer[i]);

		// Wait for TXE flag (Transmit Data Register Empty)
		Timeout = 0;
		do
		{
			Status = USART_GetFlagStatus(USART1, USART_FLAG_TXE);
			Timeout++;
		} while ((Status == RESET) && (Timeout != 0xFFFF));
	}
}

void UART1_SendStr(const char *str)
{
	uint8_t len = strlen(str);
	if (len > 255)
		len = 255;
	for (uint8_t i = 0; i < len; i++)
	{
		USART_SendData(USART1, (uint8_t)str[i]);
		uint32_t Timeout = 0;
		while ((USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) && (Timeout++ < 0xFFFF))
			;
	}
}
#endif

#ifdef HARDWARE_BSP_UART_2
#ifdef UART2_DMA_MODE
/* USART2 String Transmission via DMA ------------------------------------*/
void UART2_Send_String_DMA(const char *str)
{
	if (uart2_tx_busy)
		return; // Return if TX is busy

	uint16_t len = strlen(str); // Calculate string length
	if (len <= 0)
		return;

	uart2_tx_busy = 1; // Set TX busy flag
	if (len > USART2_TX_BUF_SIZE)
		len = USART2_TX_BUF_SIZE;						// Limit length to buffer size
	memcpy(UART2_TxRingBuffer, str, len);				// Copy string to TX ring buffer
	DMA_Cmd(USART2_TX_DMA_CHANNEL, DISABLE);			// Disable DMA for USART2 TX
	DMA_SetCurrDataCounter(USART2_TX_DMA_CHANNEL, len); // Set DMA data length
	DMA_Cmd(USART2_TX_DMA_CHANNEL, ENABLE);				// Enable DMA for USART2 TX
}

/* USART2 Byte Array Transmission via DMA --------------------------------*/
void UART2_Send_Buffer_DMA(const uint8_t *buffer, uint16_t len)
{
	if (uart2_tx_busy)
		return; // Return if TX is busy
	if (len <= 0)
		return;

	uart2_tx_busy = 1; // Set TX busy flag
	if (len > USART2_TX_BUF_SIZE)
		len = USART2_TX_BUF_SIZE;			 // Limit length to buffer size
	memcpy(UART2_TxRingBuffer, buffer, len); // Copy data to TX ring buffer

	// Disable and reset DMA before transmission
	DMA_Cmd(USART2_TX_DMA_CHANNEL, DISABLE);			// Disable DMA for USART2 TX
	DMA_SetCurrDataCounter(USART2_TX_DMA_CHANNEL, len); // Set DMA data length
	DMA_Cmd(USART2_TX_DMA_CHANNEL, ENABLE);				// Enable DMA for USART2 TX
}
#endif

void UART2_SendByte(const uint8_t data)
{
	// Wait until transmit data register is empty
	uint32_t timeout = 0xFFFF;
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
	{
		if (--timeout == 0)
		{
			return;
		}
	}
	USART_SendData(USART2, data);
}

void UART2_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
	for (uint16_t i = 0; i < length; i++)
	{
		UART2_SendByte(buffer[i]);
	}
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
		;
}
#endif

/* UART3 String Transmission via DMA ------------------------------------*/
#ifdef HARDWARE_BSP_UART_3
#ifdef UART3_DMA_MODE
void UART3_Send_String_DMA(const char *str)
{
	if (uart3_tx_busy)
		return;					// Return if TX is busy
	uint16_t len = strlen(str); // Calculate string length
	if (len <= 0)
		return;

	uart3_tx_busy = 1; // Set TX busy flag
	if (len > UART3_TX_BUF_SIZE)
		len = UART3_TX_BUF_SIZE;					   // Limit length to buffer size
	memcpy(UART3_TxBuffer1, str, len);				   // Copy string to TX buffer
	DMA_Cmd(UART3_TX_DMA_CHANNEL, DISABLE);			   // Disable DMA for UART3 TX
	DMA_SetCurrDataCounter(UART3_TX_DMA_CHANNEL, len); // Set DMA data length
	DMA_Cmd(UART3_TX_DMA_CHANNEL, ENABLE);			   // Enable DMA for UART3 TX
}

/* UART3 Byte Array Transmission via DMA --------------------------------*/
void UART3_Send_Buffer_DMA(const uint8_t *buffer, uint16_t length)
{
	if (uart3_tx_busy)
		return; // Return if TX is busy
	if (length <= 0)
		return;
	uart3_tx_busy = 1; // Set TX busy flag

	if (length > UART3_TX_BUF_SIZE)
		length = UART3_TX_BUF_SIZE; // Limit length to buffer size

	memcpy(UART3_TxBuffer1, buffer, length); // Copy data to TX buffer
	DMA_Cmd(UART3_TX_DMA_CHANNEL, DISABLE);	 // Disable DMA for USART1 TX
	DMA_ClearFlag(UART3_TX_DMA_FLAG);
	DMA_SetCurrDataCounter(UART3_TX_DMA_CHANNEL, length); // Set DMA data length
	DMA_Cmd(UART3_TX_DMA_CHANNEL, ENABLE);				  // Enable DMA for USART1 TX
}
#endif
// Send one byte over UART3
void UART3_SendByte(const uint8_t data)
{
	uint32_t timeout = 0xFFFF;
	while (USART_GetFlagStatus(UART3, USART_FLAG_TXE) == RESET)
	{
		if (--timeout == 0)
		{
			return;
		}
	}
	USART_SendData(UART3, data);
}

// Send multiple bytes over UART3
void UART3_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
	for (uint16_t i = 0; i < length; i++)
	{
		UART3_SendByte(buffer[i]);
	}
}

// Receive one byte (blocking)
uint8_t UART3_ReceiveByte(void)
{
	// Wait until data is received
	while (USART_GetFlagStatus(UART3, USART_FLAG_RXNE) == RESET)
		;
	return (uint8_t)USART_ReceiveData(UART3);
}
#endif

#ifdef HARDWARE_BSP_UART_4
// Send one byte over UART3
void UART4_SendByte(const uint8_t data)
{
	// Wait until transmit data register is empty
	uint32_t timeout = 0xFFFF;
	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET)
	{
		if (--timeout == 0)
		{
			return;
		}
	}
	USART_SendData(UART4, data);
}

// Send multiple bytes over UART4
void UART4_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
	for (uint16_t i = 0; i < length; i++)
	{
		UART4_SendByte(buffer[i]);
	}
}

// Receive one byte (blocking) over UART4
uint8_t UART4_ReceiveByte(void)
{
	// Wait until data is received
	while (USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET)
		;
	return (uint8_t)USART_ReceiveData(UART4);
}
#endif

#ifdef HARDWARE_BSP_LUART_1
/**
 * @brief Send a buffer of data via LPUART.
 * @param data Pointer to the data buffer to send.
 * @param len Number of bytes to send.
 */

void LPUART_Send_Buffer(const uint8_t *buffer, size_t length)
{
	for (size_t i = 0; i < length; i++)
	{
		/* Wait until transmit data register is empty */
		while (LPUART_GetFlagStatus(LPUART, LPUART_FLAG_TXE) == RESET)
		{
			// Wait
		}

		/* Send the byte */
		LPUART_SendData(LPUART, buffer[i]);
	}

	/* After sending all bytes, wait for Transmission Complete flag (optional) */
	while (LPUART_GetFlagStatus(LPUART, LPUART_FLAG_TC) == RESET)
	{
		// Wait
	}
}
#endif
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#if 0
/**
  * @brief  ADC channel configuration
  * @retval None
  */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define ADC1_DR_Address 0x40012440
__IO uint16_t ADC1ConvertedValue = 0;
__IO uint16_t ADC1ConvertedVoltage = 0;
__IO uint16_t RegularConvData_Tab[3];

void ADC_DMA_Config(void)
{
	//////////////////////////////////////////////////////////////////////////////////
		DMA_InitTypeDef   DMA_InitStructure;
    /* DMA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA, ENABLE);

    /* DMA Channel1 Config */
    DMA_DeInit(DMA_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 3;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_Channel1, &DMA_InitStructure);

    /* DMA Channel1 enable */
    DMA_Cmd(DMA_Channel1, ENABLE);
		//////////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
    ADC_InitTypeDef     ADC_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure;

    /* ADC DeInit */
    ADC_DeInit(ADC);

    /* GPIOA and SYSCFG Periph clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Enable Vrefint ADC  */
    SYSCFG_ADCVrefEnableCmd(ENABLE);

    /* ADC Periph clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC, ENABLE);

    /* Configure ADC channel0 as analog input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Initialize ADC structure */
    ADC_StructInit(&ADC_InitStructure);

    /* Configure the ADC in continuous mode withe a resolution equal to 12 bits  */
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward;
    ADC_Init(ADC, &ADC_InitStructure);

    /* Convert the ADC Channel0 with 55.5 Cycles as sampling time */
    ADC_ChannelConfig(ADC, ADC_Channel_0, ADC_SampleTime_55_5Cycles);

    /* Convert the ADC temperature sensor  with 55.5 Cycles as sampling time */
    ADC_ChannelConfig(ADC, ADC_Channel_TempSensor, ADC_SampleTime_55_5Cycles);
    ADC_TempSensorCmd(ADC, ENABLE);

    /* Convert the ADC Vref  with 55.5 Cycles as sampling time */
    ADC_ChannelConfig(ADC, ADC_Channel_Vrefint, ADC_SampleTime_55_5Cycles);
    ADC_VrefintCmd(ADC, ENABLE);
    ADC_SetClock(ADC, ADC_ClockMode_SynClkDiv4);

    /* ADC Calibration */
    ADC_GetCalibrationFactor(ADC);

    /* ADC DMA request in circular mode */
    ADC_DMARequestModeConfig(ADC, ADC_DMAMode_Circular);

    /* Enable ADC_DMA */
    ADC_DMACmd(ADC, ENABLE);

    /* Enable the ADC peripheral */
    ADC_Cmd(ADC, ENABLE);

    /* Wait the ADRDY flag */
    while (!ADC_GetFlagStatus(ADC, ADC_FLAG_ADRDY))
    {
    }

    /* ADC regular Software Start Conv */
    ADC_StartOfConversion(ADC);
}
#endif

#ifdef HARDWARE_BSP_ADC_CHANNEL_8
void ADC_Channel_8_Config(void)
{
#if 1
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA Peripheral clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* ADC1 Peripheral clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC, ENABLE);

	/* Configure ADC Chanenl0 as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* ADCs DeInit */
	ADC_DeInit(ADC);

	/* Configure the ADC1 in continous mode withe a resolution equal to 12 bits*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_Init(ADC, &ADC_InitStructure);
	ADC_ChannelConfig(ADC, ADC_Channel_8, ADC_SampleTime_28_5Cycles);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC);

	/* Enable the ADC peripheral */
	ADC_Cmd(ADC, ENABLE);

	/* Wait the ADRDY flag */
	while (!ADC_GetFlagStatus(ADC, ADC_FLAG_ADRDY))
	{
	}

	/* ADC1 regular Software Start Conv */
	ADC_StartOfConversion(ADC);
#endif
}

uint16_t adc_get_value_v(void)
{
	uint32_t temp1, temp2;
	double v_10 = 0;

#if 1
	while (!ADC_GetFlagStatus(ADC, ADC_FLAG_EOC))
	{
	}

	temp1 = ADC_GetConversionValue(ADC);
	temp2 = (temp1 * 3300) / 0xFFF;

	v_10 = (temp2 * 34) / 100;
#endif

	return v_10;
}
#endif
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#ifdef HARDWARE_BSP_TIMER_2
void TIM2_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable clock for TIM2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/*
	 * Configure TIM2 to generate an interrupt every 50us.
	 * Timer frequency calculation:
	 * Timer Frequency = 48 MHz / (Prescaler + 1) / (Period + 1)
	 * For 50us interval => 1 / 50us = 20kHz
	 * Example: Prescaler = 47, Period = 49
	 *          => 48MHz / 48 / 50 = 20kHz (50us)
	 */
	TIM_TimeBaseStructure.TIM_Period = 49;	  // Auto-reload value
	TIM_TimeBaseStructure.TIM_Prescaler = 47; // Prescaler value
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	// Initialize TIM2 with the specified parameters
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// Enable TIM2 update interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	// Configure NVIC for TIM2 interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	// Set TIM2 interrupt to highest priority (0 = highest)
	NVIC_SetPriority(TIM2_IRQn, 0);

	// Enable TIM2 interrupt
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Start TIM2
	TIM_Cmd(TIM2, ENABLE);
}

void TIM2_IRQHandler(void)
{
	// Check if update interrupt flag is set
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		// Clear the interrupt pending bit
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

		// Increment the 50us tick
		system_timer_tick_50us++;

		// Optionally, you can add any periodic task logic here
		// Example callback for periodic tasks every 50us
		// hal_timer_interrupt_callback(0);
	}
	// TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}
#endif

#ifdef TASK_MANAGER_STATE_MACHINE_CAN
/////////////////////////////////////////////////////////////////////////////
void CAN_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	CAN_FilterInitTypeDef CAN_FilterInitStructure;

#if 0 // def HK_DEMO_BORAD_TEST
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Connect CAN pins to AF4 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_4);
	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Connect CAN pins to AF4 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_4);
	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* NVIC configuration *******************************************************/
	NVIC_InitStructure.NVIC_IRQChannel = LCD_CAN_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* CAN configuration ********************************************************/
	/* CAN register init */
	CAN_DeInit();
	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;

#if 0
	/* CAN Baudrate = 1MBps (CAN clocked at 48 MHz) */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
	CAN_InitStructure.CAN_Prescaler = 3;
#else
	/* CAN Baudrate = 500bps (CAN clocked at 48 MHz) */
	CAN_InitStructure.CAN_BS1 = CAN_BS1_10tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
	CAN_InitStructure.CAN_Prescaler = 6; // 500bps
	// CAN_InitStructure.CAN_Prescaler = 12;//250bps
#endif
	CAN_Init(&CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* Enable FIFO 0 full Interrupt */
	CAN_ITConfig(CAN_IT_FF0, ENABLE);

	/* Enable FIFO 1 full Interrupt */
	CAN_ITConfig(CAN_IT_FF1, ENABLE);

	// uint8_t TxMessages[8] = { 0, 1, 2, 3, 4, 5, 6, 7};
	// CAN_Send_Data(0,CAN_ID_STD,TxMessages,8);
}

/**
 * @brief  This function handles CAN request.
 * @retval None
 */
void LCD_CAN_IRQHandler(void)
{
	CAN_Receive(CAN_FIFO0, &CanRxMessage);

	if (system_get_mcu_status() == MB_POWER_ST_ON)
	{
		// LOG_BUFF_LEVEL((const uint8_t *)&CanRxMessage, sizeof(CanRxMsg));
		// LED_Display(CanRxMessage.Data[0]);
		// KeyNumber = CanRxMessage.Data[0];
		CAN_Message_t CAN_Message; // = *(CAN_Message_t *)&CanRxMessage;
		CAN_Message.StdId = CanRxMessage.StdId;
		CAN_Message.ExtId = CanRxMessage.ExtId;
		CAN_Message.IDE = CanRxMessage.IDE;
		CAN_Message.RTR = CanRxMessage.RTR;
		CAN_Message.DLC = CanRxMessage.DLC;
		CAN_Message.FMI = CanRxMessage.FMI;
		for (uint8_t i = 0; i < CAN_Message.DLC; i++)
		{
			CAN_Message.Data[i] = CanRxMessage.Data[i];
		}
		can_message_receiver(&CAN_Message);
	}
}

/**
 * @brief  Send a CAN standard data frame
 * @param  std_id: Standard Identifier (11-bit, 0x000 ~ 0x7FF)// CAN ID (标准 11bit 或扩展 29bit)
 * @param  uint8_t ide,		// 0 = 标准ID, 1 = 扩展ID
 * @param  data: Pointer to the data buffer (max 8 bytes)
 * @param  length: Length of the data (0~8)
 * @retval 1 if success, 0 if failed (e.g., no mailbox available)
 */
CAN_Status_t CAN_Send_Data(uint32_t id, uint8_t ide, const uint8_t *buffer, uint8_t length)
{
	if (!buffer || length == 0 || length > CAN_DATA_MAX_LENGTH)
	{
		return CAN_ERR_PARAM;
	}

	// CanTxMsg_t CanTxMessage;
	//  根据 IDE 设置标准/扩展 ID
	if (ide == CAN_ID_STD)
	{
		CanTxMessage.StdId = id & 0x7FF; // 标准ID 11bit
		CanTxMessage.ExtId = 0;
		CanTxMessage.IDE = CAN_ID_STD; // 标准帧
	}
	else
	{
		CanTxMessage.StdId = 0;
		CanTxMessage.ExtId = id & 0x1FFFFFFF; // 扩展ID 29bit
		CanTxMessage.IDE = CAN_ID_EXT;		  // 扩展帧
	}

	CanTxMessage.RTR = CAN_RTR_DATA; // 数据帧
	CanTxMessage.DLC = length;

	// 拷贝数据
	for (uint8_t i = 0; i < length; ++i)
	{
		CanTxMessage.Data[i] = buffer[i];
	}

	// 发送
	uint8_t mailbox = CAN_Transmit(&CanTxMessage);
	if (mailbox >= CAN_TxStatus_NoMailBox)
	{ // 假设只有三个邮箱 0~2
		return CAN_ERR_TRANSMIT;
	}

	// 等待发送完成
	uint32_t timeout = 0xFFFF;
	while ((CAN_TransmitStatus(mailbox) != 1) && (timeout--)) // 1 = OK
		;

	if (timeout == 0)
		return CAN_ERR_TIMEOUT;

	LOG_LEVEL("CAN TX ID:%04X IDE:%04X DATA:", id, ide);
	LOG_BUFF(buffer, length);
	return CAN_OK;
}
/////////////////////////////////////////////////////////////////////////////
// TIM3 PWM Backlight Initialization
// pwm_freq: Target PWM frequency in Hz (e.g., 1000 for 1kHz)
// duty_cycle: Initial duty cycle in percentage (0~100)
#if 0
void TIM3_PWM_Backlight_Init(uint16_t pwm_freq, uint8_t duty_cycle)
{
	// Ensure duty cycle is in 0~100 range
	if (duty_cycle > 100)
		duty_cycle = 100;

	// Enable clocks for GPIO and TIM3
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE); // or GPIOB/C depending on your pin
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Configure GPIO for TIM3 CHx (Assume PA6 as TIM3_CH1, change to match your board)
	GPIO_InitTypeDef GPIO_InitStructure;
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Connect TIM3_CH2 to PB1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2); // AF2 for TIM3 (CH2)

	// Compute TIM3 Period and Prescaler
	uint32_t timer_clock = 48000000;				  // System clock = 48MHz
	uint16_t prescaler = (timer_clock / 1000000) - 1; // Prescaler to 1MHz timer clock
	uint16_t period = (1000000 / pwm_freq) - 1;		  // 1MHz / pwm_freq = period

	// Configure TIM3 base
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = period;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// Configure PWM mode on TIM3 Channel 1
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = (period + 1) * duty_cycle / 100; // Duty cycle
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	// Enable auto-reload preload and start timer
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}
#endif
// Function to change PWM duty cycle dynamically
void Set_Backlight_Brightness(uint8_t duty_cycle)
{
	// Make sure duty cycle is within 0-100 range
	if (duty_cycle > 100)
		duty_cycle = 100;

	// Update TIM3 PWM pulse width according to new duty cycle
	uint16_t period = TIM3->ARR;				  // Get current period
	TIM3->CCR1 = (period + 1) * duty_cycle / 100; // Set new pulse width (duty cycle)
}
#endif

/////////////////////////////////////////////////////////////////////////////
void PTL_1_UART_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
#ifdef UART3_DMA_MODE
	UART3_Send_Buffer_DMA(data, len);
#else
	UART3_Send_Buffer(buffer, length);
#endif
}
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
#ifdef TASK_MANAGER_STATE_MACHINE_I2C
#define BSP_I2C1 I2C1
#define BSP_I2C1_CLK RCC_APB1Periph_I2C1

#define BSP_I2C1_SCL_PIN GPIO_Pin_6	 /* PB.06 */
#define BSP_I2C1_SCL_GPIO_PORT GPIOB /* GPIOB */
#define BSP_I2C1_SCL_GPIO_CLK RCC_AHBPeriph_GPIOB
#define BSP_I2C1_SCL_SOURCE GPIO_PinSource6
#define BSP_I2C1_SCL_AF GPIO_AF_1

#define BSP_I2C1_SDA_PIN GPIO_Pin_7	 /* PB.07 */
#define BSP_I2C1_SDA_GPIO_PORT GPIOB /* GPIOB */
#define BSP_I2C1_SDA_GPIO_CLK RCC_AHBPeriph_GPIOB
#define BSP_I2C1_SDA_SOURCE GPIO_PinSource7
#define BSP_I2C1_SDA_AF GPIO_AF_1

#define BSP_I2C1_SMBUSALERT_PIN GPIO_Pin_5	/* PB.05 */
#define BSP_I2C1_SMBUSALERT_GPIO_PORT GPIOB /* GPIOB */
#define BSP_I2C1_SMBUSALERT_GPIO_CLK RCC_AHBPeriph_GPIOB
#define BSP_I2C1_SMBUSALERT_SOURCE GPIO_PinSource5
#define BSP_I2C1_SMBUSALERT_AF GPIO_AF_3

#define BSP_I2C1_TIMING 0x1045061D
#define BSP_I2C1_FLAG_TIMEOUT ((uint32_t)0x1000)
#define BSP_I2C1_LONG_TIMEOUT ((uint32_t)(10 * BSP_I2C1_FLAG_TIMEOUT))

void BSP_IIC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* LM75_I2C Periph clock enable */
	RCC_APB1PeriphClockCmd(BSP_I2C1_CLK, ENABLE);

	/* Configure the I2C clock source. The clock is derived from the HSI */
	RCC_I2C1CLKConfig(RCC_I2C1CLK_HSI8M);
	/* LM75_I2C_SCL_GPIO_CLK, LM75_I2C_SDA_GPIO_CLK
		 and LM75_I2C_SMBUSALERT_GPIO_CLK Periph clock enable */
	RCC_AHBPeriphClockCmd(BSP_I2C1_SCL_GPIO_CLK | BSP_I2C1_SDA_GPIO_CLK |
							  BSP_I2C1_SMBUSALERT_GPIO_CLK,
						  ENABLE);

	/* Connect PXx to I2C_SCL */
	GPIO_PinAFConfig(BSP_I2C1_SCL_GPIO_PORT, BSP_I2C1_SCL_SOURCE, BSP_I2C1_SCL_AF);

	/* Connect PXx to I2C_SDA */
	GPIO_PinAFConfig(BSP_I2C1_SDA_GPIO_PORT, BSP_I2C1_SDA_SOURCE, BSP_I2C1_SDA_AF);

	/* Connect PXx to I2C_SMBUSALER */
	GPIO_PinAFConfig(BSP_I2C1_SMBUSALERT_GPIO_PORT, BSP_I2C1_SMBUSALERT_SOURCE, BSP_I2C1_SMBUSALERT_AF);

	/* Configure LM75_I2C pins: SCL */
	GPIO_InitStructure.GPIO_Pin = BSP_I2C1_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BSP_I2C1_SCL_GPIO_PORT, &GPIO_InitStructure);

	/* Configure LM75_I2C pins: SDA */
	GPIO_InitStructure.GPIO_Pin = BSP_I2C1_SDA_PIN;
	GPIO_Init(BSP_I2C1_SDA_GPIO_PORT, &GPIO_InitStructure);

	/* Configure LM75_I2C pin: SMBUS ALERT */
	GPIO_InitStructure.GPIO_Pin = BSP_I2C1_SMBUSALERT_PIN;
	GPIO_Init(BSP_I2C1_SMBUSALERT_GPIO_PORT, &GPIO_InitStructure);
}

void BSP_IIC1_Config(void)
{
	I2C_InitTypeDef I2C_InitStructure;
	BSP_IIC1_GPIO_Config();

	/* LM75_I2C configuration */
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; // I2C_Mode_SMBusHost;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_Timing = BSP_I2C1_TIMING;

	/* Apply LM75_I2C configuration after enabling it */
	I2C_Init(BSP_I2C1, &I2C_InitStructure);

	/* LM75_I2C Peripheral Enable */
	I2C_Cmd(BSP_I2C1, ENABLE);
}

/******************************************************************************/
/*                 HK32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (KEIL_Startup_hk32l0xx.s).                                               */
/******************************************************************************/
/**
 * @brief  This function handles I2C1 Error interrupt request.
 * @param  None
 * @retval None
 */
void I2C1_IRQHandler(void)
{
	/* Check on I2C1 SMBALERT flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_ALERT))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_ALERT);
		// SMbusAlertOccurred++;
	}

	/* Check on I2C1 Time out flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_TIMEOUT))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_TIMEOUT);
	}

	/* Check on I2C1 Arbitration Lost flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_ARLO))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_ARLO);
	}

	/* Check on I2C1 PEC error flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_PECERR))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_PECERR);
	}

	/* Check on I2C1 Overrun/Underrun error flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_OVR))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_OVR);
	}

	/* Check on I2C1 Acknowledge failure error flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_NACKF))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_NACKF);
	}

	/* Check on I2C1 Bus error flag and clear it */
	if (I2C_GetITStatus(I2C1, I2C_IT_BERR))
	{
		I2C_ClearITPendingBit(I2C1, I2C_IT_BERR);
	}
}

uint8_t hal_iic1_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	uint8_t DataNum = 0;
	__IO uint32_t LM75Timeout = BSP_I2C1_LONG_TIMEOUT;
	/* Test on BUSY Flag */
	LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

	while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_BUSY) != RESET)
	{
		if ((LM75Timeout--) == 0)
		{
			return 1;
		}
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(BSP_I2C1, dev_address, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

	while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_TXIS) == RESET)
	{
		if ((LM75Timeout--) == 0)
		{
			return 2;
		}
	}

	/* Send Register address */
	I2C_SendData(BSP_I2C1, (uint8_t)reg_address);

	/* Wait until TCR flag is set */
	LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

	while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_TCR) == RESET)
	{
		if ((LM75Timeout--) == 0)
		{
			return 3;
		}
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(BSP_I2C1, dev_address, 2, I2C_AutoEnd_Mode, I2C_No_StartStop);

	while (DataNum != length)
	{
		/* Wait until TXIS flag is set */
		LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

		while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_TXIS) == RESET)
		{
			if ((LM75Timeout--) == 0)
			{
				return 4;
			}
		}

		/* Write data to TXDR */
		I2C_SendData(BSP_I2C1, (uint8_t)(buffer[DataNum]));

		/* Update number of transmitted data */
		DataNum++;
	}

	/* Wait until STOPF flag is set */
	LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

	while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_STOPF) == RESET)
	{
		if ((LM75Timeout--) == 0)
		{
			return 5;
		}
	}

	/* Clear STOPF flag */
	I2C_ClearFlag(BSP_I2C1, I2C_ICR_STOPCF);
	return 0;
}
uint8_t hal_iic2_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}
uint8_t hal_iic3_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}

uint8_t hal_iic4_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}

uint8_t hal_iic5_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}

uint8_t hal_iic6_write(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}

uint8_t hal_iic1_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	__IO uint32_t LM75Timeout = BSP_I2C1_LONG_TIMEOUT;
	// uint8_t LM75_BufferRX[2] = {0, 0};
	// uint16_t tmp = 0;
	uint32_t DataNum = 0;

	/* Test on BUSY Flag */
	LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

	while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_BUSY) != RESET)
	{
		if ((LM75Timeout--) == 0)
		{
			return 1;
		}
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(BSP_I2C1, dev_address, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

	/* Wait until TXIS flag is set */
	LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

	while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_TXIS) == RESET)
	{
		if ((LM75Timeout--) == 0)
		{
			return 2;
		}
	}

	/* Send Register address */
	I2C_SendData(BSP_I2C1, (uint8_t)reg_address);

	/* Wait until TC flag is set */
	LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

	while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_TC) == RESET)
	{
		if ((LM75Timeout--) == 0)
		{
			return 3;
		}
	}

	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(BSP_I2C1, dev_address, 2, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	/* Reset local variable */
	DataNum = 0;

	/* Wait until all data are received */
	while (DataNum != length)
	{
		/* Wait until RXNE flag is set */
		LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

		while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_RXNE) == RESET)
		{
			if ((LM75Timeout--) == 0)
			{
				return 4;
			}
		}

		/* Read data from RXDR */
		buffer[DataNum] = I2C_ReceiveData(BSP_I2C1);

		/* Update number of received data */
		DataNum++;
	}

	/* Wait until STOPF flag is set */
	LM75Timeout = BSP_I2C1_LONG_TIMEOUT;

	while (I2C_GetFlagStatus(BSP_I2C1, I2C_ISR_STOPF) == RESET)
	{
		if ((LM75Timeout--) == 0)
		{
			return 5;
		}
	}

	/* Clear STOPF flag */
	I2C_ClearFlag(BSP_I2C1, I2C_ICR_STOPCF);

	return length;
}

uint8_t hal_iic2_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}

uint8_t hal_iic3_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}

uint8_t hal_iic4_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}

uint8_t hal_iic5_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}

uint8_t hal_iic6_read(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer, uint8_t length)
{
	return 0;
}
#endif
#endif
