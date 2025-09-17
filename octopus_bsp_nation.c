
/* Includes ------------------------------------------------------------------*/
#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "octopus_bsp_nation.h"
#include "octopus_adc.h"

volatile uint32_t system_tick_counter_ms;
// ADC通道号数组，PA0->ADC_CH0, PA1->ADC_CH1...
uint8_t adc_channels[ADC_CHANNEL_NUM] = {ADC_CH_1, ADC_CH_2, ADC_CH_3};

// DMA循环缓冲区，每通道采样 ADC_SAMPLE_COUNT 次
uint16_t adc_channel_buffer[ADC_CHANNEL_NUM * ADC_SAMPLE_COUNT];

#ifdef PLATFORM_NATION_RTOS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief  This function handles SysTick Handler.
 */
void SysTick_Handler(void)
{
    system_tick_counter_ms++;
}

///////BT/debug
__weak void UART1_RX_Callback(uint8_t *buffer, uint16_t length) ///////BT
{
#ifdef TASK_MANAGER_STATE_MACHINE_BT_MUSIC
    ptl_2_receive_callback(PTL2_MODULE_BT, buffer, length);
#endif
}

// Weak callback function for USART2 RX  ///////BLE-MCU
__weak void UART2_RX_Callback(uint8_t *buffer, uint16_t length)
{
    // UART2_Send_Buffer(buffer,length);
}

// Weak callback function for UART3 RX
///////SOC
__weak void UART3_RX_Callback(uint8_t *buffer, uint16_t length)
{
    // LOG_LEVEL("UART3_RX_Callback bytes:%d ",length);
    // UART3_Send_Buffer(buffer,length);
}

///////4G/GPS
__weak void UART4_RX_Callback(uint8_t *buffer, uint16_t length)
{
    // UART4_Send_Buffer(buffer,length);
}

// Weak callback function for LPUART RX
///////bafang/the third protocol
__weak void LPUART_RX_Callback(uint8_t *buffer, uint16_t length)
{
    /// LPUART_Send_Buffer(buffer,length);
    /// UART1_Send_Buffer(buffer,length);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialize DWT cycle counter for microsecond resolution
void platform_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

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
void platform_delay_us__(uint32_t us)
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SYS_Config(void)
{
    // SysTick_Config() takes the reload value as parameter.
    // The reload value = (System clock frequency / desired interrupt frequency) - 1
    // For 1ms tick: reload = (sysclk / 1000) - 1
    if (SysTick_Config(SystemCoreClock / 1000))
    {
        // If return value is non-zero, reload value was invalid (> 0xFFFFFF)
        while (1)
            ; // Stay here to indicate error
    }
}

void RCC_Config(void)
{
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOA, ENABLE);
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOB, ENABLE);
    /* Enable GPIOC clocks */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_GPIOC, ENABLE);

    /* Enable USARTx Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE);
    /* Enable USARTx Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB1_PERIPH_USART2, ENABLE);
    /* Enable USARTx Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB1_PERIPH_USART3, ENABLE);
    /* Enable USARTx Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_UART4, ENABLE);
    /* Enable USARTx Clock */
    RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_UART5, ENABLE);

    /* Enable DMA clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA, ENABLE);

    /* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);

    /* RCC_ADCHCLK_DIV16*/
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV16);
    RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSE, RCC_ADC1MCLK_DIV8); // selsect HSE as RCC ADC1M CLK Source
}

void GPIO_Config(void)
{

    GPIO_InitType GPIO_InitStructure;

    // ---------- USART1: PA9=TX, PA10=RX ----------
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate function push-pull
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_Up; // Pull-up for RX
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART1;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    // ---------- USART2: PA2=TX, PA3=RX ----------
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART2;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_Up;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART2;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    // ---------- USART3: PB10=TX, PB11=RX ----------
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_Up;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF4_USART3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    // ---------- UART4: PC10=TX, PC11=RX ----------
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_UART4;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_Up;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_UART4;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);

    // ---------- UART5: PC12=TX, PD2=RX ----------
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_UART5;
    GPIO_InitPeripheral(GPIOC, &GPIO_InitStructure);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_Up;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_UART5;
    GPIO_InitPeripheral(GPIOD, &GPIO_InitStructure);

    // ---------- LPUART1: PB6=TX, PB7=RX ----------
    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_LPUART;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);

    GPIO_InitStruct(&GPIO_InitStructure);
    GPIO_InitStructure.Pin = GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_Up;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF6_LPUART;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UART1_Config_IRQ(void)
{
    USART_InitType USART_InitStructure;

    // Initialize structure with default values
    USART_StructInit(&USART_InitStructure);

    // Set UART parameters
    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

    // Initialize USART1
    USART_Init(USART1, &USART_InitStructure);

    // Enable RX interrupt
    USART_ConfigInt(USART1, USART_INT_RXDNE, ENABLE);

    // Enable NVIC interrupt for USART1
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 1);

    // Enable USART1
    USART_Enable(USART1, ENABLE);
}

// ---------------- UART2 RX Interrupt Configuration ----------------
void UART2_Config_IRQ(void)
{
    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

    USART_Init(USART2, &USART_InitStructure);

    USART_ConfigInt(USART2, USART_INT_RXDNE, ENABLE);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 1);

    USART_Enable(USART2, ENABLE);
}

// ---------------- UART3 RX Interrupt Configuration ----------------
void UART3_Config_IRQ(void)
{
    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

    USART_Init(USART3, &USART_InitStructure);

    USART_ConfigInt(USART3, USART_INT_RXDNE, ENABLE);
    NVIC_EnableIRQ(USART3_IRQn);
    NVIC_SetPriority(USART3_IRQn, 1);

    USART_Enable(USART3, ENABLE);
}

// ---------------- UART4 RX Interrupt Configuration ----------------
void UART4_Config_IRQ(void)
{
    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

    USART_Init(UART4, &USART_InitStructure);

    USART_ConfigInt(UART4, USART_INT_RXDNE, ENABLE);
    NVIC_EnableIRQ(UART4_IRQn);
    NVIC_SetPriority(UART4_IRQn, 1);

    USART_Enable(UART4, ENABLE);
}

// ---------------- UART5 RX Interrupt Configuration ----------------
void UART5_Config_IRQ(void)
{
    USART_InitType USART_InitStructure;
    USART_StructInit(&USART_InitStructure);

    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

    USART_Init(UART5, &USART_InitStructure);

    USART_ConfigInt(UART5, USART_INT_RXDNE, ENABLE);
    NVIC_EnableIRQ(UART5_IRQn);
    NVIC_SetPriority(UART5_IRQn, 1);

    USART_Enable(UART5, ENABLE);
}

// ---------------- LPUART1 RX Interrupt Configuration ----------------
void LPUART_Config(void)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void USART1_IRQHandler(void)
{
    // Check if RX data register not empty interrupt occurred
    if (USART_GetIntStatus(USART1, USART_INT_RXDNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART1); // Read data

        // Clear RX interrupt flag (optional, safe)
        USART_ClrIntPendingBit(USART1, USART_INT_RXDNE);
    }

    if (USART_GetIntStatus(USART1, USART_INT_OREF) != RESET)
    {
        /*Read the STS register first,and the read the DAT
        register to clear the overflow interrupt*/
        (void)USART1->STS;
        (void)USART1->DAT;
    }
}

// ---------------- UART2 IRQ Handler ----------------
void USART2_IRQHandler(void)
{
    if (USART_GetIntStatus(USART2, USART_INT_RXDNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART2);

        USART_ClrIntPendingBit(USART2, USART_INT_RXDNE);
    }

    if (USART_GetIntStatus(USART2, USART_INT_OREF) != RESET)
    {
        /*Read the STS register first,and the read the DAT
        register to clear the overflow interrupt*/
        (void)USART2->STS;
        (void)USART2->DAT;
    }
}

// ---------------- UART3 IRQ Handler ----------------
void USART3_IRQHandler(void)
{
    if (USART_GetIntStatus(USART3, USART_INT_RXDNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART3);

        USART_ClrIntPendingBit(USART3, USART_INT_RXDNE);
    }

    if (USART_GetIntStatus(USART3, USART_INT_OREF) != RESET)
    {
        /*Read the STS register first,and the read the DAT
        register to clear the overflow interrupt*/
        (void)USART3->STS;
        (void)USART3->DAT;
    }
}

// ---------------- UART4 IRQ Handler ----------------
void UART4_IRQHandler(void)
{
    if (USART_GetIntStatus(UART4, USART_INT_RXDNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(UART4);

        USART_ClrIntPendingBit(UART4, USART_INT_RXDNE);
    }

    if (USART_GetIntStatus(UART4, USART_INT_OREF) != RESET)
    {
        /*Read the STS register first,and the read the DAT
        register to clear the overflow interrupt*/
        (void)UART4->STS;
        (void)UART4->DAT;
    }
}

// ---------------- UART5 IRQ Handler ----------------
void UART5_IRQHandler(void)
{
    if (USART_GetIntStatus(UART5, USART_INT_RXDNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(UART5);

        USART_ClrIntPendingBit(UART5, USART_INT_RXDNE);
    }

    if (USART_GetIntStatus(UART5, USART_INT_OREF) != RESET)
    {
        /*Read the STS register first,and the read the DAT
        register to clear the overflow interrupt*/
        (void)UART5->STS;
        (void)UART5->DAT;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ---------------------------------- UART1 ----------------------------------
void UART1_SendByte(uint8_t data)
{
    // Wait until transmit data register empty
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXDE) == RESET)
        ;
    USART_SendData(USART1, data);
}

void UART1_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        UART1_SendByte(buffer[i]);
    }
}

// ---------------------------------- UART2 ----------------------------------
void UART2_SendByte(uint8_t data)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXDE) == RESET)
        ;
    USART_SendData(USART2, data);
}

void UART2_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        UART2_SendByte(buffer[i]);
    }
}

// ---------------------------------- UART3 ----------------------------------
void UART3_SendByte(uint8_t data)
{
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXDE) == RESET)
        ;
    USART_SendData(USART3, data);
}

void UART3_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        UART3_SendByte(buffer[i]);
    }
}

// ---------------------------------- UART4 ----------------------------------
void UART4_SendByte(uint8_t data)
{
    while (USART_GetFlagStatus(UART4, USART_FLAG_TXDE) == RESET)
        ;
    USART_SendData(UART4, data);
}

void UART4_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        UART4_SendByte(buffer[i]);
    }
}

// ---------------------------------- UART5 ----------------------------------
void UART5_SendByte(uint8_t data)
{
    while (USART_GetFlagStatus(UART5, USART_FLAG_TXDE) == RESET)
        ;
    USART_SendData(UART5, data);
}

void UART5_Send_Buffer(const uint8_t *buffer, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        UART5_SendByte(buffer[i]);
    }
}

/**********************************************
 * ADC DMA 初始化函数
 **********************************************/
void ADC_DMA_Config(void)
{
    ADC_InitType ADC_InitStructure;
    DMA_InitType DMA_InitStructure;
    //GPIO_InitType GPIO_InitStructure;

    /* 3. 配置 DMA */
    DMA_DeInit(DMA_CH1);
    DMA_InitStructure.PeriphAddr = (uint32_t)&ADC->DAT;
    DMA_InitStructure.MemAddr = (uint32_t)adc_channel_buffer;
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize = ADC_CHANNEL_NUM * ADC_SAMPLE_COUNT;
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc = DMA_MEM_INC_ENABLE;

    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_HALFWORD;
    DMA_InitStructure.MemDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.CircularMode = DMA_MODE_CIRCULAR;
    DMA_InitStructure.Priority = DMA_PRIORITY_HIGH;
    DMA_InitStructure.Mem2Mem = DMA_M2M_DISABLE;
    DMA_Init(DMA_CH1, &DMA_InitStructure);
    DMA_RequestRemap(DMA_REMAP_ADC1, DMA, DMA_CH1, ENABLE);
    DMA_EnableChannel(DMA_CH1, ENABLE);
    /* Enable DMA Transfer Complete and Half Transfer interrupts */
    DMA_ConfigInt(DMA_CH1, DMA_INT_TXC, ENABLE); /* TXC = Transfer Complete (platform macro) */
    //DMA_ConfigInt(DMA_CH1, DMA_INT_HTC, ENABLE); /* HTC = Half Transfer (optional) */
    /* 4. 配置 ADC */
    ADC_DeInit(ADC);
    ADC_InitStructure.MultiChEn = ENABLE;                    // 扫描多通道
    ADC_InitStructure.ContinueConvEn = ENABLE;               // 连续转换
    ADC_InitStructure.ExtTrigSelect = ADC_EXT_TRIGCONV_NONE; // 软件触发
    ADC_InitStructure.DatAlign = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber = ADC_CHANNEL_NUM;
    ADC_Init(ADC, &ADC_InitStructure);

    /* 5. 配置每个通道顺序 */
    for (int i = 0; i < ADC_CHANNEL_NUM; i++)
    {
        ADC_ConfigRegularChannel(ADC, adc_channels[i], i + 1, ADC_SAMP_TIME_55CYCLES5);
    }

    /* 6. 开启 DMA 请求 */
    ADC_EnableDMA(ADC, ENABLE);
    /* 7. 使能 ADC 并校准 */
    ADC_Enable(ADC, ENABLE);
    /* Check ADC Ready */
    while (ADC_GetFlagStatusNew(ADC, ADC_FLAG_RDY) == RESET)
        ;
    /* Start ADC1 calibration */
    ADC_StartCalibration(ADC);
    /* Check the end of ADC1 calibration */
    while (ADC_GetCalibrationStatus(ADC))
        ;

    /* Start ADC1 Software Conversion */
    ADC_EnableSoftwareStartConv(ADC, ENABLE);
}

static void ADC_ComputeFiltered(void)
{
    for (int ch = 0; ch < ADC_CHANNEL_NUM; ch++)
    {
        uint32_t sum = 0;
        for (int i = 0; i < ADC_SAMPLE_COUNT; i++)
        {
            sum += adc_channel_buffer[ch + i * ADC_CHANNEL_NUM];
        }
        adc_channel_value[ch] = sum / ADC_SAMPLE_COUNT;
    }
}
/****************************************************************************
 * @brief DMA1 Channel1 IRQ handler
 *        - On Transfer Complete (full buffer filled): process full buffer safely.
 *        - On Half Transfer: optional early processing of first half.
 *
 * Note: Keep ISR short. If heavy processing required, set flag and process in task.
 ****************************************************************************/
void DMA_Channel1_IRQHandler(void)
{
    /* Transfer Complete */
    //if (DMA_GetFlagStatus(DMA_FLAG_TC1, DMA))
    {
			  ADC_ComputeFiltered();
        DMA_ClearFlag(DMA_INT_TXC,DMA);
        /* Option A: do averaging inside ISR (fast enough) */
        //ADC_CalcAverage_ISR();
			  DMA_ClrIntPendingBit(DMA_INT_GLB1, DMA);
    }
}

#endif
