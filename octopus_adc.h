#ifndef __OCTOPUS_TASK_MANAGER_ADC_H__
#define __OCTOPUS_TASK_MANAGER_ADC_H__

/**************************************************************************************
 * @file    octopus_adc.h
 * @brief   ADC Key handling module for Octopus project.
 *          Supports multi-channel ADC sampling, DMA circular buffer,
 *          average filtering, key mapping, debounce, and key learning.
 * @version  1.0
 * @date     2024-12-12
 * @author   Octopus Team
 * @note    Designed for N32L4xx series MCU with official library style.
 **************************************************************************************/

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/
#include "octopus_base.h" // Base include file for the Octopus project
#include "octopus_key.h"
/**************************************************************************************
 * MACROS
 **************************************************************************************/

// #define KEY_DEBOUNCE_MS 20  /**< Debounce count threshold */

/**************************************************************************************
 * TYPE DEFINITIONS
 **************************************************************************************/
/**
 * @enum OctopusAdcChannel_t
 * @brief Enumeration of ADC channels used in the Octopus project.
 *
 * This enum defines up to 20 logical ADC channels. Each channel can be
 * mapped to a physical ADC input pin or a functional purpose (e.g. keys,
 * sensors, battery voltage).
 */
typedef enum
{
    OCT_ADC_CHANNEL_0 = 0, /**< ADC Channel 0  - Reserved / General purpose */
    OCT_ADC_CHANNEL_1,     /**< ADC Channel 1  - Reserved / General purpose */
    OCT_ADC_CHANNEL_2,     /**< ADC Channel 2  - Reserved / General purpose */
    OCT_ADC_CHANNEL_3,     /**< ADC Channel 3  - Reserved / General purpose */
    OCT_ADC_CHANNEL_4,     /**< ADC Channel 4  - Reserved / General purpose */
    OCT_ADC_CHANNEL_5,     /**< ADC Channel 5  - Reserved / General purpose */
    OCT_ADC_CHANNEL_6,     /**< ADC Channel 6  - Reserved / General purpose */
    OCT_ADC_CHANNEL_7,     /**< ADC Channel 7  - Reserved / General purpose */
    OCT_ADC_CHANNEL_8,     /**< ADC Channel 8  - Reserved / General purpose */
    OCT_ADC_CHANNEL_9,     /**< ADC Channel 9  - Reserved / General purpose */
    OCT_ADC_CHANNEL_10,    /**< ADC Channel 10 - Reserved / General purpose */
    OCT_ADC_CHANNEL_11,    /**< ADC Channel 11 - Reserved / General purpose */
    OCT_ADC_CHANNEL_12,    /**< ADC Channel 12 - Reserved / General purpose */
    OCT_ADC_CHANNEL_13,    /**< ADC Channel 13 - Reserved / General purpose */
    OCT_ADC_CHANNEL_14,    /**< ADC Channel 14 - Reserved / General purpose */
    OCT_ADC_CHANNEL_15,    /**< ADC Channel 15 - Reserved / General purpose */
    OCT_ADC_CHANNEL_16,    /**< ADC Channel 16 - Reserved / General purpose */
    OCT_ADC_CHANNEL_17,    /**< ADC Channel 17 - Reserved / General purpose */
    OCT_ADC_CHANNEL_18,    /**< ADC Channel 18 - Reserved / General purpose */
    OCT_ADC_CHANNEL_19,    /**< ADC Channel 19 - Reserved / General purpose */

    OCT_ADC_CHANNEL_MAX /**< Total number of ADC channels */
} adc_channel_t;

/**
 * @brief Key mapping structure
 *        Defines ADC voltage range corresponding to a specific key ID.
 */
typedef struct
{
    key_state_t state;
    bool pressed;    ///< Whether the key is currently pressed.
    bool release;    ///< Whether the key has been released.
    bool dispatched; ///< Whether the key event has been dispatched.
    bool ignore;     ///< Ignore  the key until press again

    uint32_t start_tick_count;
    uint32_t press_duration; ///< Duration for long-press detection.
} KEY_STATUS;

typedef struct
{
    adc_channel_t channel;
    uint16_t value; /**< Maximum ADC value for the key */
    uint8_t key_s;  /**< Assigned key ID */
    uint8_t key_l;
    KEY_STATUS status;
} ADC_KEY_STATUS;

typedef struct
{
    adc_channel_t channel;
    uint16_t value; /**< Maximum ADC value for the key */
    uint8_t key_s;  /**< Assigned key ID */
    uint8_t key_l;
} ADC_KEY_LEARN_STATUS;

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

/**************************************************************************************
 * 配置参数
 **************************************************************************************/
// ADC通道号数组，PA0->ADC_CH0, PA1->ADC_CH1...
extern uint8_t adc_channels[];
// DMA循环缓冲区，每通道采样 ADC_SAMPLE_COUNT 次
extern uint16_t adc_channel_buffer[];
// 平均值结果，每个通道一个值
extern uint16_t adc_channel_value[];
extern ADC_KEY_STATUS *adc_key_array[]; /**< ADC key channel structures */

/**************************************************************************************
 * FUNCTION PROTOTYPES
 **************************************************************************************/
void task_adc_init_running(void);
void task_adc_start_running(void);
void task_adc_assert_running(void);
void task_adc_running(void);
void task_adc_post_running(void);
void task_adc_stop_running(void);

#endif // __OCTOPUS_TASK_MANAGER_ADC_H__
