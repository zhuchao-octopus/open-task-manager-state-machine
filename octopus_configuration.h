/********************************************************************************
 * @file    octopus_configuration.h
 * @author  Octopus Team
 * @date    2025-09-01
 * @version v1.0
 *
 * @brief
 *   Project configuration header file.
 *   This file contains all compile-time switches for platform selection,
 *   RTOS type, Task Manager modules, and feature toggles. It serves as the
 *   central place to control project build options.
 *
 * @details
 *   - Platform/RTOS selection: Choose one platform macro only.
 *   - Task Manager: Enable or disable state machine modules independently.
 *   - Flash options: Configure how flash memory is managed and mapped.
 *
 * @usage
 *   - Include this file in "octopus_platform.h".
 *   - Define only one PLATFORM_xxx macro.
 *   - Define required TASK_MANAGER_STATE_MACHINE_xxx macros.
 *   - Comment unused modules to reduce binary size.
 *
 * @note
 *   - Do not modify this file per source file; changes here affect the whole
 *     project build.
 *   - Some macros may conflict; enable carefully.
 *   - Always rebuild the entire project after modifications.
 ********************************************************************************/

#ifndef __OCTOPUS_CONFIGURATION_H__
#define __OCTOPUS_CONFIGURATION_H__

///////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------//
//                            PLATFORM / RTOS SELECTION                         //
//------------------------------------------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////
// Define which platform and RTOS to use by enabling the corresponding macro.
// ⚠ Only one PLATFORM_xxx macro should be enabled at a time.

// #define PLATFORM_ITE_OPEN_RTOS    // Enable ITE platform with OPEN RTOS
// #define PLATFORM_CST_OSAL_RTOS    // CST platform with OSAL RTOS
// #define PLATFORM_X86_WIND_RTOS    // XB6 platform with WIND RTOS
// #define PLATFORM_STM32_RTOS       // STM32 platform with RTOS
// #define PLATFORM_LINUX_RISC // X86 / ARM Linux platform
#define PLATFORM_NATION_RTOS
///////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------//
//                          TASK MANAGER CONFIGURATION                          //
//------------------------------------------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////

// State machine operation mode
#define TASK_MANAGER_STATE_MACHINE_MCU  1   /**< MCU main control mode */
// #define TASK_MANAGER_STATE_MACHINE_SOC 1 /**< SOC mode (reserved) */

///////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------//
//                          Enable / Disable Modules                            //
//------------------------------------------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////

#define TASK_MANAGER_STATE_MACHINE_GPIO     1   /**< GPIO handling */
#define TASK_MANAGER_STATE_MACHINE_ADC     1
#define TASK_MANAGER_STATE_MACHINE_FLASH    1   /**< Flash memory handling */
#define TASK_MANAGER_STATE_MACHINE_SYSTEM 1 /**< System-level state machine */
#define TASK_MANAGER_STATE_MACHINE_UPDATE 1   /**< OTA / Firmware update */

#define TASK_MANAGER_STATE_MACHINE_KEY 1    /**< Key input handling */
#define TASK_MANAGER_STATE_MACHINE_CARINFOR 1 /**< Car information processing */


#define TASK_MANAGER_STATE_MACHINE_PTL 1 /**< PTL protocol handler */
#define TASK_MANAGER_STATE_MACHINE_UPF       1   /**< UART Packet Framework */
#define TASK_MANAGER_STATE_MACHINE_IPC 1 /**< Inter-process communication */
#define TASK_MANAGER_STATE_MACHINE_I2C 1

//#define TASK_MANAGER_STATE_MACHINE_CAN       1   /**< CAN bus protocol */
//#define TASK_MANAGER_STATE_MACHINE_BAFANG    1   /**< Bafang system support */
//#define TASK_MANAGER_STATE_MACHINE_LING_HUI_LIION2 1 /**< Ling Hui battery */

//#define TASK_MANAGER_STATE_MACHINE_LOT4G        1   /**< 4G module handler */
//#define TASK_MANAGER_STATE_MACHINE_BT_MUSIC     1   /**< Classic Bluetooth */

//#define TASK_MANAGER_STATE_MACHINE_SIF       1   /**< SIF protocol handler */
//#define TASK_MANAGER_STATE_MACHINE_BLE       1   /**< Bluetooth Low Energy */
//#define TASK_MANAGER_STATE_MACHINE_BMS       1   /**< Battery Management System */

#define TASK_MANAGER_STATE_MACHINE_AUDIO 1

#define TASK_MANAGER_STATE_MACHINE_LOG_CHANNEL 3
///////////////////////////////////////////////////////////////////////////////////
//------------------------------------------------------------------------------//
//                             FLASH CONFIGURATION                              //
//------------------------------------------------------------------------------//
///////////////////////////////////////////////////////////////////////////////////

//#define FLASH_USE_EEROM_FOR_DATA_SAVING   /**< Use EEPROM instead of Flash */
//#define FLASH_MAPPING_VECT_TABLE_TO_SRAM true /**< Remap vector table to SRAM */

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
#ifdef TARGET_BANK_SLOT_A
#define FLASH_BANK_CONFIG_MODE_SLOT BANK_SLOT_A /**< Select Flash bank slot */
#elif defined(TARGET_BANK_SLOT_B)
#define FLASH_BANK_CONFIG_MODE_SLOT BANK_SLOT_B /**< Select Flash bank slot */
#else
#define FLASH_BANK_CONFIG_MODE_SLOT BANK_SLOT_INVALID /**< Select Flash bank slot */
#endif

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
/**********************************************************************************
 * Audio Processor Configuration
 * 支持 ASP + DSP 型号选择
 *********************************************************************************/

// === ASP 芯片 ID ===
#define ASP_BD37534      1
#define ASP_BD37544      2
#define ASP_TDA7419      3
#define ASP_BD34602      4
#define ASP_LC75341      5
#define ASP_BU32107      6

// === DSP 芯片 ID ===
#define DSP_ADAU1701     101
#define DSP_ADAU1452     102
#define DSP_TMS320C55X   103
#define DSP_CS48L10      104

// === 用户配置 ===

#define AUDIO_ASP_DSP_MODEL   ASP_BD37534   // 默认 ASP

#define VOLUME_CUT_WHEN_NAVI_MIX        9
#define DFT_NAVI_AUDIO_CH   NAVI_AUDIO_LEFT_CH//ռڽעʹʱĬɏ.ш
#ifdef CUSTOM_S217
#define A_SRC_RADIO_EXTRA_GAIN_ATTEN        1
#define A_SRC_HOST_EXTRA_GAIN_ATTEN     7
#define A_SRC_BT_MODULE_EXTRA_GAIN_ATTEN        10
#else
#define A_SRC_RADIO_EXTRA_GAIN_ATTEN        0
#define A_SRC_HOST_EXTRA_GAIN_ATTEN     4
#define A_SRC_BT_MODULE_EXTRA_GAIN_ATTEN        -3
#endif
#define A_SRC_DVD_EXTRA_GAIN_ATTEN      -4
#define A_SRC_AUXIN_EXTRA_GAIN_ATTEN        -7
#define A_SRC_TV_EXTRA_GAIN_ATTEN       0
#define A_NAVI_MIX_EXTRA_DB 9
#define A_BT_PHONE_EXTRA_DB 12
#define A_ALL_EXTRA_DB          3
#define CONFIG_LOUDNESS_DB  6
#endif /* __OCTOPUS_CONFIGURATION_H__ */
