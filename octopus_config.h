/*******************************************************************************
 * @file    octopus_config.h
 * @brief   Configuration header for Octopus system hardware and feature setup.
 *
 * This file defines constants and macros used to configure various subsystems
 * such as tuner models, DVD modules, AUX input modes, fan temperature behavior,
 * navigation audio routing, parking detection, power voltage thresholds, and more.
 *
 * It also defines default feature selections and firmware versioning mechanisms
 * based on compile-time macros. The configuration ensures the embedded system
 * is customized per hardware and software design specifications.
 *
 * @version 1.0.0
 * @date    2024-12-11
 * @author  Octopus Team
 ******************************************************************************/

#ifndef __OCTOPUS_CONFIG_DEFINE_H__
#define __OCTOPUS_CONFIG_DEFINE_H__

typedef enum
{
	FV_SRC_AUXIN=0x05,
	FV_SRC_BACKCAR=0x0B,
	FV_SRC_F_CAM=0x0C,
	FV_SRC_L_CAM=0x20,
	FV_SRC_R_CAM=0x21
} FRONT_VIDEO_SRC;

// ====================== Tuner device selection ==========================
#define TUNER_NONE                0
#define TUNER_NXP_TEF6606         1
#define TUNER_NXP_TEA6851         2
#define TUNER_SILICONLAB_SI476X   3
#define TUNER_NXP_TEA668X         4
#define TUNER_ST_TDA7708          5
#define TUNER_SILICONLAB_SI475X   6

// ====================== DVD module types ================================
#define DVD_DVS                  0
#define DVD_XINWA                1

// ====================== AUX input selection ============================
#define AUXIN_FRONT              0
#define AUXIN_REAR               1
#define AUXIN_FRONT_REAR         2

// ====================== Fan startup temperature ========================
#define FAN_START_ALL_TIME       0
#define FAN_START_45_C           1
#define FAN_START_46             2
#define FAN_START_47             3
#define FAN_START_48             4
#define FAN_START_49             5
#define FAN_START_50             6
#define FAN_START_51             7
#define FAN_START_52             8
#define FAN_START_53             9
#define FAN_START_54            10
#define FAN_START_55            11
#define FAN_START_56            12
#define FAN_START_57            13
#define FAN_START_58            14
#define FAN_CLOSE               15

// ====================== Navigation audio channel =======================
#define NAVI_AUDIO_LEFT_CH              0
#define NAVI_AUDIO_RIGHT_CH             1
#define NAVI_AUDIO_LEFT_RIGHT_CH        2
#define NAVI_AUDIO_ALL_CH               3

// ====================== Parking detection mode =========================
#define PARKING_OFF_MODEL               0 // No detection
#define PARKING_LEVEL_MODEL             1 // Voltage-based detection
#define PARKING_SERIAL_MODEL            2 // Serial-based detection

// ====================== Illumination modes =============================
#define ILLUMIN_AUTO_MODE               0
#define ILLUMIN_NIGHT_MODE              1
#define ILLUMIN_DAY_MODE                2

// ====================== Illumination detect method ====================
#define ILLUMIN_DETCET_MODE_LEVEL       0
#define ILLUMIN_DETCET_MODE_PULSE       1

// ====================== ASP (audio processor) selection ===============
#define ASP_NONE        0
#define ASP_TDA7415     1
#define ASP_TDA7719     2
#define ASP_BD37534     3
#define ASP_BU32107     4

// ====================== Voltage detection levels (ADC counts) =========
#define VOLT_5V         64
#define VOLT_6V         78
#define VOLT_7V         90
#define VOLT_8V         103
#define VOLT_8V5        109
#define VOLT_9V         116
#define VOLT_9V5        123
#define VOLT_10V        130
#define VOLT_10V5       136
#define VOLT_12V        156
#define VOLT_15V        193
#define VOLT_16V        206
#define VOLT_17V        219

// ====================== Encoder trigger edge type =====================
#define RISE_LEVEL               1
#define FALL_LEVEL               2
#define RISE_FALL_LEVEL          3

/*****************************
 *       Configuration       *
 *****************************/
/*---- Audio ----*/
#define ASP_MODEL       ASP_BD37534   // Select non-DSP audio processor
#define VOLUME_CUT_WHEN_NAVI_MIX      9
#define DFT_NAVI_AUDIO_CH             NAVI_AUDIO_LEFT_CH
#define A_SRC_RADIO_EXTRA_GAIN_ATTEN      0
#define A_SRC_HOST_EXTRA_GAIN_ATTEN       0
#define A_SRC_BT_MODULE_EXTRA_GAIN_ATTEN  0
#define A_SRC_DVD_EXTRA_GAIN_ATTEN        0
#define A_SRC_AUXIN_EXTRA_GAIN_ATTEN      -4
#define A_SRC_TV_EXTRA_GAIN_ATTEN         0
#define A_SRC_VTR_EXTRA_GAIN_ATTEN        -4
#define A_NAVI_MIX_EXTRA_DB               12
#define A_BT_PHONE_EXTRA_DB               3
#define A_ALL_EXTRA_DB                    3
#define CONFIG_LOUDNESS_DB                12

/*---- Radio ----*/
#define TUNER_MODEL  TUNER_SILICONLAB_SI475X
#define RADIO_DEFAULT_REGION    RADIO_AREA_EUROPE
#define RDS_FUNCTION            1 // Enable RDS support

#ifndef TEA668X_V205
#define TEA668X_V102
#endif

#if TUNER_MODEL==TUNER_SILICONLAB_SI475X
#define FIX_RADIO_BAND_SWITCH_POP
#endif

/*---- DVD ----*/
#define DVD_FUNCTION_ENABLE
#define DFT_DVD_MODE        DVD_XINWA

/*---- AUX Input ----*/
#define DFT_AUXIN_MODE      AUXIN_REAR

/*---- Car Signals ----*/
#define DFT_PARKING_MODE     PARKING_LEVEL_MODEL
#define DFT_ILLUMIN_MODE     ILLUMIN_AUTO_MODE
#define DEF_ILLUMIN_DETECT_MODE ILLUMIN_DETCET_MODE_LEVEL

/*---- Power Management ----*/
#define HIGH_VOLT_PROTECT_OFF     VOLT_17V
#define HIGH_VOLT_PROTECT_ON      VOLT_16V
#define LOW_VOLT_PROTECT_OFF      VOLT_8V5
#define LOW_VOLT_PROTECT_ON       VOLT_9V5

/*---- Miscellaneous ----*/
#define DEF_BEEP_ONOFF    ON
#define DFT_FAN_START_TEMPERATURE FAN_START_50

/*****************************
 *       Version Info        *
 *****************************/

#define MCU_VERSION_BASE   __DATE__ "_" __TIME__

#if !defined(MCU_VERSION_BASE)
#error "MCU_VERSION_BASE must be defined!"
#endif

#if ASP_MODEL==ASP_BU32107
#define MCU_DSP     "D_"
#else
#define MCU_DSP     ""
#endif

#if TUNER_MODEL==TUNER_ST_TDA7708
#define MCU_RADIO   "R7708_"
#else
#define MCU_RADIO   ""
#endif

#ifdef KD2000
#define MCU_VERSION "CSM4_KD2000_" MCU_DSP MCU_RADIO MCU_VERSION_BASE
#elif defined (CUSTOM_887)
#define MCU_VERSION "CSM4_6_" MCU_DSP MCU_RADIO MCU_VERSION_BASE
#else
#define MCU_VERSION "CSM4_AT6751_" MCU_DSP MCU_RADIO MCU_VERSION_BASE
#endif

#endif // __OCTOPUS_CONFIG_H__
