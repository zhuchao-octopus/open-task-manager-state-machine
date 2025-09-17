
#ifndef _AUDIO_DRIVER_H_
#define _AUDIO_DRIVER_H_

#include "octopus_base.h" //  Base include file for the Octopus project.

#define MIN_VOLUME 0
#define MAX_VOLUME 40
#define DEFAULT_VOLUME 12
#define DEFAULT_BT_VOLUME 15

#define MIN_LOUDNESS 0
#define MAX_LOUDNESS 4
#define DEFAULT_LOUDNESS 0
#define MIN_SUBWOOFER_LVL 0

#if AUDIO_ASP_DSP_MODEL == ASP_BU32107
#define MAX_SUBWOOFER_LVL 15
#else
#define MAX_SUBWOOFER_LVL 14
#endif

#define DEFAULT_SUBWOOFER_LVL 9
#define MIN_FIELD_LEVEL 0
#define MAX_FIELD_LEVEL 14
#define DEFAULT_FIELD_LEVEL 7

#define MIN_EQ_LEVEL 0
#define MAX_EQ_LEVEL 20
#define DEFAULT_EQ_LEVEL 10

#define NAVI_BREAK_DIRECT 0xE0

#define SPECTRUM_NUMS 16

#define DSP_SPK_GAIN_MAX 12
#define DSP_SPK_GAIN_MIN 0

typedef enum
{
	SOURCE_TUNER = 0, // HD Radio
	SOURCE_DVD = 1,
	SOURCE_DVDC = 2, // CDC
	SOURCE_TV = 3,
	SOURCE_NAVI = 4,
	SOURCE_AUX = 5, // AUX IN 1
	SOURCE_DTV = 6,
	SOURCE_SD = 7,
	SOURCE_XM = 8, // SERIUS
	SOURCE_IPOD = 9,
	SOURCE_USB = 10,
	SOURCE_CAMERA = 11,
	SOURCE_FRONT_AUX = 12, // AUX IN 2
	SOURCE_BT = 13,
	SOURCE_HDMI = 14,
	SOURCE_HDRADIO = 15,
	SOURCE_AVOFF = 16, // source off
	SOURCE_3G = 17,
	SOURCE_IPOD_USB = 18,
	NUM_OF_SOURCE = 19,
	SOURCE_FOLLOW_FRONT = 0xFE
} audio_source_t;

typedef enum
{
	AUDIO_SRC_NONE = 0,
	AUDIO_SRC_RADIO,
	AUDIO_SRC_HOST,
	AUDIO_SRC_BT_MODULE,
	AUDIO_SRC_DVD,
	AUDIO_SRC_AUXIN,
	AUDIO_SRC_TV,
	AUDIO_SRC_HDMI,
	AUDIO_SRC_VTR,
	AUDIO_SRC_NUMS
} audio_driver_source_t;

#if (AUDIO_ASP_DSP_MODEL == ASP_BU32107)
typedef enum
{
	EQ_MODE_CUSTOM = 0,
	EQ_MODE_FLAT,
	EQ_MODE_JAZZ,
	EQ_MODE_POP,
	EQ_MODE_CLASSIC,
	EQ_MODE_ROCK,
	EQ_MODE_DBB,
	EQ_MODE_NUMS,
	EQ_MODE_CUSTOM_1 = 0x10,
	EQ_MODE_CUSTOM_2 = 0x11,
	EQ_MODE_CUSTOM_3 = 0x12,
	EQ_MODE_CUSTOM_4 = 0x13,
	EQ_MODE_CUSTOM_5 = 0x14,
	EQ_MODE_CUSTOM_6 = 0x15
} EQ_MODE;

typedef enum
{
	EQ_FREQ_80HZ = 0,
	EQ_FREQ_125HZ,
	EQ_FREQ_200HZ_FAKE,
	EQ_FREQ_200HZ,
	EQ_FREQ_315HZ,
	EQ_FREQ_500HZ,
	EQ_FREQ_800HZ,
	EQ_FREQ_1K25HZ_FAKE,
	EQ_FREQ_1K25HZ,
	EQ_FREQ_2KHZ,
	EQ_FREQ_3K15HZ,
	EQ_FREQ_5KHZ,
	EQ_FREQ_8KHZ_FAKE,
	EQ_FREQ_8KHZ,
	EQ_FREQ_12K5HZ,

	EQ_FREQ_NUMS
} EQ_FREQ;

#else

typedef enum
{
	EQ_MODE_CUSTOM = 0,
	EQ_MODE_FLAT,
	EQ_MODE_JAZZ,
	EQ_MODE_POP,
	EQ_MODE_CLASSIC,
	EQ_MODE_ROCK,
	EQ_MODE_NEWS,
	EQ_MODE_URBAN,
	EQ_MODE_TECHNO,
	EQ_MODE_FILM,
	EQ_MODE_MIDI,
	EQ_MODE_NUMS
} EQ_MODE;

typedef enum
{
	/* bass */
	EQ_FREQ_60HZ = 0,
	EQ_FREQ_100HZ,
	EQ_FREQ_120HZ,
	/* middle */
	EQ_FREQ_0K5HZ,
	EQ_FREQ_1KHZ,
	EQ_FREQ_1K5HZ,
	/* treble */
	EQ_FREQ_10KHZ,
	EQ_FREQ_12K5HZ,
	EQ_FREQ_15KHZ,

	EQ_FREQ_NUMS
} EQ_FREQ;

#endif

typedef enum
{
	BEEP_STATE_PWR_DOWN = 0,
	BEEP_STATE_IDLE,
	BEEP_STATE_BEEP_ING,
	BEEP_STATE_NUMS
} BEEP_STATE;

typedef enum
{
	BEEP_MODE_NONE,
	BEEP_MODE_SHORT,
	BEEP_MODE_DOUBLE,
	BEEP_MODE_LONG,
	BEEP_MODE_NUMS
} BEEP_MODE;

typedef enum
{
	AUDIO_MUTE_NONE = 0x00,
	AUDIO_MUTE_SYSTEM = 0x01,
	AUDIO_MUTE_BATT = 0x02,
	AUDIO_MUTE_DRIVER = 0x04,
	AUDIO_MUTE_USER = 0x08,
	AUDIO_MUTE_RADIO = 0x10,
	AUDIO_MUTE_TEMP = 0x20,
	AUDIO_MUTE_EXT_FORCE = 0x40
} AUDIO_MUTE_FLAG;

typedef enum
{
	AUDIO_STATE_PWR_DOWN = 0,
	AUDIO_STATE_PWR_OFF_ING,
	AUDIO_STATE_PWR_ON_ING,
	AUDIO_STATE_INIT_ING,
	AUDIO_STATE_IDLE,
	AUDIO_STATE_NUMS
} AUDIO_STATE;

typedef enum
{
	AUDIO_EVT_NONE,
	AUDIO_EVT_NAVI_EXTRA_GAIN,
	AUDIO_EVT_OUTPUT_TYPE,
	AUDIO_EVT_ADJ_CH_VOL,
	AUDIO_EVT_NUMS
} AUDIO_EVENT;

typedef enum
{
	AUDIO_SRC_SW_STATE_NONE = 0,
	AUDIO_SRC_SW_STATE_PREPARE,
	AUDIO_SRC_SW_STATE_READY,
	AUDIO_SRC_SW_STATE_NUMS
} AUDIO_SOURCE_SWITCH_STATE;

typedef enum
{
	AUDIO_OUTPUT_NORMAL = 0,
	AUDIO_OUTPUT_N_3DB,
	AUDIO_OUTPUT_N_6DB,
	AUDIO_OUTPUT_N_9DB,
	AUDIO_OUTPUT_N_12DB,
	AUDIO_OUTPUT_N_15DB,
	AUDIO_OUTPUT_TYPE_NUMS
} AUDIO_OUTPUT_TYPE;

typedef enum
{
	AUDIO_SF_MODE_ALL = 0,
	AUDIO_SF_MODE_DRIVER,
	AUDIO_SF_MODE_COPILOT,
	AUDIO_SF_MODE_RL,
	AUDIO_SF_MODE_RR,
	AUDIO_SF_MODE_USER,
	AUDIO_SF_MODE_NUMS
} AUDIO_SF_MODE;

typedef enum
{
	AUDIO_SR_MODE_FLAT = 0,
	AUDIO_SR_MODE_RECITAL,
	AUDIO_SR_MODE_CONCERT,
	AUDIO_SR_MODE_BGM,
	AUDIO_SR_MODE_MOVIE,
	AUDIO_SR_MODE_DRAMA,
	AUDIO_SR_MODE_NUMS
} AUDIO_SR_MODE;

typedef enum
{
	AUDIO_SPK_FL = 0,
	AUDIO_SPK_FR,
	AUDIO_SPK_RL,
	AUDIO_SPK_RR,
	AUDIO_SPK_NUMS
} AUDIO_SPK;

typedef struct
{
	AUDIO_STATE state;
	audio_driver_source_t sys_source;
	audio_driver_source_t cur_source;
	uint8_t cur_vol;
	uint8_t system_vol;
	uint8_t navi_mix_vol;
	int8_t extra_input_gain_factory[AUDIO_SRC_NUMS];
	int8_t extra_input_gain_user[AUDIO_SRC_NUMS];
	int8_t extra_input_gain[AUDIO_SRC_NUMS];
	int8_t mute;
	bool soft_mute;
	bool bt_phone_on;
	uint8_t bt_phone_vol;
	bool bt_ring_on;
	uint8_t bt_ring_vol;
	bool navi_on;
	uint8_t navi_vol;
	uint8_t navi_timer;
	bool navi_fix_first_pop;
	bool navi_break_on;
	bool navi_break_on_cache;
	/* navi_mix_extra_gain: */
	/* 0x00: me
	ans no navi mix, but break directly. */
	/* 0x01~0xFF: means navi mix, and the mix extra gain is  */
	/* -127dB ~ 127dB, which 0x80 is 0dB, 0x7F is -1dB, 0x81 is +1dB and etc. */
	uint8_t navi_mix_extra_gain;
	bool rds_ta_break_on;
	bool app_3rd_break_on;
	bool reverse_on;
	uint8_t fader;	 /* 0->14: front->rear */
	uint8_t balance; /* 0->14: left->right */
	uint8_t loudness;
	uint8_t subwoofer;
	uint8_t pwr_timer;
	uint16_t bt_phone_timer;
	uint16_t temp_mute_timer;  /* in ms unit */
	uint16_t temp_mute_timer2; /* in ms unit */
	AUDIO_SOURCE_SWITCH_STATE src_sw_state;
	bool android_snd_on;

	bool bt_voice_on;
	uint8_t bt_voice_vol;
	uint16_t bt_voice_timer;
	bool carplay_phone_on;

	uint8_t output_type;
	//	bool overheat;
	uint8_t ext_force_mute_timer;

	bool loud_on;
	bool hpf_on;
	uint8_t hpf_freq;
	bool dsp_bass_on;
	uint8_t dsp_bass_freq;
	uint8_t dsp_bass_gain;
	bool subwoofer_on;

	/* EQ member */
	EQ_MODE eq_mode;
	uint8_t eq_visible_level[EQ_FREQ_NUMS];
	uint8_t eq_custom_level[EQ_FREQ_NUMS];
	uint8_t eq_custom_level_1[EQ_FREQ_NUMS];
	uint8_t eq_custom_level_2[EQ_FREQ_NUMS];
	uint8_t eq_custom_level_3[EQ_FREQ_NUMS];
	uint8_t eq_custom_level_4[EQ_FREQ_NUMS];
	uint8_t eq_custom_level_5[EQ_FREQ_NUMS];
	uint8_t eq_custom_level_6[EQ_FREQ_NUMS];

	/* Spectrum analyzer */
	bool sa_en;
	uint8_t sa_timer;
	uint8_t sa_data[SPECTRUM_NUMS];

	bool phat_en;
	uint8_t phat_gain;
	bool core_en;
	uint8_t core_gain;
	bool space_en;
	uint8_t space_gain;
	AUDIO_SF_MODE sf_mode; /* sound field mode */
	bool spk_on[AUDIO_SPK_NUMS];
	uint8_t spk_gain[AUDIO_SPK_NUMS];
	uint8_t spk_delay[AUDIO_SPK_NUMS];
	bool spk_user_on[AUDIO_SPK_NUMS];
	uint8_t spk_user_gain[AUDIO_SPK_NUMS];
	uint8_t spk_user_delay[AUDIO_SPK_NUMS];
	AUDIO_SR_MODE surround_mode; /* surround mode */
	uint8_t soundfield_expert_mode;

	bool disabled_soundfield;
} AUDIO_INFO;

typedef struct
{
	uint8_t audio_beep_onoff;
	uint16_t timer;
	BEEP_STATE state;
	BEEP_MODE mode;
} audio_beep_infor_t;

typedef struct
{
	uint8_t audio_output_type;
	uint8_t audio_navi_mix_extra_gain;
	uint8_t audio_vol_ctrl_when_reverse;
	uint8_t audio_front_source;
	audio_beep_infor_t audio_beep_infor;
} audio_mata_infor_t;

#define IS_VALID_VOLUME(vol) (/*(vol>=MIN_VOLUME)&&*/ (vol <= MAX_VOLUME))
#if AUDIO_ASP_DSP_MODEL == ASP_BU32107
#define IS_VALID_EQ_MODE(mode) ((mode < EQ_MODE_NUMS) || ((mode >= EQ_MODE_CUSTOM_1) && (mode <= EQ_MODE_CUSTOM_6)))
#else
#define IS_VALID_EQ_MODE(mode) (mode < EQ_MODE_NUMS)
#endif

#define IS_VALID_EQ_FREQ(freq) (freq < EQ_FREQ_NUMS)
#define IS_VALID_FAD_BAL_LEVEL(lvl) ((lvl >= MIN_FIELD_LEVEL) && (lvl <= MAX_FIELD_LEVEL))
#define IS_VALID_LOUDNESS_LEVEL(lvl) ((lvl >= MIN_LOUDNESS) && (lvl <= MAX_LOUDNESS))
#define IS_VALID_SUBWOOFER_LEVEL(lvl) ((lvl >= MIN_SUBWOOFER_LVL) && (lvl <= MAX_SUBWOOFER_LVL))

#define EQ3_U8_TO_S8(u, s)                                  \
	do                                                      \
	{                                                       \
		s = (int8_t)((int8_t)u - (int8_t)DEFAULT_EQ_LEVEL); \
	} while (0);

/***************************************
 * our varible
 **************************************/
extern AUDIO_INFO g_audio_info; // do not write any member outside audio_driver.c, just read only.
extern audio_mata_infor_t audio_mata_infor;
/*******************************************************
 *   interface for system
 *******************************************************/
void audio_init(void);

void audio_main(void);
void audio_set_source(audio_source_t av_src);
void audio_set_volume(uint8_t ch, uint8_t vol);
uint8_t audio_get_volume(void);
uint8_t audio_volume_up(void);
uint8_t audio_volume_down(void);

void audio_set_vol_ctrl_when_reverse(uint8_t val);
void audio_set_mute(AUDIO_MUTE_FLAG flag, bool mute);
void audio_set_soft_mute(bool mute);
void audio_set_mute_temporary(uint16_t time_ms);
void audio_set_mute_temporary2(uint16_t time_ms);
void audio_set_bt_phone(bool on);
void audio_set_bt_ring(bool on);
void audio_set_carplay_phone(bool on);
void audio_set_bt_voice(bool on);
void audio_set_navi_break(bool on);
void audio_set_rds_ta_break(bool on);
void audio_set_app_3rd_break(bool on);
void audio_set_android_sound_on(bool on);
void audio_set_eq_mode(EQ_MODE mode);
void audio_set_eq_custom_level(EQ_FREQ freq, uint8_t level);
void audio_save_eq_user_mode(uint8_t id);
void audio_set_fader(uint8_t level);
void audio_set_balance(uint8_t level);
void audio_set_loudness(uint8_t level);
void audio_set_subwoofer(uint8_t level);
void audio_set_navi_extra_gain(uint8_t gain);
void audio_set_pwr_ctrl(bool on);
void audio_set_output_type(uint8_t type);
void audio_adjust_channel_volume(void);
void audio_set_loud_on(bool on);
void audio_set_hpf(bool on, uint8_t freq);
void audio_set_dsp_bass(bool on, uint8_t freq, uint8_t gain);
void audio_set_subwoofer_on(bool on);
void audio_set_spectrum_analyzer(bool on);
void audio_set_dsp_phat(bool on, uint8_t gain);
void audio_set_dsp_core(bool on, uint8_t gain);
void audio_set_dsp_space(bool on, uint8_t gain);
void audio_set_dsp_soundfield(uint8_t mode);
void audio_set_dsp_speaker(uint8_t spk, bool on, uint8_t gain, uint8_t delay);
void audio_set_dsp_surround(uint8_t mode);
void audio_set_dsp_sf_expert_mode(uint8_t mode);
void audio_soundfield_ctrl(uint8_t disabled);

/*******************************************************
 *   interface for eq
 *******************************************************/
void audio_eq_init(void);
void audio_eq_update_mode(EQ_MODE mode);
void audio_eq_update_level(EQ_FREQ freq, uint8_t level);

/*******************************************************
 *   interface for physical audio device
 *******************************************************/
bool audio_dev_init(void);
void audio_dev_deinit(void);
void audio_dev_update_source(audio_driver_source_t src, uint8_t front_source);
void audio_dev_update_volume(uint8_t vol);
void audio_dev_update_navi_mix(bool on_off, uint8_t vol_ctrl_when_reverse);
void audio_dev_update_navi_mix_vol(uint8_t vol, uint8_t vol_ctrl_when_reverse);
void audio_dev_update_eq(void); // update EQ settings according to the AUDIO_INFO's eq_cur_freq & eq_cur_level member
void audio_dev_update_fader_balance(uint8_t fad, uint8_t bal, uint8_t front_source);
void audio_dev_update_loudness(uint8_t loud, uint8_t front_source);
void audio_dev_update_subwoofer(uint8_t subwoofer);
void audio_dev_update_dsp_settings_1(void);
void audio_dev_update_dsp_settings_2(void);
void audio_dev_update_spectrum_data(void);
void audio_dev_set_soft_mute(bool on);

#endif /* _AUDIO_DRIVER_H_ */
