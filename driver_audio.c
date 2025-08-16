

#include "octopus_platform.h"
#include "octopus_gpio.h"
#include "driver_audio.h"

#ifdef TASK_MANAGER_STATE_MACHINE_AUDIO
//#define AUDIO_INIT_SRC_FOR_TEST AUDIO_SRC_HOST
AUDIO_INFO g_audio_info;	// do not write any member outside audio_driver.c, just read only.
audio_mata_infor_t audio_mata_infor;

static uint8_t _audio_change_volume(uint8_t start_vol, uint8_t target_vol)
{
	uint8_t vol;
	if (start_vol < target_vol) {
		vol = start_vol + 1;
	} else if (start_vol > target_vol) {
		vol = start_vol - 1;
	} else {
		vol = start_vol;
	}
	return vol;
}

static audio_driver_source_t _audio_select_source(audio_source_t av_src)
{
	audio_driver_source_t src = AUDIO_SRC_NONE;
	switch (av_src) {
		case SOURCE_TUNER:
		case SOURCE_XM:
		case SOURCE_HDRADIO:
			src = AUDIO_SRC_RADIO;
			break;
		case SOURCE_DVD:
		case SOURCE_DVDC:
			src = AUDIO_SRC_DVD;
			break;
		case SOURCE_TV:
		case SOURCE_DTV:
		case SOURCE_HDMI:
			src = AUDIO_SRC_TV;
			break;
		case SOURCE_AUX:
			src = AUDIO_SRC_AUXIN;
			break;
		case SOURCE_FRONT_AUX:
			src = AUDIO_SRC_VTR;
			break;
		case SOURCE_SD:
		case SOURCE_USB:
			src = AUDIO_SRC_HOST;
			break;
		case SOURCE_BT:
		//if (1==g_bt_type) {
		src = AUDIO_SRC_HOST;
		//} else if (2==g_bt_type) {
		//	src = AUDIO_SRC_HOST;
		//} else {
		//  src = AUDIO_SRC_BT_MODULE;
		//}
			break;
		case SOURCE_AVOFF:
		case NUM_OF_SOURCE:
		default:
			src = AUDIO_SRC_NONE;
			break;
	}
	return src;
}

static void _audio_do_set_navi_break(bool on_off)
{
	if (g_audio_info.navi_break_on!= on_off) {
		g_audio_info.navi_break_on = on_off;
		audio_dev_update_navi_mix(g_audio_info.navi_break_on,audio_mata_infor.audio_vol_ctrl_when_reverse);
		audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);
		audio_dev_update_fader_balance(g_audio_info.fader, g_audio_info.balance,audio_mata_infor.audio_front_source);
	}

	if (g_audio_info.navi_break_on) {
		audio_set_mute(AUDIO_MUTE_DRIVER, false);
	}	
}

static void audio_reverse_handler(uint8_t reverse_detect)
{
	bool should_update_navi_mix_vol = false;
	if ( (!reverse_detect) && (g_audio_info.reverse_on) ) {
		should_update_navi_mix_vol = true;
		audio_set_mute_temporary(1000);
	} else if ( (reverse_detect) && (!g_audio_info.reverse_on) ) {
		should_update_navi_mix_vol = true;
	}
	
	if (reverse_detect) {
		g_audio_info.reverse_on = true;
	} else {
		g_audio_info.reverse_on = false;
	}

	if (should_update_navi_mix_vol) {
		audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);
	}
}

static void audio_source_handler(uint8_t g_bt_type,bool g_fake_pwr_off)
{
	audio_driver_source_t src = AUDIO_SRC_NONE;

	if (g_audio_info.bt_phone_on) {
		if (g_audio_info.carplay_phone_on) {
			src = AUDIO_SRC_HOST;
		} else {
			if (1==g_bt_type) {
				src = AUDIO_SRC_HOST;
			} else if (2==g_bt_type) {
				src = AUDIO_SRC_HOST;
			} else {
				src = AUDIO_SRC_BT_MODULE;
			}
		}
	} else if (g_audio_info.bt_ring_on) {
		src = AUDIO_SRC_HOST;
	} else if (g_audio_info.bt_voice_on) {
		src = AUDIO_SRC_BT_MODULE;
	} else if (g_fake_pwr_off) {
		src = AUDIO_SRC_NONE;
	} else if (g_audio_info.rds_ta_break_on) {
		src = AUDIO_SRC_RADIO;
	} else if (g_audio_info.app_3rd_break_on) {
		src = AUDIO_SRC_HOST;
	} else {
		src = g_audio_info.sys_source;
	}

	if (g_audio_info.bt_phone_timer) {
		// we just switch to/from bt phone
		--g_audio_info.bt_phone_timer;
		src = AUDIO_SRC_NONE;
	}

	if ( (AUDIO_SRC_RADIO == src) && (AUDIO_MUTE_RADIO==g_audio_info.mute) ){
		src = AUDIO_SRC_NONE;
	}

	if ( (AUDIO_SRC_DVD == src) ) {
		g_audio_info.src_sw_state = AUDIO_SRC_SW_STATE_READY;
		src = AUDIO_SRC_NONE;
	}

	if ((AUDIO_SRC_HOST == src) && (!g_audio_info.android_snd_on)) {
		src =AUDIO_SRC_NONE;
	}

	if (AUDIO_SRC_NONE != src) {
		audio_set_mute(AUDIO_MUTE_DRIVER, false);
	}
	
	if (src != g_audio_info.cur_source) 
	{
		if (AUDIO_SRC_HOST == g_audio_info.cur_source) 
		{
			// we are exiting HOST channel, do navi mix on if need
			if (g_audio_info.navi_break_on_cache) {
				_audio_do_set_navi_break(true);
			}
		} 
		else if (AUDIO_SRC_HOST == src) {
			// we are entering HOST channel, do naiv mix off if it's already on
			// if (g_audio_info.navi_break_on_cache) {
			//	_audio_do_set_navi_break(FALSE);
			//}
		}
		
		g_audio_info.cur_source = src;
		if (g_audio_info.app_3rd_break_on) {
			g_audio_info.src_sw_state = AUDIO_SRC_SW_STATE_READY;
		} else {
			g_audio_info.src_sw_state = AUDIO_SRC_SW_STATE_PREPARE;
		}
	}
	if (AUDIO_SRC_SW_STATE_READY == g_audio_info.src_sw_state) {
		audio_dev_update_source(g_audio_info.cur_source,audio_mata_infor.audio_front_source);
		audio_dev_update_eq();
		audio_dev_update_loudness(g_audio_info.loudness,audio_mata_infor.audio_front_source);
		g_audio_info.src_sw_state = AUDIO_SRC_SW_STATE_NONE;
	}

	if (g_audio_info.navi_timer>1) {
		--g_audio_info.navi_timer;
		if (1==g_audio_info.navi_timer) {
			if (g_audio_info.navi_break_on_cache) {
				_audio_do_set_navi_break(true);
			}
		}
	}
}

static void audio_volume_handler(uint8_t vol_ctrl_when_reverse)
{
	uint8_t vol, target_vol;
	uint16_t tmp;

	target_vol = g_audio_info.system_vol;
	if (g_audio_info.bt_phone_on) 
	{
		target_vol = g_audio_info.bt_phone_vol;
	} 
	else if (g_audio_info.bt_ring_on) 
	{
		target_vol = g_audio_info.bt_ring_vol;
	} 
	else if (g_audio_info.bt_voice_on) 
	{
		target_vol = g_audio_info.bt_voice_vol;
	} 
	else if (g_audio_info.reverse_on) 
	{
		if (0==vol_ctrl_when_reverse) {
			target_vol = 0;
		} else {
			tmp = (uint16_t)(target_vol) * (uint16_t)(vol_ctrl_when_reverse) / (uint16_t)100;
			target_vol = tmp&0xFF;
		}
	}

	if (AUDIO_SRC_SW_STATE_PREPARE == g_audio_info.src_sw_state) {
		target_vol = 0;
	}
	
	vol = _audio_change_volume(g_audio_info.cur_vol, target_vol);

	if (vol != g_audio_info.cur_vol) {
		g_audio_info.cur_vol = vol;
		audio_dev_update_volume(g_audio_info.cur_vol);
	} else {
		if (AUDIO_SRC_SW_STATE_PREPARE == g_audio_info.src_sw_state) {
			g_audio_info.src_sw_state = AUDIO_SRC_SW_STATE_READY;
		}
	}
}

static void audio_event_handler(uint8_t event_id,uint16_t event_parameter)
{
	//EVENT *pEvt;
	//pEvt=GetEvent(AUDIO_MODULE);
	switch(event_id) {
		case AUDIO_EVT_NAVI_EXTRA_GAIN:
			audio_set_navi_extra_gain(LSB_BIT(event_parameter));
			if (audio_mata_infor.audio_navi_mix_extra_gain != g_audio_info.navi_mix_extra_gain) {
				audio_mata_infor.audio_navi_mix_extra_gain = g_audio_info.navi_mix_extra_gain;
				//ak_flash_save_info();
			}
			///PostEvent(WINCE_MODULE, TX_TO_GUI_SYSTEM_NAVI_MIX_INFO, g_audio_info.navi_mix_extra_gain);
			break;
		case AUDIO_EVT_OUTPUT_TYPE:
			if (0x80 == LSB_BIT(event_parameter)) {
//				g_audio_info.overheat = FALSE;
			} else if (0x81 == LSB_BIT(event_parameter)) {
//				g_audio_info.overheat = TRUE;
//				if (g_audio_info.system_vol > DEFAULT_VOLUME) {
//					g_audio_info.system_vol = DEFAULT_VOLUME;
//				}
			} else {
				audio_set_output_type(LSB_BIT(event_parameter));
				if (audio_mata_infor.audio_output_type != g_audio_info.output_type) {
					audio_mata_infor.audio_output_type = g_audio_info.output_type;
					//ak_flash_save_info();
				}
				///PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_CUR_MAX_OUTPUT_VOL, NONE);
			}
			break;
		case AUDIO_EVT_ADJ_CH_VOL:
			audio_adjust_channel_volume();
			break;
		default:
			break;
	}
}

static void audio_temp_mute_handler(void)
{
	if (0!=g_audio_info.temp_mute_timer) {
		--g_audio_info.temp_mute_timer;
		if (0==g_audio_info.temp_mute_timer) {
			audio_set_mute(AUDIO_MUTE_TEMP, false);
		}
	}
	if (0!=g_audio_info.temp_mute_timer2) {
		--g_audio_info.temp_mute_timer2;
		if (0==g_audio_info.temp_mute_timer2) {
			audio_dev_set_soft_mute(false);
		}
	}
}

static void audio_ext_force_mute_handler(void)
{
}

static void audio_sa_handler(void)
{
}

void audio_init(void)
{
	int cnt;

	g_audio_info.state = AUDIO_STATE_PWR_DOWN;
	g_audio_info.cur_source = AUDIO_SRC_NONE;
#ifdef AUDIO_INIT_SRC_FOR_TEST
	g_audio_info.sys_source = AUDIO_INIT_SRC_FOR_TEST;	// for test
#else
	g_audio_info.sys_source = AUDIO_SRC_NONE;
#endif
	g_audio_info.android_snd_on = false;
	g_audio_info.cur_vol = 0;
	g_audio_info.system_vol = DEFAULT_VOLUME;
	g_audio_info.navi_mix_vol = 30;
	g_audio_info.extra_input_gain_factory[AUDIO_SRC_NONE] = 0;
	g_audio_info.extra_input_gain_factory[AUDIO_SRC_RADIO] = -4;
	g_audio_info.extra_input_gain_factory[AUDIO_SRC_HOST] = 0;
	g_audio_info.extra_input_gain_factory[AUDIO_SRC_BT_MODULE] = 0;
	g_audio_info.extra_input_gain_factory[AUDIO_SRC_DVD] = -4;
	g_audio_info.extra_input_gain_factory[AUDIO_SRC_AUXIN] = -6;
	g_audio_info.extra_input_gain_factory[AUDIO_SRC_TV] = 0;
	g_audio_info.extra_input_gain_factory[AUDIO_SRC_HDMI] = -2;
	g_audio_info.extra_input_gain_factory[AUDIO_SRC_VTR] = -3;
	g_audio_info.extra_input_gain_user[AUDIO_SRC_NONE] = 0;
	g_audio_info.extra_input_gain_user[AUDIO_SRC_RADIO] = 0;
	g_audio_info.extra_input_gain_user[AUDIO_SRC_HOST] = 0;
	g_audio_info.extra_input_gain_user[AUDIO_SRC_BT_MODULE] = 0;
	g_audio_info.extra_input_gain_user[AUDIO_SRC_DVD] = 0;
	g_audio_info.extra_input_gain_user[AUDIO_SRC_AUXIN] = 0;
	g_audio_info.extra_input_gain_user[AUDIO_SRC_TV] = 0;
	g_audio_info.extra_input_gain_user[AUDIO_SRC_HDMI] = 0;
	g_audio_info.extra_input_gain_user[AUDIO_SRC_VTR] = 0;
	for (cnt=0; cnt<AUDIO_SRC_NUMS; cnt++) {
		g_audio_info.extra_input_gain[cnt] = g_audio_info.extra_input_gain_factory[cnt] +
										g_audio_info.extra_input_gain_user[cnt];
	}
	g_audio_info.mute = 0;
	g_audio_info.soft_mute = false;
	g_audio_info.bt_phone_on = false;
	g_audio_info.bt_phone_vol = DEFAULT_BT_VOLUME;
	g_audio_info.bt_ring_on = false;
	g_audio_info.bt_ring_vol = DEFAULT_BT_VOLUME;
	g_audio_info.navi_on = false;
	g_audio_info.navi_vol = DEFAULT_VOLUME;
	g_audio_info.navi_timer = 0;
	g_audio_info.carplay_phone_on = false;
	g_audio_info.navi_break_on = false;
	g_audio_info.navi_fix_first_pop = true;
	g_audio_info.navi_break_on_cache = false;
	g_audio_info.app_3rd_break_on = false;
	g_audio_info.rds_ta_break_on = false;
	g_audio_info.reverse_on = false;
	g_audio_info.fader = DEFAULT_FIELD_LEVEL;
	g_audio_info.balance = DEFAULT_FIELD_LEVEL;
	g_audio_info.loudness = DEFAULT_LOUDNESS;
	g_audio_info.subwoofer = DEFAULT_SUBWOOFER_LVL;
	g_audio_info.pwr_timer = 0;
	g_audio_info.bt_phone_timer = 0;
	g_audio_info.temp_mute_timer = 0;
	g_audio_info.temp_mute_timer2 = 0;
	g_audio_info.src_sw_state = AUDIO_SRC_SW_STATE_NONE;
	g_audio_info.bt_voice_on = false;
	g_audio_info.bt_voice_vol = DEFAULT_VOLUME;
	g_audio_info.bt_voice_timer = 0;
//	g_audio_info.overheat = false;
	g_audio_info.ext_force_mute_timer = 0;
	g_audio_info.loud_on = false;
	g_audio_info.hpf_on = false;
	g_audio_info.hpf_freq = 0;
	g_audio_info.dsp_bass_on = false;
	g_audio_info.dsp_bass_freq = 0;
	g_audio_info.dsp_bass_gain = 0;
	g_audio_info.subwoofer_on = false;
	g_audio_info.phat_en = false;
	g_audio_info.phat_gain = 0;
	g_audio_info.core_en = false;
	g_audio_info.core_gain = 0;
	g_audio_info.space_en = false;
	g_audio_info.space_gain = 0;
	g_audio_info.sf_mode = AUDIO_SF_MODE_USER;
	for (cnt=0; cnt<AUDIO_SPK_NUMS; cnt++) {
		g_audio_info.spk_on[cnt] = true;
		g_audio_info.spk_gain[cnt] = DSP_SPK_GAIN_MAX;
		g_audio_info.spk_delay[cnt] = 0;
		g_audio_info.spk_user_gain[cnt] = DSP_SPK_GAIN_MAX;
		g_audio_info.spk_user_delay[cnt] = 0;
	}
	g_audio_info.soundfield_expert_mode = 0;
	g_audio_info.disabled_soundfield = false;
	audio_eq_init();	
}

void audio_main(void)
{
	if (AUDIO_STATE_PWR_OFF_ING == g_audio_info.state) {
//		audio_set_mute(AUDIO_MUTE_DRIVER, TRUE);
		audio_dev_deinit();
		g_audio_info.pwr_timer = 0;
		g_audio_info.bt_phone_timer = 0;
		g_audio_info.state = AUDIO_STATE_PWR_DOWN;
		g_audio_info.bt_phone_on = false;
		g_audio_info.bt_ring_on = false;
		g_audio_info.navi_on = false;
		g_audio_info.navi_timer = 0;
		g_audio_info.carplay_phone_on = false;
		g_audio_info.navi_break_on = false;
		g_audio_info.navi_fix_first_pop = true;
		g_audio_info.navi_break_on_cache = false;
		g_audio_info.reverse_on = false;
		g_audio_info.bt_voice_on = false;
		g_audio_info.bt_voice_timer = 0;
		return;
	}
	if (AUDIO_STATE_PWR_ON_ING == g_audio_info.state) {
		audio_set_mute(AUDIO_MUTE_USER, false);
		g_audio_info.state = AUDIO_STATE_INIT_ING;
		g_audio_info.pwr_timer = 0;
		return;
	}
	if (AUDIO_STATE_INIT_ING == g_audio_info.state) {

		if (true == audio_dev_init()) {
			g_audio_info.cur_source = g_audio_info.sys_source;
			g_audio_info.bt_phone_on = false;
			g_audio_info.bt_ring_on = false;
			g_audio_info.navi_on = false;
			g_audio_info.navi_timer = 0;
			g_audio_info.carplay_phone_on = false;
			g_audio_info.bt_voice_on = false;
			g_audio_info.navi_break_on = false;
			g_audio_info.navi_fix_first_pop = true;
			g_audio_info.navi_break_on_cache = false;
			g_audio_info.sa_en = false;
			g_audio_info.sa_timer = 0;
			audio_dev_set_soft_mute(false);
			audio_dev_update_source(g_audio_info.cur_source,audio_mata_infor.audio_front_source);
			audio_dev_update_volume(g_audio_info.cur_vol);
			audio_dev_update_fader_balance(g_audio_info.fader, g_audio_info.balance,audio_mata_infor.audio_front_source);
			audio_dev_update_loudness(g_audio_info.loudness,audio_mata_infor.audio_front_source);
			audio_dev_update_subwoofer(g_audio_info.subwoofer);
			audio_dev_update_eq();
			audio_dev_update_dsp_settings_1();
			audio_dev_update_dsp_settings_2();
			g_audio_info.pwr_timer = 0;
			g_audio_info.bt_phone_timer = 0;
			g_audio_info.bt_voice_timer = 0;
			g_audio_info.state = AUDIO_STATE_IDLE;
			g_audio_info.temp_mute_timer = 0;
			g_audio_info.temp_mute_timer2 = 0;
			g_audio_info.src_sw_state = AUDIO_SRC_SW_STATE_NONE;
			g_audio_info.ext_force_mute_timer = 0;
			g_audio_info.soft_mute = false;
			audio_set_mute(AUDIO_MUTE_TEMP, false);
		}
		return;
	}

	if (AUDIO_STATE_IDLE == g_audio_info.state) {
		//audio_event_handler();
		//audio_reverse_handler();
		//audio_source_handler();
		//audio_volume_handler();
		//audio_temp_mute_handler();
		//audio_ext_force_mute_handler();
		//audio_sa_handler();
	}
}

void audio_set_source(audio_source_t av_src)
{
	audio_driver_source_t src;
	src = _audio_select_source(av_src);
	g_audio_info.sys_source = src;
}

void audio_set_volume(uint8_t ch, uint8_t vol)
{
	switch (ch) 
{
		case 1:
			g_audio_info.system_vol = vol;
			break;
		case 2:
			g_audio_info.navi_mix_vol = vol;
			audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);
			break;
		case 3:
			g_audio_info.bt_ring_vol = vol;
			break;
		case 4:
			g_audio_info.bt_phone_vol = vol;
			break;
	}
	if ( (1==ch) && (0!=vol) ) {
		audio_set_mute(AUDIO_MUTE_USER, false);
	}
}

uint8_t audio_get_volume(void)
{
	if ( g_audio_info.mute & AUDIO_MUTE_USER ) {
		return 0;
	} else if (g_audio_info.bt_phone_on) {
		return g_audio_info.bt_phone_vol;
	} else if (g_audio_info.bt_ring_on) {
		return g_audio_info.bt_ring_vol;
	} else if (g_audio_info.navi_on) {
		return g_audio_info.navi_mix_vol;
	} else if (g_audio_info.bt_voice_on) {
		return g_audio_info.bt_voice_vol;
	} else {
		return g_audio_info.system_vol;
	}
}

uint8_t audio_volume_up(void)
{
	uint8_t vol;
	uint8_t done=0;

	if (g_audio_info.bt_phone_on) {
		vol = g_audio_info.bt_phone_vol + 1;
		if (IS_VALID_VOLUME(vol)) {
			g_audio_info.bt_phone_vol = vol;
			done=1;
		}
	} else if (g_audio_info.bt_ring_on) {
		vol = g_audio_info.bt_ring_vol + 1;
		if (IS_VALID_VOLUME(vol)) {
			g_audio_info.bt_ring_vol = vol;
			done=1;
		}
	} else if (g_audio_info.navi_on) {
		vol = g_audio_info.navi_mix_vol + 1;
		if (IS_VALID_VOLUME(vol)) {
			g_audio_info.navi_mix_vol = vol;
			audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);
			done=1;
		}
	} else if (g_audio_info.bt_voice_on) {
		vol = g_audio_info.bt_voice_vol + 1;
		if (IS_VALID_VOLUME(vol)) {
			g_audio_info.bt_voice_vol = vol;
			done=1;
		}
	} else {
		if ( 0== (g_audio_info.mute & AUDIO_MUTE_USER) ) {
			vol = g_audio_info.system_vol + 1;
			if (IS_VALID_VOLUME(vol)) {
				g_audio_info.system_vol = vol;
				done=1;
			}
		}
	}

	if ( (g_audio_info.bt_phone_on) || (g_audio_info.bt_ring_on) || (g_audio_info.navi_on) ) {
		audio_set_mute((AUDIO_MUTE_FLAG)(g_audio_info.mute), true);
	} else {
		audio_set_mute(AUDIO_MUTE_USER, false);
	}

	return done;
}

uint8_t audio_volume_down(void)
{
	uint8_t vol;
	uint8_t done=0;

	if (g_audio_info.bt_phone_on) {
		vol = g_audio_info.bt_phone_vol - 1;
		if (IS_VALID_VOLUME(vol)) {
			g_audio_info.bt_phone_vol = vol;
			done=1;
		}
	} else if (g_audio_info.bt_ring_on) {
		vol = g_audio_info.bt_ring_vol - 1;
		if (IS_VALID_VOLUME(vol)) {
			g_audio_info.bt_ring_vol = vol;
			done=1;
		}
	} else if (g_audio_info.navi_on) {
		vol = g_audio_info.navi_mix_vol - 1;
		if (IS_VALID_VOLUME(vol)) {
			g_audio_info.navi_mix_vol = vol;
			audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);
			done=1;
		}
	} else if (g_audio_info.bt_voice_on) {
		vol = g_audio_info.bt_voice_vol - 1;
		if (IS_VALID_VOLUME(vol)) {
			g_audio_info.bt_voice_vol = vol;
			done=1;
		}
	} else {
		if ( 0== (g_audio_info.mute & AUDIO_MUTE_USER) ) {
			vol = g_audio_info.system_vol - 1;
			if (IS_VALID_VOLUME(vol)) {
				g_audio_info.system_vol = vol;
				done=1;
			}
		}
	}

	if ( (g_audio_info.bt_phone_on) || (g_audio_info.bt_ring_on) || (g_audio_info.navi_on) ) {
		audio_set_mute((AUDIO_MUTE_FLAG)(g_audio_info.mute), true);
	} else {
		audio_set_mute(AUDIO_MUTE_USER, false);
	}

	return done;
}

void audio_set_mute(AUDIO_MUTE_FLAG flag, bool mute)
{
	if (false == mute) {
		g_audio_info.mute = (g_audio_info.mute & (~flag));
	} else {
		g_audio_info.mute = (g_audio_info.mute | flag);
	}

	if (g_audio_info.mute & AUDIO_MUTE_EXT_FORCE) {
		AUDIO_DSP_MUTE;
		return;
	}

	if (AUDIO_MUTE_RADIO==g_audio_info.mute) {
		// we will set audio channel to none, if just radio mute
		audio_set_soft_mute(false);
		AUDIO_DSP_UNMUTE;
		return;
	}

	if (g_audio_info.navi_on && !(g_audio_info.mute&AUDIO_MUTE_TEMP)) {
		audio_set_soft_mute(false);
		AUDIO_DSP_UNMUTE;
	} else if (g_audio_info.bt_ring_on && !(g_audio_info.mute&AUDIO_MUTE_TEMP)) {
		audio_set_soft_mute(false);
		AUDIO_DSP_UNMUTE;
	} else if (g_audio_info.bt_phone_on && !(g_audio_info.mute&AUDIO_MUTE_TEMP)) {
		audio_set_soft_mute(false);
		AUDIO_DSP_UNMUTE;
	} else if (0 == g_audio_info.mute) {
		audio_set_soft_mute(false);
		AUDIO_DSP_UNMUTE;
	} else if (BEEP_STATE_BEEP_ING == audio_mata_infor.audio_beep_infor.state) {
		// do nothing
	} else {
		audio_set_soft_mute(true);
		AUDIO_DSP_MUTE;
	}
}
void audio_set_soft_mute(bool mute)
{
	g_audio_info.soft_mute = mute;
	audio_dev_set_soft_mute(mute);
}

void audio_set_mute_temporary(uint16_t time_ms)
{
	uint16_t tmp;

	tmp = time_ms/12;
	if (tmp > g_audio_info.temp_mute_timer) {
		// our audio_main is in the 12ms task
		g_audio_info.temp_mute_timer = tmp;		
	}

	if (0!=g_audio_info.temp_mute_timer) {
		audio_set_mute(AUDIO_MUTE_TEMP, TRUE);
	} else {
		audio_set_mute(AUDIO_MUTE_TEMP, FALSE);
	}
}

void audio_set_mute_temporary2(uint16_t time_ms)
{
	uint16_t tmp;

	tmp = time_ms/12;
	if (0==tmp) {
		g_audio_info.temp_mute_timer2 = 4;
	} else if (tmp > g_audio_info.temp_mute_timer2) {
		// our audio_main is in the 12ms task
		g_audio_info.temp_mute_timer2 = tmp;		
	}

	if (0!=g_audio_info.temp_mute_timer2) {
		audio_dev_set_soft_mute(TRUE);
	} else {
		audio_dev_set_soft_mute(FALSE);
	}
}

void audio_set_bt_phone(bool on)
{
	if (g_audio_info.bt_phone_on != on) {
//		audio_set_mute_temporary(800);
		g_audio_info.bt_phone_on = on;
		audio_dev_update_fader_balance(g_audio_info.fader, g_audio_info.balance,audio_mata_infor.audio_front_source);
		g_audio_info.bt_phone_timer = 300;

		// make sure volume will update according BT phone on/off
		audio_dev_update_volume(g_audio_info.cur_vol);

		// make sure adjust navi mix vol when in/out bt phone
		audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);

		// force unmute when enter/exit BT phone
//		if (g_audio_info.mute & AUDIO_MUTE_USER) {
//			audio_set_mute(AUDIO_MUTE_USER, FALSE);
//			PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_VOLUME_INFO, NONE);
//			PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_FLAG_INFO, NONE);
//		}

		// let audio has chance to force unmute, when bt phone on
		audio_set_mute((AUDIO_MUTE_FLAG)(g_audio_info.mute), TRUE);
	}
}

void audio_set_bt_ring(bool on)
{
	if (g_audio_info.bt_ring_on != on) {
//		audio_set_mute_temporary(800);
		g_audio_info.bt_ring_on = on;
		audio_dev_update_fader_balance(g_audio_info.fader, g_audio_info.balance,audio_mata_infor.audio_front_source);
		g_audio_info.bt_phone_timer = 300;

		// make sure volume will update according BT phone on/off
		audio_dev_update_volume(g_audio_info.cur_vol);

		// make sure adjust navi mix vol when in/out bt phone
		audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);

		// force unmute when enter/exit BT phone
//		if (g_audio_info.mute & AUDIO_MUTE_USER) {
//			audio_set_mute(AUDIO_MUTE_USER, FALSE);
//			PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_VOLUME_INFO, NONE);
//			PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_FLAG_INFO, NONE);
//		}

		// let audio has chance to force unmute, when bt phone on
		audio_set_mute((AUDIO_MUTE_FLAG)(g_audio_info.mute), TRUE);
	}
}

void audio_set_bt_voice(bool on)
{
	if (g_audio_info.bt_voice_on != on) {
		g_audio_info.bt_voice_on = on;
		audio_dev_update_fader_balance(g_audio_info.fader, g_audio_info.balance,audio_mata_infor.audio_front_source);
		g_audio_info.bt_voice_timer = 300;

		// make sure adjust navi mix vol when in/out bt voice
		audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);

		// let audio has chance to force unmute, when bt voice on
		audio_set_mute((AUDIO_MUTE_FLAG)(g_audio_info.mute), TRUE);
	}
}

void audio_set_carplay_phone(bool on)
{
	g_audio_info.carplay_phone_on = on;
}

void audio_set_navi_break(bool on)
{
	g_audio_info.navi_on = on;
	g_audio_info.navi_break_on_cache = on;

	// let mute do it's logic when navi
	audio_set_mute((AUDIO_MUTE_FLAG)(g_audio_info.mute), TRUE);

	if (!on) {
		g_audio_info.navi_timer = 0;
	}
	
	if (on && (AUDIO_SRC_HOST == g_audio_info.cur_source)) {
		// do not make mix if we are in HOST channel
		if (0==g_audio_info.navi_timer) {
			g_audio_info.navi_timer = 20;
		}
		return;
	}
	_audio_do_set_navi_break(on);
}

void audio_set_rds_ta_break(bool on)
{
	g_audio_info.rds_ta_break_on = on;
}

void audio_set_app_3rd_break(bool on)
{
	g_audio_info.app_3rd_break_on = on;
}

void audio_set_android_sound_on(bool on)
{
	g_audio_info.android_snd_on = on;
}

void audio_set_eq_mode(EQ_MODE mode)
{
	if (!IS_VALID_EQ_MODE(mode)) {
		return;
	}
	if (mode != g_audio_info.eq_mode) {
		audio_eq_update_mode(mode);
		audio_dev_update_eq();
		//PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_EQUALIZER_INFO, NONE);
	}
}

void audio_set_eq_custom_level(EQ_FREQ freq, uint8_t level)
{
	if (!IS_VALID_EQ_FREQ(freq)) {
		return;
	}

	audio_eq_update_level(freq, level);
	audio_dev_update_eq();
	//PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_EQUALIZER_INFO, NONE);
}

void audio_save_eq_user_mode(uint8_t id)
{
#if ASP_MODEL==ASP_BU32107
	uint8_t cnt;
	for (cnt=0; cnt<EQ_FREQ_NUMS; cnt++) {
		switch (id) {
			case EQ_MODE_CUSTOM_1:
				g_audio_info.eq_custom_level_1[cnt] = g_audio_info.eq_custom_level[cnt];
				break;
			case EQ_MODE_CUSTOM_2:
				g_audio_info.eq_custom_level_2[cnt] = g_audio_info.eq_custom_level[cnt];
				break;
			case EQ_MODE_CUSTOM_3:
				g_audio_info.eq_custom_level_3[cnt] = g_audio_info.eq_custom_level[cnt];
				break;
			case EQ_MODE_CUSTOM_4:
				g_audio_info.eq_custom_level_4[cnt] = g_audio_info.eq_custom_level[cnt];
				break;
			case EQ_MODE_CUSTOM_5:
				g_audio_info.eq_custom_level_5[cnt] = g_audio_info.eq_custom_level[cnt];
				break;
			case EQ_MODE_CUSTOM_6:
				g_audio_info.eq_custom_level_6[cnt] = g_audio_info.eq_custom_level[cnt];
				break;
			default:
				return;
		}
	}		

	g_audio_info.eq_mode = (EQ_MODE)id;
#endif
}

void audio_set_fader(uint8_t level)
{
	if (level>MAX_FIELD_LEVEL) {
		return;
	}
	if (level != g_audio_info.fader) {
		g_audio_info.fader = level;
		audio_dev_update_fader_balance(g_audio_info.fader, g_audio_info.balance,audio_mata_infor.audio_front_source);
		//PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_EQUALIZER_INFO, NONE);
	}
}

void audio_set_balance(uint8_t level)
{
	if (level>MAX_FIELD_LEVEL) {
		return;
	}
	if (level != g_audio_info.balance) {
		g_audio_info.balance = level;
		audio_dev_update_fader_balance(g_audio_info.fader, g_audio_info.balance,audio_mata_infor.audio_front_source);
		//PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_EQUALIZER_INFO, NONE);
	}
}

void audio_set_loudness(uint8_t level)
{
	if (level>MAX_LOUDNESS) {
		return;
	}
	if (level != g_audio_info.loudness) {
		g_audio_info.loudness = level;
		audio_dev_update_loudness(g_audio_info.loudness,audio_mata_infor.audio_front_source);
		//PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_EQUALIZER_INFO, NONE);
	}
}

void audio_set_subwoofer(uint8_t level)
{
	if (level > MAX_SUBWOOFER_LVL) {
		return;
	}
	if (level != g_audio_info.subwoofer) {
		g_audio_info.subwoofer = level;
		audio_dev_update_subwoofer(g_audio_info.subwoofer);
		//PostEvent(WINCE_MODULE, TX_TO_GUI_AUDIO_EQUALIZER_INFO, NONE);
	}
}


void audio_set_navi_extra_gain(uint8_t gain)
{
	if (0==gain) {
		// for legacy MCU comm protocal support
		gain = NAVI_BREAK_DIRECT;
	}
	g_audio_info.navi_mix_extra_gain = gain;
	audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);
	audio_dev_update_fader_balance(g_audio_info.fader, g_audio_info.balance,audio_mata_infor.audio_front_source);
}

void audio_set_pwr_ctrl(bool on)
{
	if (FALSE == on) {
		g_audio_info.state = AUDIO_STATE_PWR_OFF_ING;
	} else {
		g_audio_info.state = AUDIO_STATE_PWR_ON_ING;
	}
}

void audio_set_vol_ctrl_when_reverse(uint8_t val)
{
	if (val>100) {
		return;
	}
	audio_mata_infor.audio_vol_ctrl_when_reverse = val;
	audio_dev_update_navi_mix_vol(g_audio_info.navi_mix_vol,audio_mata_infor.audio_vol_ctrl_when_reverse);
}

void audio_set_output_type(uint8_t type)
{
	if (type >= AUDIO_OUTPUT_TYPE_NUMS) {
		return;
	}
	g_audio_info.output_type = type;
}

void audio_adjust_channel_volume(void)
{
	uint8_t cnt;
	for (cnt=0; cnt<AUDIO_SRC_NUMS; cnt++) {
		g_audio_info.extra_input_gain[cnt] = g_audio_info.extra_input_gain_factory[cnt] +
										g_audio_info.extra_input_gain_user[cnt];
	}

	// force the audio device to update our new extra gain
	audio_dev_update_source(g_audio_info.cur_source,audio_mata_infor.audio_front_source);
	audio_dev_update_volume(g_audio_info.cur_vol);
}

void audio_set_loud_on(bool on)
{
	g_audio_info.loud_on = on;
	audio_dev_update_loudness(g_audio_info.loudness,audio_mata_infor.audio_front_source);
}

void audio_set_hpf(bool on, uint8_t freq)
{
	g_audio_info.hpf_on = on;
	g_audio_info.hpf_freq = freq;
	audio_dev_update_dsp_settings_1();
}

void audio_set_dsp_bass(bool on, uint8_t freq, uint8_t gain)
{
	g_audio_info.dsp_bass_on = on;
	g_audio_info.dsp_bass_freq = freq;
	g_audio_info.dsp_bass_gain = gain;
	audio_dev_update_dsp_settings_1();
}

void audio_set_subwoofer_on(bool on)
{
	g_audio_info.subwoofer_on = on;
	audio_dev_update_subwoofer(g_audio_info.subwoofer);
}

void audio_set_spectrum_analyzer(bool on)
{
	g_audio_info.sa_en = on;
}

void audio_set_dsp_phat(bool on, uint8_t gain)
{
	g_audio_info.phat_en = on;
	g_audio_info.phat_gain = gain;
	audio_dev_update_dsp_settings_1();
}

void audio_set_dsp_core(bool on, uint8_t gain)
{
	g_audio_info.core_en = on;
	g_audio_info.core_gain = gain;
	audio_dev_update_dsp_settings_1();
}

void audio_set_dsp_space(bool on, uint8_t gain)
{
	g_audio_info.space_en = on;
	g_audio_info.space_gain = gain;
	audio_dev_update_dsp_settings_1();
}

void audio_set_dsp_soundfield(uint8_t mode)
{
	uint8_t cnt;

	g_audio_info.sf_mode = (AUDIO_SF_MODE)mode;
	switch (mode) {
		case AUDIO_SF_MODE_ALL:
		case AUDIO_SF_MODE_DRIVER:
		case AUDIO_SF_MODE_COPILOT:
		case AUDIO_SF_MODE_RL:
		case AUDIO_SF_MODE_RR:
			for (cnt=0; cnt<AUDIO_SPK_NUMS; cnt++) {
				g_audio_info.spk_on[cnt] = TRUE;
				g_audio_info.spk_gain[cnt] = DSP_SPK_GAIN_MAX;
				g_audio_info.spk_delay[cnt] = 0;
			}
			break;
		case AUDIO_SF_MODE_USER:
			for (cnt=0; cnt<AUDIO_SPK_NUMS; cnt++) {
				g_audio_info.spk_on[cnt] = g_audio_info.spk_user_on[cnt];
				g_audio_info.spk_gain[cnt] = g_audio_info.spk_user_gain[cnt];
				g_audio_info.spk_delay[cnt] = g_audio_info.spk_user_delay[cnt];
			}		
			break;
	}

	switch (mode) {
		case AUDIO_SF_MODE_DRIVER:
			g_audio_info.spk_gain[AUDIO_SPK_FL] = DSP_SPK_GAIN_MAX-3;
			g_audio_info.spk_delay[AUDIO_SPK_FL] = 8;
			break;
		case AUDIO_SF_MODE_COPILOT:
			g_audio_info.spk_gain[AUDIO_SPK_FR] = DSP_SPK_GAIN_MAX-3;
			g_audio_info.spk_delay[AUDIO_SPK_FR] = 8;
			break;
		case AUDIO_SF_MODE_RL:
			g_audio_info.spk_gain[AUDIO_SPK_RL] = DSP_SPK_GAIN_MAX-3;
			g_audio_info.spk_delay[AUDIO_SPK_RL] = 8;
			break;
		case AUDIO_SF_MODE_RR:
			g_audio_info.spk_gain[AUDIO_SPK_RR] = DSP_SPK_GAIN_MAX-3;
			g_audio_info.spk_delay[AUDIO_SPK_RR] = 8;
			break;
		default:
			break;
	}
	audio_dev_update_dsp_settings_2();
}

void audio_set_dsp_speaker(uint8_t spk, bool on, uint8_t gain, uint8_t delay)
{
	uint8_t cnt;

	// update the custom level from current visible level, because
	// maybe we are adjusting custom level from the preset levels.
	for (cnt=0; cnt<AUDIO_SPK_NUMS; cnt++) {
		g_audio_info.spk_user_gain[cnt] = g_audio_info.spk_gain[cnt];
		g_audio_info.spk_user_delay[cnt] = g_audio_info.spk_delay[cnt];
	}		

	g_audio_info.spk_user_on[spk] = on;
	g_audio_info.spk_on[spk] = on;
	if (gain > DSP_SPK_GAIN_MAX)
		gain = DSP_SPK_GAIN_MAX;
	g_audio_info.spk_user_gain[spk] = gain;
	g_audio_info.spk_user_delay[spk] = delay;
	g_audio_info.spk_gain[spk] = gain;
	g_audio_info.spk_delay[spk] = delay;

	g_audio_info.sf_mode = AUDIO_SF_MODE_USER;

	audio_dev_update_dsp_settings_2();
}

void audio_set_dsp_surround(uint8_t mode)
{
	g_audio_info.surround_mode = (AUDIO_SR_MODE)mode;
	audio_dev_update_dsp_settings_2();
}

void audio_set_dsp_sf_expert_mode(uint8_t mode)
{
	g_audio_info.soundfield_expert_mode = mode;
	audio_dev_update_dsp_settings_2();
}

void audio_soundfield_ctrl(uint8_t disabled)
{
	if (0==disabled) {
		g_audio_info.disabled_soundfield = FALSE;
	} else {
		g_audio_info.disabled_soundfield = TRUE;
	}
	audio_dev_update_fader_balance(g_audio_info.fader, g_audio_info.balance,audio_mata_infor.audio_front_source);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if 0 //ASP_MODEL==ASP_BU32107

const u8 g_eq_preset_level[EQ_MODE_NUMS-1][EQ_FREQ_NUMS]={
                /*  80HZ    125HZ  200HZ_   200HZ   315HZ  500HZ    800HZ   1.25K_     1.25K     2K     3.15K    5K     8K_    8K    12.5K  */
/*FLAT*/    {    10,      10,       10,        10,        10,      10,       10,       10,        10,        10,      10,     10,    10,    10,    10    },
/*JAZZ*/    {    14,      15,       11,        11,        10,       8,        10,       10,        10,        11,      12,     11,    13,    13,    15    },
/*POP*/     {    13,      14,       15,        15,        13,      11,       10,        8,          8,          8,       10,     13,    14,    14,    15    },
/*CLSC*/    {    11,      12,       10,        10,        10,       10,       10,       10,        10,        10,      10,    11,    13,    13,    15    },
/*ROCK*/   {    13,      14,       13,         13,         6,        7,        10,       12,       12,       11,      13,     15,    13,    13,    14    },
/*DBB*/     {    11,      12,       13,         13,         8,        7,        10,       12,       12,       11,      13,     15,    17,    17,    15    },
};

#else

const u8 g_eq_preset_level[EQ_MODE_NUMS-1][EQ_FREQ_NUMS]={
                /*  60HZ    100HZ  120HZ   0.5KHZ  1KHZ    1.5KHZ  10KHZ   12.5KHZ 15KHZ	*/
/*FLAT*/    {    10,      10,       10,        10,      10,       10,       10,        10,        10    },
/*JAZZ*/    {    20,      16,       12,        10,       6,         2,       14,         17,       20    },
/*POP*/     {     9,       11,       13,        17,      20,       17,       11,          9,         7    },
/*CLSC*/    {    20,      18,       16,         8,        2,        8,        16,        18,       20    },
/*ROCK*/    {    20,      18,       16,         9,        6,        9,       16,         18,       20    },
/*NEWS*/   {    5,        8,        11,        14,       17,      14,       11,          8,        5    },
/*URBN*/    {    18,      16,       14,        10,        9,       10,       14,        16,       18    },
/*TECH*/    {    20,      18,       16,        10,        9,       10,       12,        14,       16    },
/*FILM*/     {     7,       8,        11,        14,       18,       14,       11,         8,        7    },
/*MIDI*/     {    20,      18,       16,        10,        9,       10,       12,        14,       16    }
};

#endif

void audio_eq_init(void)
{
	u8 cnt;
	g_audio_info.eq_mode = EQ_MODE_FLAT;
	for (cnt=0; cnt<EQ_FREQ_NUMS; cnt++) {
		g_audio_info.eq_visible_level[cnt] = DEFAULT_EQ_LEVEL;
		g_audio_info.eq_custom_level[cnt] = DEFAULT_EQ_LEVEL;
		g_audio_info.eq_custom_level_1[cnt] = DEFAULT_EQ_LEVEL;
		g_audio_info.eq_custom_level_2[cnt] = DEFAULT_EQ_LEVEL;
		g_audio_info.eq_custom_level_3[cnt] = DEFAULT_EQ_LEVEL;
		g_audio_info.eq_custom_level_4[cnt] = DEFAULT_EQ_LEVEL;
		g_audio_info.eq_custom_level_5[cnt] = DEFAULT_EQ_LEVEL;
		g_audio_info.eq_custom_level_6[cnt] = DEFAULT_EQ_LEVEL;
	}
}

static void _audio_eq_update_value(void)
{
	u8 cnt;
	
  #if ASP_MODEL==ASP_BU32107
	if (g_audio_info.eq_mode >= EQ_MODE_CUSTOM_1) {
		for (cnt=0; cnt<EQ_FREQ_NUMS; cnt++) {
			switch (g_audio_info.eq_mode) {
				case EQ_MODE_CUSTOM_1:
					g_audio_info.eq_visible_level[cnt] = g_audio_info.eq_custom_level_1[cnt];
					break;
				case EQ_MODE_CUSTOM_2:
					g_audio_info.eq_visible_level[cnt] = g_audio_info.eq_custom_level_2[cnt];
					break;
				case EQ_MODE_CUSTOM_3:
					g_audio_info.eq_visible_level[cnt] = g_audio_info.eq_custom_level_3[cnt];
					break;
				case EQ_MODE_CUSTOM_4:
					g_audio_info.eq_visible_level[cnt] = g_audio_info.eq_custom_level_4[cnt];
					break;
				case EQ_MODE_CUSTOM_5:
					g_audio_info.eq_visible_level[cnt] = g_audio_info.eq_custom_level_5[cnt];
					break;
				case EQ_MODE_CUSTOM_6:
					g_audio_info.eq_visible_level[cnt] = g_audio_info.eq_custom_level_6[cnt];
					break;
				default:
					break;
			}
		}	

		return;
	}
  #endif
	
	if (EQ_MODE_CUSTOM == g_audio_info.eq_mode) 
	{
		for (cnt=0; cnt<EQ_FREQ_NUMS; cnt++) {
			g_audio_info.eq_visible_level[cnt] = g_audio_info.eq_custom_level[cnt];
		}
	} 
	else 
	{
		for (cnt=0; cnt<EQ_FREQ_NUMS; cnt++) {
			g_audio_info.eq_visible_level[cnt] = g_eq_preset_level[g_audio_info.eq_mode-1][cnt];
		}
	}
}

void audio_eq_update_mode(EQ_MODE mode)
{
	g_audio_info.eq_mode = mode;
	_audio_eq_update_value();
}

void audio_eq_update_level(EQ_FREQ freq, u8 level)
{
	u8 cnt;

	// update the custom level from current visible level, because
	// maybe we are adjusting custom level from the preset levels.
	for (cnt=0; cnt<EQ_FREQ_NUMS; cnt++) {
		g_audio_info.eq_custom_level[cnt] = g_audio_info.eq_visible_level[cnt];
	}		
	
	g_audio_info.eq_mode = EQ_MODE_CUSTOM;
	g_audio_info.eq_custom_level[freq] = level;
	_audio_eq_update_value();
}

#endif

