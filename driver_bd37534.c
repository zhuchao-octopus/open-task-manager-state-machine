
#include "octopus_platform.h"
#include "octopus_iic.h"
#include "driver_audio.h"

#if AUDIO_ASP_DSP_MODEL == ASP_BD37534

#include "driver_bd37534.h"

static uint8_t g_reg_value[BD37534_REG_NUMS];
static uint8_t g_reg_addr[BD37534_REG_NUMS] =
	{
		0x01, 0x02, 0x03,					/* initial setup 1/2/3 */
		0x05, 0x06,							/* input selector & gain */
		0x20,								/* volume gain or attenuation */
		0x28, 0x29, 0x2A, 0x2B, 0x2C,		/* output fader gain or attenuation */
		0x30,								/* mixing gain or attenuation */
		0x41, 0x44, 0x47, 0x51, 0x54, 0x57, /* EQ setting */
		0x75,								/* loudness gain */
};

static const int8_t g_bd37534_atten_table[8] = {0, -3, -6, -9, -12, -16, -20, -55};

// output type 0: normal volume
static const int8_t g_bd37534_gains[] = {
	-79,					 /* volume 0 */
	-65, -58, -53, -48, -44, /* volume 1~5 */
	-41, -38, -35, -32, -30, /* volume 6~10 */
	-29, -28, -27, -26, -25, /* volume 11~15 */
	-24, -23, -22, -21, -20, /* volume 16~20 */
	-19, -18, -17, -16, -15, /* volume 21~25 */
	-14, -13, -12, -11, -10, /* volume 26~30 */
	-9, -8, -7, -6, -5,		 /* volume 31~35 */
	-4, -2, 0, 2, 4			 /* volume 36~40 */
};

static const int8_t g_bd37534_bt_gains[] = {
	-79,					 /* volume 0 */
	-65, -58, -53, -48, -44, /* volume 1~5 */
	-38, -33, -29, -26, -24, /* volume 6~10 */
	-22, -20, -19, -18, -17, /* volume 11~15 */
	-16, -15, -14, -13, -12, /* volume 16~20 */
	-11, -10, -9, -8, -7,	 /* volume 21~25 */
	-6, -5, -4, -3, -2,		 /* volume 26~30 */
	-1, 0, 1, 1, 2,			 /* volume 31~35 */
	2, 3, 3, 4, 4			 /* volume 36~40 */
};

// static bool g_bd37534_soft_mute = FALSE;
static bool g_bd37534_avoff = false;

static void bd37534_write(uint8_t addr, uint8_t value)
{
	i2c_start();
	i2c_send_byte(BD37534_I2C_W_ADDR);
	i2c_wait_ack();
	i2c_send_byte(addr);
	i2c_wait_ack();
	i2c_send_byte(value);
	i2c_wait_ack();
	i2c_stop();
}

static void bd37534_do_input_mute()
{
	if (/* (!g_bd37534_soft_mute) && */ (!g_bd37534_avoff))
	{
		g_reg_value[BD37534_REG_INPUT_GAIN] &= (uint8_t)(~(1 << 7));
	}
	else
	{
		g_reg_value[BD37534_REG_INPUT_GAIN] |= (1 << 7);
	}
	bd37534_write(g_reg_addr[BD37534_REG_INPUT_GAIN], g_reg_value[BD37534_REG_INPUT_GAIN]);
}

static void bd37534_update_main_vol(uint8_t vol)
{
	int8_t gain;
#if MAX_VOLUME == 40
	if (vol > MAX_VOLUME)
	{
		return;
	}
	if (0 == vol)
	{
		g_reg_value[BD37534_REG_VOLUME_GAIN] = 0xFF;
	}
	else
	{
		if (g_audio_info.bt_phone_on)
		{
			gain = g_bd37534_bt_gains[vol];
		}
		else
		{
			gain = g_bd37534_gains[vol];
		}
		gain += 5;
		// gain += 6;

		if (gain > 15)
		{
			gain = 15;
		}
		else if (gain < -79)
		{
			gain = -79;
		}
		g_reg_value[BD37534_REG_VOLUME_GAIN] = BD37534_GAIN_TO_REG(gain);
	}
	bd37534_write(g_reg_addr[BD37534_REG_VOLUME_GAIN], g_reg_value[BD37534_REG_VOLUME_GAIN]);

#else
#error "we didn't support this max volume" MAX_VOLUME
#endif
}

static void bd37534_update_mix_vol(uint8_t vol, uint8_t vol_ctrl_when_reverse)
{
	int8_t gain;
	uint16_t tmp;

#if MAX_VOLUME == 40
	if (vol > MAX_VOLUME)
	{
		return;
	}

	if ((vol > 0) && g_audio_info.navi_fix_first_pop)
	{
		// audio_set_mute_temporary(200);
		platform_delay_us__(50);
		g_audio_info.navi_fix_first_pop = false;
	}

	if (g_audio_info.reverse_on)
	{
		if (0 == vol_ctrl_when_reverse)
		{
			vol = 0;
		}
		else
		{
			tmp = (u16)(vol) * (u16)(vol_ctrl_when_reverse) / (u16)100;
			vol = tmp & 0xFF;
		}
	}

	if (0 == vol)
	{
		g_reg_value[BD37534_REG_MIXING] = 0xFF;
	}
	else
	{
		gain = g_bd37534_gains[vol];
		gain += A_NAVI_MIX_EXTRA_DB;

		if (NAVI_BREAK_DIRECT != g_audio_info.navi_mix_extra_gain)
		{
			// add extra gain from user
			if (g_audio_info.navi_mix_extra_gain > 0x80)
			{
				gain += (int8_t)(g_audio_info.navi_mix_extra_gain - 0x80);
			}
			else if (g_audio_info.navi_mix_extra_gain < 0x80)
			{
				gain -= (int8_t)(0x80 - g_audio_info.navi_mix_extra_gain);
			}
		}

		// clamp the gain
		if (gain > 15)
		{
			gain = 15;
		}
		else if (gain < -79)
		{
			gain = -79;
		}
		g_reg_value[BD37534_REG_MIXING] = BD37534_GAIN_TO_REG(gain);
	}
	bd37534_write(g_reg_addr[BD37534_REG_MIXING], g_reg_value[BD37534_REG_MIXING]);

#else
#error "we didn't support this max volume" MAX_VOLUME
#endif
}

static void bd37534_update_output_gain(uint8_t fad, uint8_t bal, audio_driver_source_t src, uint8_t front_source)
{
	int8_t fl_atten, fr_atten, rl_atten, rr_atten;
	int8_t extra_atten = 0;
#if MAX_FIELD_LEVEL == 14
	fl_atten = 0;
	fr_atten = 0;
	rl_atten = 0;
	rr_atten = 0;
	if ((AUDIO_SRC_HOST == src) && (front_source == SOURCE_BT))
	{
		src = AUDIO_SRC_BT_MODULE;
	}
	if (front_source == SOURCE_HDMI)
	{
		extra_atten = g_audio_info.extra_input_gain[AUDIO_SRC_HDMI];
	}
	else
	{
		extra_atten = g_audio_info.extra_input_gain[src];
	}
	if ((g_audio_info.bt_phone_on))
	{
		extra_atten += A_BT_PHONE_EXTRA_DB;
	}

	if (extra_atten > 16)
	{
		// audio_dev_update_source can only increase 16dB max, so we handle the left
		extra_atten -= 16;
	}

	if (!g_audio_info.disabled_soundfield)
	{
		if (fad <= 7)
		{
			rl_atten += g_bd37534_atten_table[7 - fad];
			rr_atten += g_bd37534_atten_table[7 - fad];
		}
		else
		{
			fl_atten += g_bd37534_atten_table[fad - 7];
			fr_atten += g_bd37534_atten_table[fad - 7];
		}
		if (bal <= 7)
		{
			fr_atten += g_bd37534_atten_table[7 - bal];
			rr_atten += g_bd37534_atten_table[7 - bal];
		}
		else
		{
			fl_atten += g_bd37534_atten_table[bal - 7];
			rl_atten += g_bd37534_atten_table[bal - 7];
		}
	}

	if (fl_atten < -79)
		fl_atten = -79;
	if (fr_atten < -79)
		fr_atten = -79;
	if (rl_atten < -79)
		rl_atten = -79;
	if (rr_atten < -79)
		rr_atten = -79;

	fl_atten += extra_atten;
	fr_atten += extra_atten;
	rl_atten += extra_atten;
	rr_atten += extra_atten;

	if ((0 != g_audio_info.loudness) && (AUDIO_SRC_NONE != g_audio_info.cur_source))
	{
		// add loudness compansation
		fl_atten += CONFIG_LOUDNESS_DB;
		fr_atten += CONFIG_LOUDNESS_DB;
		rl_atten += CONFIG_LOUDNESS_DB;
		rr_atten += CONFIG_LOUDNESS_DB;
	}

	if ((!g_audio_info.bt_phone_on) && (!g_audio_info.bt_ring_on) && (!g_audio_info.bt_voice_on) && (g_audio_info.navi_break_on))
	{
		if (NAVI_BREAK_DIRECT == g_audio_info.navi_mix_extra_gain)
		{ // navi break directly
			// mute all speakers
			fl_atten = -79;
			fr_atten = -79;
			rl_atten = -79;
			rr_atten = -79;
		}
		else
		{ // navi mix
			// cut down the front speaker
			fl_atten -= VOLUME_CUT_WHEN_NAVI_MIX;
			fr_atten -= VOLUME_CUT_WHEN_NAVI_MIX;
		}
	}

	if (fl_atten < -79)
		fl_atten = -79;
	if (fr_atten < -79)
		fr_atten = -79;
	if (rl_atten < -79)
		rl_atten = -79;
	if (rr_atten < -79)
		rr_atten = -79;
	if (fl_atten > 15)
		fl_atten = 15;
	if (fr_atten > 15)
		fr_atten = 15;
	if (rl_atten > 15)
		rl_atten = 15;
	if (rr_atten > 15)
		rr_atten = 15;

	g_reg_value[BD37534_REG_FADER_1CH_FRONT] = BD37534_GAIN_TO_REG(fl_atten);
	bd37534_write(g_reg_addr[BD37534_REG_FADER_1CH_FRONT], g_reg_value[BD37534_REG_FADER_1CH_FRONT]);
	g_reg_value[BD37534_REG_FADER_2CH_FRONT] = BD37534_GAIN_TO_REG(fr_atten);
	bd37534_write(g_reg_addr[BD37534_REG_FADER_2CH_FRONT], g_reg_value[BD37534_REG_FADER_2CH_FRONT]);
	g_reg_value[BD37534_REG_FADER_1CH_REAR] = BD37534_GAIN_TO_REG(rl_atten);
	bd37534_write(g_reg_addr[BD37534_REG_FADER_1CH_REAR], g_reg_value[BD37534_REG_FADER_1CH_REAR]);
	g_reg_value[BD37534_REG_FADER_2CH_REAR] = BD37534_GAIN_TO_REG(rr_atten);
	bd37534_write(g_reg_addr[BD37534_REG_FADER_2CH_REAR], g_reg_value[BD37534_REG_FADER_2CH_REAR]);
#else
#error "we didn't support this sound field max level" MAX_FIELD_LEVEL
#endif
}

bool audio_dev_init(void)
{
	uint8_t cnt;

	// BD37534_REG_INIT_SET_1:
	// Advanced switch: ON; time of mute 3.2msec; time of gain 14.4msec
	g_reg_value[BD37534_REG_INIT_SET_1] = 0xB7;

	// BD37534_REG_INIT_SET_2:
	// subwoofer: 120HZ fc, from LPF; Level meter: hold, 180degree;
	g_reg_value[BD37534_REG_INIT_SET_2] = 0x83;

	// BD37534_REG_INIT_SET_3:
	// Loudness fo: 800HZ
	g_reg_value[BD37534_REG_INIT_SET_3] = 0x09;

	// BD37534_REG_INPUT_SELECT:
	// input selector: input E2 for HOST
	g_reg_value[BD37534_REG_INPUT_SELECT] = BD37534_INPUT_E2;

	// BD37534_REG_INPUT_GAIN:
	// input gain: mute, 0dB
	g_reg_value[BD37534_REG_INPUT_GAIN] = 0x80;

	// BD37534_REG_VOLUME_GAIN
	// volume gain: mute
	g_reg_value[BD37534_REG_VOLUME_GAIN] = 0xFF;

	// BD37534_REG_FADER_XCH_XXXXX
	// fader gain: mute
	g_reg_value[BD37534_REG_FADER_1CH_FRONT] = 0xFF;
	g_reg_value[BD37534_REG_FADER_2CH_FRONT] = 0xFF;
	g_reg_value[BD37534_REG_FADER_1CH_REAR] = 0xFF;
	g_reg_value[BD37534_REG_FADER_2CH_REAR] = 0xFF;

	// BD37534_REG_FADER_SUBWOOFER
	// subwoofer: +7dB	// increase our subwoofer volume
	g_reg_value[BD37534_REG_FADER_SUBWOOFER] = 0x79;

	// BD37534_REG_MIXING
	// mixing: off
	g_reg_value[BD37534_REG_MIXING] = 0xFF;

	// EQ
	// BASS: 0.5Q,60HZ,0dB; MIDDLE: 1.25Q,1KHZ,0dB; TREBLE: 0.75Q,12K5HZ,0dB
	g_reg_value[BD37534_REG_BASS_SET] = 0x00;
	g_reg_value[BD37534_REG_MIDDLE_SET] = 0x12;
	g_reg_value[BD37534_REG_TREBLE_SET] = 0x20;
	g_reg_value[BD37534_REG_BASS_GAIN] = 0x80;
	g_reg_value[BD37534_REG_MIDDLE_GAIN] = 0x80;
	g_reg_value[BD37534_REG_TREBLE_GAIN] = 0x80;

	// BD37534_REG_LOUD_GAIN
	// Loudness: Hicut1; 0dB
	g_reg_value[BD37534_REG_LOUD_GAIN] = 0x00;

	for (cnt = 0; cnt < BD37534_REG_NUMS; cnt++)
	{
		bd37534_write(g_reg_addr[cnt], g_reg_value[cnt]);
	}

	return true;
}

void audio_dev_deinit(void)
{
}

void audio_dev_update_source(audio_driver_source_t src, uint8_t front_source)
{
	int8_t extra_gain = 0;

	// update input channel selector
	switch (src)
	{
	case AUDIO_SRC_RADIO:
		g_reg_value[BD37534_REG_INPUT_SELECT] = BD37534_INPUT_D;
		break;
	case AUDIO_SRC_HOST:
		g_reg_value[BD37534_REG_INPUT_SELECT] = BD37534_INPUT_E2;
		break;
	case AUDIO_SRC_VTR:
		g_reg_value[BD37534_REG_INPUT_SELECT] = BD37534_INPUT_C;
		break;
	case AUDIO_SRC_DVD:
		g_reg_value[BD37534_REG_INPUT_SELECT] = BD37534_INPUT_E1;
		break;
	case AUDIO_SRC_AUXIN:
		g_reg_value[BD37534_REG_INPUT_SELECT] = BD37534_INPUT_A;
		break;
	case AUDIO_SRC_TV:
		g_reg_value[BD37534_REG_INPUT_SELECT] = BD37534_INPUT_B;
		break;
	default:
		g_reg_value[BD37534_REG_INPUT_SELECT] = BD37534_INPUT_SHORT;
		break;
	}
	if (BD37534_INPUT_SHORT != g_reg_value[BD37534_REG_INPUT_SELECT])
	{
		bd37534_write(g_reg_addr[BD37534_REG_INPUT_SELECT], g_reg_value[BD37534_REG_INPUT_SELECT]);
	}

	// update input mute
	if (BD37534_INPUT_SHORT == g_reg_value[BD37534_REG_INPUT_SELECT])
	{
		g_bd37534_avoff = true;
	}
	else
	{
		g_bd37534_avoff = true;
	}
	bd37534_do_input_mute();

	// update extra gain when switch source
	g_reg_value[BD37534_REG_INPUT_GAIN] &= (uint8_t)(~(0x1F << 0));
	if ((AUDIO_SRC_HOST == src) && (front_source == SOURCE_BT))
	{
		extra_gain = g_audio_info.extra_input_gain[AUDIO_SRC_BT_MODULE];
	}
	else if (front_source == SOURCE_HDMI)
	{
		extra_gain = g_audio_info.extra_input_gain[AUDIO_SRC_HDMI];
	}
	else
	{
		extra_gain = g_audio_info.extra_input_gain[src];
	}

	if ((g_audio_info.bt_phone_on))
	{
		extra_gain += A_BT_PHONE_EXTRA_DB;
	}

	extra_gain += 6;

	if (extra_gain > 0)
	{
		if (extra_gain > 16)
		{
			// we can only increase 16dB max
			extra_gain = 16;
		}
		g_reg_value[BD37534_REG_INPUT_GAIN] |= (extra_gain << 0);
	}
	bd37534_write(g_reg_addr[BD37534_REG_INPUT_GAIN], g_reg_value[BD37534_REG_INPUT_GAIN]);

	// update extra attenuation or not when switch source
	bd37534_update_output_gain(g_audio_info.fader, g_audio_info.balance, src, front_source);
}

void audio_dev_set_soft_mute(bool on)
{
#if 1
	// g_bd37534_soft_mute = on;
	bd37534_do_input_mute();
#endif
}

void audio_dev_update_volume(uint8_t vol)
{
	bd37534_update_main_vol(vol);
}

void audio_dev_update_navi_mix(bool on_off, uint8_t vol_ctrl_when_reverse)
{
	if (false == on_off)
	{
		bd37534_update_mix_vol(0, vol_ctrl_when_reverse);
	}
	else
	{
		bd37534_update_mix_vol(g_audio_info.navi_mix_vol, vol_ctrl_when_reverse);
	}
}

void audio_dev_update_navi_mix_vol(uint8_t vol, uint8_t vol_ctrl_when_reverse)
{
	if (g_audio_info.bt_phone_on)
	{
		bd37534_update_mix_vol(0, vol_ctrl_when_reverse);
	}
	else if (g_audio_info.bt_ring_on)
	{
		bd37534_update_mix_vol(0, vol_ctrl_when_reverse);
	}
	else if (g_audio_info.bt_voice_on)
	{
		bd37534_update_mix_vol(0, vol_ctrl_when_reverse);
	}
	else if (g_audio_info.navi_break_on)
	{
		bd37534_update_mix_vol(vol, vol_ctrl_when_reverse);
	}
}

// update EQ settings according to the AUDIO_INFO's eq_cur_freq & eq_cur_level member

static uint8_t bd37534_calc_eq_gain_reg_value(int8_t level)
{
	if (level > 30)
	{
		return 0x0C;
	}
	else if (level < -30)
	{
		return 0x8C;
	}
	switch (level)
	{
	case 30:
		return 0x0C;
	case 29:
	case 28:
	case 27:
		return 0x0A;
	case 26:
	case 25:
	case 24:
		return 0x09;
	case 23:
	case 22:
	case 21:
		return 0x08;
	case 20:
	case 19:
	case 18:
		return 0x07;
	case 17:
	case 16:
	case 15:
		return 0x06;
	case 14:
	case 13:
	case 12:
		return 0x04;
	case 11:
	case 10:
	case 9:
		return 0x03;
	case 8:
	case 7:
	case 6:
		return 0x02;
	case 5:
	case 4:
	case 3:
		return 0x01;
	case 2:
	case 1:
	case 0:
		return 0x00;
	case -1:
	case -2:
	case -3:
		return 0x81;
	case -4:
	case -5:
	case -6:
		return 0x82;
	case -7:
	case -8:
	case -9:
		return 0x83;
	case -10:
	case -11:
	case -12:
		return 0x84;
	case -13:
	case -14:
	case -15:
		return 0x86;
	case -16:
	case -17:
	case -18:
		return 0x87;
	case -19:
	case -20:
	case -21:
		return 0x88;
	case -22:
	case -23:
	case -24:
		return 0x89;
	case -25:
	case -26:
	case -27:
		return 0x8A;
	case -28:
	case -29:
	case -30:
		return 0x8C;
	default:
		break;
	}
	return 0x00;
}

void audio_dev_update_eq(void)
{
	int8_t val;
	int8_t tmp;

	if (AUDIO_SRC_NONE == g_audio_info.cur_source)
	{
		// force to bypass EQ, not to introduce noise from EQ module
		bd37534_write(g_reg_addr[BD37534_REG_BASS_GAIN], 0x00);
		bd37534_write(g_reg_addr[BD37534_REG_MIDDLE_GAIN], 0x00);
		bd37534_write(g_reg_addr[BD37534_REG_TREBLE_GAIN], 0x00);
		return;
	}

	/* calc the BASS gain */
	val = 0;
	EQ3_U8_TO_S8(g_audio_info.eq_visible_level[EQ_FREQ_60HZ], tmp);
	val += tmp;
	EQ3_U8_TO_S8(g_audio_info.eq_visible_level[EQ_FREQ_100HZ], tmp);
	val += tmp;
	EQ3_U8_TO_S8(g_audio_info.eq_visible_level[EQ_FREQ_120HZ], tmp);
	val += tmp;
	g_reg_value[BD37534_REG_BASS_GAIN] = bd37534_calc_eq_gain_reg_value(val);
	bd37534_write(g_reg_addr[BD37534_REG_BASS_GAIN], g_reg_value[BD37534_REG_BASS_GAIN]);

	/* calc the MIDDLE gain */
	val = 0;
	EQ3_U8_TO_S8(g_audio_info.eq_visible_level[EQ_FREQ_0K5HZ], tmp);
	val += tmp;
	EQ3_U8_TO_S8(g_audio_info.eq_visible_level[EQ_FREQ_1KHZ], tmp);
	val += tmp;
	EQ3_U8_TO_S8(g_audio_info.eq_visible_level[EQ_FREQ_1K5HZ], tmp);
	val += tmp;
	g_reg_value[BD37534_REG_MIDDLE_GAIN] = bd37534_calc_eq_gain_reg_value(val);
	bd37534_write(g_reg_addr[BD37534_REG_MIDDLE_GAIN], g_reg_value[BD37534_REG_MIDDLE_GAIN]);

	/* calc the TREBLE gain */
	val = 0;
	EQ3_U8_TO_S8(g_audio_info.eq_visible_level[EQ_FREQ_10KHZ], tmp);
	val += tmp;
	EQ3_U8_TO_S8(g_audio_info.eq_visible_level[EQ_FREQ_12K5HZ], tmp);
	val += tmp;
	EQ3_U8_TO_S8(g_audio_info.eq_visible_level[EQ_FREQ_15KHZ], tmp);
	val += tmp;
	g_reg_value[BD37534_REG_TREBLE_GAIN] = bd37534_calc_eq_gain_reg_value(val);
	bd37534_write(g_reg_addr[BD37534_REG_TREBLE_GAIN], g_reg_value[BD37534_REG_TREBLE_GAIN]);
}

void audio_dev_update_fader_balance(uint8_t fad, uint8_t bal, uint8_t front_source)
{
	bd37534_update_output_gain(fad, bal, g_audio_info.cur_source, front_source);
}

void audio_dev_update_loudness(uint8_t loud, uint8_t front_source)
{
	if (AUDIO_SRC_NONE == g_audio_info.cur_source)
	{
		loud = 0;
	}

	if (0 == loud)
	{
		// LOUD on->off, set volume first
		bd37534_update_output_gain(g_audio_info.fader, g_audio_info.balance, g_audio_info.cur_source, front_source);
		g_reg_value[BD37534_REG_LOUD_GAIN] &= (uint8_t)(~(0x1F << 0));
		bd37534_write(g_reg_addr[BD37534_REG_LOUD_GAIN], g_reg_value[BD37534_REG_LOUD_GAIN]);
	}
	else
	{
		// LOUD off->on, set Loud first
		g_reg_value[BD37534_REG_LOUD_GAIN] &= (uint8_t)(~(0x1F << 0));
		g_reg_value[BD37534_REG_LOUD_GAIN] |= (CONFIG_LOUDNESS_DB << 0);
		bd37534_write(g_reg_addr[BD37534_REG_LOUD_GAIN], g_reg_value[BD37534_REG_LOUD_GAIN]);
		bd37534_update_output_gain(g_audio_info.fader, g_audio_info.balance, g_audio_info.cur_source, front_source);
	}
}

void audio_dev_update_subwoofer(uint8_t subwoofer)
{
#if MAX_SUBWOOFER_LVL == 14
	int8_t gain;
	//	gain = (subwoofer-7)*2;
	gain = subwoofer; // increase our subwoofer volume
	gain -= 4;
	g_reg_value[BD37534_REG_FADER_SUBWOOFER] = BD37534_GAIN_TO_REG(gain);
	bd37534_write(g_reg_addr[BD37534_REG_FADER_SUBWOOFER], g_reg_value[BD37534_REG_FADER_SUBWOOFER]);
#else
#error "we didn't support this subwoofer max level" MAX_SUBWOOFER_LVL
#endif
}

void audio_dev_update_dsp_settings_1(void)
{
}

void audio_dev_update_dsp_settings_2(void)
{
}

void audio_dev_update_spectrum_data(void)
{
}

#endif
