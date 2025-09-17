
#include "octopus_platform.h" // Include platform-specific header for hardware platform details
#include "driver_audio.h"

static void do_beep(void)
{
	switch (audio_mata_infor.audio_beep_infor.mode)
	{
	case BEEP_MODE_SHORT:
		if (0 == audio_mata_infor.audio_beep_infor.timer)
		{
			TIM_SetCmp3(TIMER_BEEP, 32);
		}
		else if (audio_mata_infor.audio_beep_infor.timer > 36)
		{
			audio_mata_infor.audio_beep_infor.mode = BEEP_MODE_NONE;
		}
		break;
	case BEEP_MODE_DOUBLE:
		if (0 == audio_mata_infor.audio_beep_infor.timer)
		{
			TIM_SetCmp3(TIMER_BEEP, 32);
		}
		else if (50 == audio_mata_infor.audio_beep_infor.timer)
		{
			TIM_SetCmp3(TIMER_BEEP, 0);
		}
		else if (100 == audio_mata_infor.audio_beep_infor.timer)
		{
			TIM_SetCmp3(TIMER_BEEP, 32);
		}
		else if (audio_mata_infor.audio_beep_infor.timer > 120)
		{
			audio_mata_infor.audio_beep_infor.mode = BEEP_MODE_NONE;
		}
		break;
	case BEEP_MODE_LONG:
		if (0 == audio_mata_infor.audio_beep_infor.timer)
		{
			TIM_SetCmp3(TIMER_BEEP, 32);
		}
		else if (audio_mata_infor.audio_beep_infor.timer > 500)
		{
			audio_mata_infor.audio_beep_infor.mode = BEEP_MODE_NONE;
		}
		break;
	case BEEP_MODE_NONE:
		TIM_SetCmp3(TIMER_BEEP, 0);
		audio_mata_infor.audio_beep_infor.state = BEEP_STATE_IDLE;
		audio_set_mute((AUDIO_MUTE_FLAG)(g_audio_info.mute), true);
		break;

	case BEEP_MODE_NUMS:
		break;
	}

	++audio_mata_infor.audio_beep_infor.timer;
}

void beep_init(void)
{
	audio_mata_infor.audio_beep_infor.state = BEEP_STATE_PWR_DOWN;
	audio_mata_infor.audio_beep_infor.mode = BEEP_MODE_NONE;
	audio_mata_infor.audio_beep_infor.timer = 0;
	TIM_SetCmp3(TIMER_BEEP, 0);
}

void beep_main(void)
{
	switch (audio_mata_infor.audio_beep_infor.state)
	{
	case BEEP_STATE_PWR_DOWN:
		audio_mata_infor.audio_beep_infor.state = BEEP_STATE_IDLE;
		break;
	case BEEP_STATE_IDLE:
		break;
	case BEEP_STATE_BEEP_ING:
		audio_set_mute(AUDIO_MUTE_DRIVER, false);
		do_beep();
		break;
	default:
		break;
	}
}

void beep_short_mode(void)
{
	if (BEEP_STATE_IDLE != audio_mata_infor.audio_beep_infor.state)
	{
		return;
	}
	if (0 == audio_get_volume())
	{
		return;
	}
	audio_mata_infor.audio_beep_infor.mode = BEEP_MODE_SHORT;
	audio_mata_infor.audio_beep_infor.state = BEEP_STATE_BEEP_ING;
	audio_mata_infor.audio_beep_infor.timer = 0;
}

void beep_double_mode(void)
{
	if (BEEP_STATE_IDLE != audio_mata_infor.audio_beep_infor.state)
	{
		return;
	}
	if (0 == audio_get_volume())
	{
		return;
	}
	audio_mata_infor.audio_beep_infor.mode = BEEP_MODE_DOUBLE;
	audio_mata_infor.audio_beep_infor.state = BEEP_STATE_BEEP_ING;
	audio_mata_infor.audio_beep_infor.timer = 0;
}

void beep_long_mode(void)
{
	if (BEEP_STATE_IDLE != audio_mata_infor.audio_beep_infor.state)
	{
		return;
	}
	if (0 == audio_get_volume())
	{
		return;
	}
	audio_mata_infor.audio_beep_infor.mode = BEEP_MODE_LONG;
	audio_mata_infor.audio_beep_infor.state = BEEP_STATE_BEEP_ING;
	audio_mata_infor.audio_beep_infor.timer = 0;
}
