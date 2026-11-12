/*
 * settle.c
 *
 * Pure maths, so it can be run on a PC against a model of the servo (firmware/test/settle_test.c).
 */

#include <math.h>

#include "settle.h"

float settle_step(float error_deg, float trim_us)
{
	if (error_deg <= SETTLE_BAND_DEG && error_deg >= -SETTLE_BAND_DEG)
	{
		return 0.0f;
	}

	float step = SETTLE_GAIN_US_PER_DEG * error_deg;
	if (step > SETTLE_MAX_STEP_US)
	{
		step = SETTLE_MAX_STEP_US;
	}
	else if (step < -SETTLE_MAX_STEP_US)
	{
		step = -SETTLE_MAX_STEP_US;
	}

	// the limit only stops the trim growing: a trim already beyond it (a move can start there) is never
	// stepped back by the excess in one go, only by the ordinary step when the error asks for it
	if (step > 0.0f && trim_us + step > SETTLE_LIMIT_US)
	{
		step = (trim_us < SETTLE_LIMIT_US) ? SETTLE_LIMIT_US - trim_us : 0.0f;
	}
	else if (step < 0.0f && trim_us + step < -SETTLE_LIMIT_US)
	{
		step = (trim_us > -SETTLE_LIMIT_US) ? -SETTLE_LIMIT_US - trim_us : 0.0f;
	}
	return step;
}

float carried_offset_us(float offset_us, float progress, float distance_deg)
{
	if (progress < 0.0f)
	{
		progress = 0.0f;
	}
	else if (progress > 1.0f)
	{
		progress = 1.0f;
	}
	float kept = 1.0f - fabsf(distance_deg) / CARRY_FADE_DEG;	// the part a short move keeps at its end
	if (kept < 0.0f)
	{
		kept = 0.0f;
	}
	// what is kept at the end is limited; the start is not, so the first pulse is always the one being sent
	float end_us = offset_us * kept;
	if (end_us > CARRY_LIMIT_US)
	{
		end_us = CARRY_LIMIT_US;
	}
	else if (end_us < -CARRY_LIMIT_US)
	{
		end_us = -CARRY_LIMIT_US;
	}
	return end_us + (offset_us - end_us) * (1.0f - progress);
}
