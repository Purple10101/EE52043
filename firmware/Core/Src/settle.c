/*
 * settle.c
 *
 * Pure maths, so it can be run on a PC against a model of the servo (firmware/test/settle_test.c).
 */

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

	if (trim_us + step > SETTLE_LIMIT_US)
	{
		step = SETTLE_LIMIT_US - trim_us;
	}
	else if (trim_us + step < -SETTLE_LIMIT_US)
	{
		step = -SETTLE_LIMIT_US - trim_us;
	}
	return step;
}
