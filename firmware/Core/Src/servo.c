/*
 * servo.c
 *
 *  Created on: Feb 4, 2025
 *      Author: asus
 */

#include "servo.h"
#include "tim.h"

void servo_init(servo_t *_servo, float _min_pulse_width, float _max_pulse_width, float _zero_pulse_width,
				float _min_angle, float _max_angle, float _offset_angle, float _cpos, int _channel)
{
	_servo->min_pulse_width = _min_pulse_width;
	_servo->max_pulse_width = _max_pulse_width;
	_servo->zero_pulse_width = _zero_pulse_width;

	// uncalibrated values
	_servo->calib_min_pulse_width = _min_pulse_width;
	_servo->calib_max_pulse_width = _max_pulse_width;
	_servo->calib_zero_pulse_width = _zero_pulse_width;

	_servo->min_angle = _min_angle;
	_servo->max_angle = _max_angle;
	_servo->offset_angle = _offset_angle;

	_servo->cpos = _cpos;
	_servo->last_angle_adjustment = 0;

	switch (_channel)
	{
		case 0:
			// PC7 - timer 3 channel 2
			_servo->pwm_timer = &htim3;
			_servo->pwm_timer_channel = TIM_CHANNEL_2;
			break;
		case 1:
			// PA6 - timer 3 channel 1
			_servo->pwm_timer = &htim3;
			_servo->pwm_timer_channel = TIM_CHANNEL_1;
			break;
		case 2:
			// PB0 - timer 3 channel 3
			_servo->pwm_timer = &htim3;
			_servo->pwm_timer_channel = TIM_CHANNEL_3;
			break;
		case 3:
			// PB1 - timer 3 channel 4
			_servo->pwm_timer = &htim3;
			_servo->pwm_timer_channel = TIM_CHANNEL_4;
			break;
		default:
			// PC7 - timer 3 channel 2
			_servo->pwm_timer = &htim3;
			_servo->pwm_timer_channel = TIM_CHANNEL_2;
			break;
	}
}

float servo_get_pulse_width_us(servo_t *_servo)
{
    uint16_t pulse_width = __HAL_TIM_GET_COMPARE(_servo->pwm_timer, _servo->pwm_timer_channel);
    float pulse_width_us = (float)pulse_width / PWM_TICKS_PER_US;
    return pulse_width_us;
}

void servo_set_pulse_width_us(servo_t *_servo, float _pulse_width_us)
{
	uint16_t pulse_width = (uint16_t)(_pulse_width_us * PWM_TICKS_PER_US + 0.5f);
	__HAL_TIM_SET_COMPARE(_servo->pwm_timer, _servo->pwm_timer_channel, pulse_width);
}

float servo_get_angle_from_pulse_width(servo_t *_servo)
{
	float pulse_width_us = servo_get_pulse_width_us(_servo);
    float angle;

    if (pulse_width_us > _servo->calib_zero_pulse_width)
    {
        // Positive side
        angle = (pulse_width_us - _servo->calib_zero_pulse_width)
                * (_servo->max_angle - 0)
                / (_servo->calib_max_pulse_width - _servo->calib_zero_pulse_width);
    }
    else if (pulse_width_us < _servo->calib_zero_pulse_width)
    {
        // Negative side
        angle = (pulse_width_us - _servo->calib_zero_pulse_width)
                * (0 - _servo->min_angle)
                / (_servo->calib_zero_pulse_width - _servo->calib_min_pulse_width);
    }
    else
    {
        angle = 0.0;
    }

    return angle;
}

float servo_pulse_for_angle(servo_t *_servo, float _angle)
{
	float pulse_width_us;
	_angle = _angle - _servo->offset_angle;
	if (_angle > 0)
	{
		pulse_width_us = _servo->calib_zero_pulse_width + ((_angle - 0)
					   * (_servo->calib_max_pulse_width - _servo->calib_zero_pulse_width)
					   / (_servo->max_angle - 0));
	}
	else if (_angle < 0)
	{
		pulse_width_us = _servo->calib_zero_pulse_width + ((_angle - 0)
					   * (_servo->calib_zero_pulse_width - _servo->calib_min_pulse_width)
					   / (0 - _servo->min_angle));
	}
	else
	{
		pulse_width_us = _servo->calib_zero_pulse_width;
	}
	return pulse_width_us;
}

void servo_set_angle(servo_t *_servo, float _angle)
{
	// set the current position parameter
	_servo->cpos = _angle;
	servo_set_pulse_width_us(_servo, servo_pulse_for_angle(_servo, _angle));
}

void servo_wait_frame(servo_t *_servo)
{
	// one move step per PWM period: the pulse written now is latched at the next update (preload on)
	// gives up after about two frames, so a stopped timer can't hang the board
	uint32_t start = HAL_GetTick();
	__HAL_TIM_CLEAR_FLAG(_servo->pwm_timer, TIM_FLAG_UPDATE);
	while (__HAL_TIM_GET_FLAG(_servo->pwm_timer, TIM_FLAG_UPDATE) == RESET && HAL_GetTick() - start < 7)
	{
	}
}
