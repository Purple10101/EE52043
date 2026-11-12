/*
 * servo.h
 *
 *  Created on: Feb 4, 2025
 *      Author: asus
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdio.h>
#include <math.h>

#include "stm32f4xx_hal.h"

// TIM3 counts at 90 MHz / 9: 0.1 us a tick, 30000 ticks a period (333 Hz)
#define PWM_TICKS_PER_US 10.0f
#define SERVO_FRAME_S 0.003f			// one PWM period: the moves step once a frame (servo_wait_frame)

typedef struct {
	float min_pulse_width;
	float max_pulse_width;
	float zero_pulse_width;

	float calib_min_pulse_width;
	float calib_max_pulse_width;
	float calib_zero_pulse_width;

	float min_angle;
	float max_angle;
	float offset_angle;

	float cpos;
	float last_angle_adjustment;

	TIM_HandleTypeDef *pwm_timer;
	uint32_t pwm_timer_channel;
} servo_t;

void servo_init(servo_t *_servo, float _min_pulse_width, float _max_pulse_width, float _zero_pulse_width,
				float _min_angle, float _max_angle, float _offset_angle, float _cpos, int _channel);

float servo_get_pulse_width_us(servo_t *_servo);
void servo_set_pulse_width_us(servo_t *_servo, float _pulse_width_us);

float servo_get_angle_from_pulse_width(servo_t *_servo);
float servo_pulse_for_angle(servo_t *_servo, float _angle);
void servo_set_angle(servo_t *_servo, float _angle);

// wait for the start of the next PWM period of this servo's timer (its update flag; no update IRQ may use it)
void servo_wait_frame(servo_t *_servo);

#endif /* INC_SERVO_H_ */
