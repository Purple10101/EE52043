/*
 * servo_controller.c
 *
 *  Created on: Feb 2, 2025
 *      Author: asus
 */

#include "servo_controller.h"
#include "settle.h"
#include "potentiometers.h"

// function prototypes
// ----------------------------------------------------------------------------------------------------

void calc_double_s_parameters(float T, float h, float alpha, float beta,
							  float *T_a, float *T_j,
							  float *v_max, float *a_max, float *j_max);

float calc_double_s_value(float T, float h, float T_a, float T_j,
						  float v_max, float a_max, float j_max,
						  float t);

// ----------------------------------------------------------------------------------------------------

void servo_controller_init(servo_controller_t *_servo_controller,
						   servo_t *_axis_0, servo_t *_axis_1, servo_t *_axis_2, servo_t *_axis_3)
{
    _servo_controller->axis_0 = _axis_0;
    _servo_controller->axis_1 = _axis_1;
    _servo_controller->axis_2 = _axis_2;
    _servo_controller->axis_3 = _axis_3;

    servo_init(_axis_0, 2500, 500, 1500, -90, 90, 0, 0, 0);
	servo_init(_axis_1, 2500, 500, 1500, -90, 45, 90, 0, 1);
	servo_init(_axis_2, 2500, 500, 1500, -180, 180, 0, 0, 2);
	servo_init(_axis_3, 500, 2500, 1500, -90, 90, 0, 0, 3);

    printf("servo controller initialised\n");
}

void move_j(servo_controller_t *_servo_controller, int _axis, float _angle, float _T)
{
    // assign axis
    servo_t *axis = _servo_controller->axis_0;
    switch (_axis)
    {
        case 1:
            axis = _servo_controller->axis_1;
            break;
        case 2:
            axis = _servo_controller->axis_2;
            break;
        case 3:
			axis = _servo_controller->axis_3;
			break;
        default:
            break;
    }

	// input variables
    float cpos = axis->cpos;
	float tpos = _angle;        // target position in degrees

	float h = tpos - cpos;     	// displacement in degrees
	float T = _T;				// time to complete move
	float alpha = 0.3;         	// acceleration phase duration as fraction of T
	float beta = 0.3;          	// jerk phase duration as fraction of T_a

	// calculate double-S profile parameters
	float T_a, T_j, v_max, a_max, j_max;
	calc_double_s_parameters(T, h, alpha, beta, &T_a, &T_j, &v_max, &a_max, &j_max);

	// calculate samples
	int samples = (int)(T * 1000.0 / SAMPLING_TIME);

	for (int i = 0; i < samples; i++) {
		double t = i * SAMPLING_TIME / 1000.0; // convert sampling time to seconds
        float dpos = cpos + calc_double_s_value(T, h, T_a, T_j, v_max, a_max, j_max, t);

        servo_set_angle(axis, dpos);

        HAL_Delay(SAMPLING_TIME);
	}
}

void move_js(servo_controller_t *_servo_controller,
		     float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T)
{
    servo_t *axis[AXES] = {_servo_controller->axis_0,
                           _servo_controller->axis_1,
						   _servo_controller->axis_2,
    					   _servo_controller->axis_3};
	// input variables
    float cpos[AXES];
    float tpos[AXES] = {_angle_0, _angle_1, _angle_2, _angle_3};

    float h[AXES];     	        // displacement in degrees
	float T = _T;				// time to complete move
	float alpha = 0.3;         	// acceleration phase duration as fraction of T
	float beta = 0.3;          	// jerk phase duration as fraction of T_a

    // calculate double-S profile parameters
	float T_a[AXES], T_j[AXES], v_max[AXES], a_max[AXES], j_max[AXES];

    for (int i = 0; i < AXES; i++)
    {
        cpos[i] = axis[i]->cpos;
        h[i] = tpos[i] - cpos[i];
    	calc_double_s_parameters(T, h[i], alpha, beta, &T_a[i], &T_j[i], &v_max[i], &a_max[i], &j_max[i]);
    }

	// calculate samples
	int samples = (int)(T * 1000.0 / SAMPLING_TIME);

	for (int i = 0; i < samples; i++) {
        double t = i * SAMPLING_TIME / 1000.0; // convert sampling time to seconds
        for (int j = 0; j < AXES; j++)
        {
            float dpos = cpos[j]
                         + calc_double_s_value(T, h[j], T_a[j], T_j[j], v_max[j], a_max[j], j_max[j], t);

            servo_set_angle(axis[j], dpos);
        }

        HAL_Delay(SAMPLING_TIME);
	}
}

// ----------------------------------------------------------------------------------------------------

void servo_controller_scara_init(servo_controller_scara_t *_servo_controller_scara,
						   	     servo_t *_axis_0, servo_t *_axis_1, servo_t *_axis_2, servo_t *_axis_3,
						   	   	 float _settling_time,
								 float _P_0, float _I_0, float _D_0,
							 	 float _P_1, float _I_1, float _D_1)
{
	_servo_controller_scara->axis_0 = _axis_0;
	_servo_controller_scara->axis_1 = _axis_1;
	_servo_controller_scara->axis_2 = _axis_2;
	_servo_controller_scara->axis_3 = _axis_3;

	servo_init(_axis_0, 2500, 500, 1500, -90, 90, 0, 0, 0);
	servo_init(_axis_1, 2500, 500, 1500, -90, 30, 90, 0, 1);
	servo_init(_axis_2, 500, 2500, 1550, -180, 180, 0, 0, 2);
	servo_init(_axis_3, 500, 2300, 1500, 90, -60, 0, 0, 3);

	_servo_controller_scara->settling_time = _settling_time;

	_servo_controller_scara->P_0 = _P_0;
	_servo_controller_scara->I_0 = _I_0;
	_servo_controller_scara->D_0 = _D_0;

	_servo_controller_scara->P_1 = _P_1;
	_servo_controller_scara->I_1 = _I_1;
	_servo_controller_scara->D_1 = _D_1;

	_servo_controller_scara->approach_dir[0] = PLAY_APPROACH_DEFAULT;
	_servo_controller_scara->approach_dir[1] = PLAY_APPROACH_DEFAULT;
	_servo_controller_scara->play_deg[0] = PLAY_BASE_DEG;
	_servo_controller_scara->play_deg[1] = PLAY_ELBOW_DEG;
	_servo_controller_scara->engaged[0] = 0;
	_servo_controller_scara->engaged[1] = 0;

	printf("servo controller initialised\n");
}

int servo_controller_scara_set_play(servo_controller_scara_t *_servo_controller_scara, int _axis,
									float _play_deg, int _dir)
{
	if ((_axis != 0 && _axis != 1) || _dir < -1 || _dir > 1 || !(_play_deg >= 0.0f && _play_deg <= 20.0f))
	{
		return 0;
	}
	_servo_controller_scara->approach_dir[_axis] = _dir;
	_servo_controller_scara->play_deg[_axis] = _play_deg;
	_servo_controller_scara->engaged[_axis] = 0;	// which side the servo is on is not known any more
	return 1;
}

void servo_controller_scara_reinit(servo_controller_scara_t *_servo_controller_scara,
								   float _settling_time,
								   float _P_0, float _I_0, float _D_0,
								   float _P_1, float _I_1, float _D_1)
{
	_servo_controller_scara->settling_time = _settling_time;

	_servo_controller_scara->P_0 = _P_0;
	_servo_controller_scara->I_0 = _I_0;
	_servo_controller_scara->D_0 = _D_0;

	_servo_controller_scara->P_1 = _P_1;
	_servo_controller_scara->I_1 = _I_1;
	_servo_controller_scara->D_1 = _D_1;
}

void servo_controller_scara_auto_calibrate_axis(servo_controller_scara_t *_servo_controller_scara, int _axis)
{
	servo_t *axis = _servo_controller_scara->axis_0;
	switch (_axis)
	{
		case 1:
			axis = _servo_controller_scara->axis_1;
			break;
		default:
			break;
	}

	float pulse_width_us = axis->zero_pulse_width;
	float pot_angle = potentiometers_read_angle(_axis);

	// reset to zero position: ramp there at the sweep's own pace (5 us per 5 ms) instead of
	// jumping. With no pulse on the pin yet (power-up), start from where the pot says the arm is.
	float ramp_us = servo_get_pulse_width_us(axis);
	if (ramp_us == 0)
	{
		ramp_us = servo_pulse_for_angle(axis, pot_angle);
	}
	while (fabsf(ramp_us - pulse_width_us) > 5)
	{
		ramp_us += (pulse_width_us > ramp_us) ? 5 : -5;
		servo_set_pulse_width_us(axis, ramp_us);
		HAL_Delay(5);
	}
	servo_set_pulse_width_us(axis, pulse_width_us);
	HAL_Delay(500);

	// check whether the axis is inverted
	// ==================== AXIS IS NOT INVERTED ====================
	if (axis->max_pulse_width >= axis->min_pulse_width)
	{
		// sweep from 0 to max pulse width (deadband = 4), stopping when max angle reached
		for (int i = axis->zero_pulse_width; i <= axis->max_pulse_width; i+=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			// check position using encoder
			pot_angle = potentiometers_read_angle(_axis);
			// if angle within 20% of target angle, move slower
			if (pot_angle >= (axis->max_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else // otherwise move fast
			{
				HAL_Delay(5);
			}

			// check if target angle is reached
			if (pot_angle >= (axis->max_angle + axis->offset_angle))
			{
				axis->calib_max_pulse_width = pulse_width_us;
				break;
			}
		}
		// sweep from max angle to min angle
		for (int i = pulse_width_us; i >= axis->min_pulse_width; i-=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			// check position using encoder
			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle <= (axis->min_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle <= (axis->min_angle + axis->offset_angle))
			{
				axis->calib_min_pulse_width = pulse_width_us;
				break;
			}
		}
		// sweep towards max pulse width again, but stopping at 0 degrees
		for (int i = pulse_width_us; i <= axis->max_pulse_width; i+=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle >= (axis->min_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle >= (0 + axis->offset_angle))
			{
				axis->calib_zero_pulse_width = pulse_width_us;
				break;
			}
		}
	}
	// ==================== AXIS IS INVERTED ====================
	else
	{
		// sweep towards max angle
		for (int i = axis->zero_pulse_width; i >= axis->max_pulse_width; i-=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle >= (axis->max_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle >= (axis->max_angle + axis->offset_angle))
			{
				axis->calib_max_pulse_width = pulse_width_us;
				break;
			}
		}
		// sweep from max angle to min angle
		for (int i = pulse_width_us; i <= axis->min_pulse_width; i+=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle <= (axis->min_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle <= (axis->min_angle + axis->offset_angle))
			{
				axis->calib_min_pulse_width = pulse_width_us;
				break;
			}
		}
		// return to 0
		for (int i = pulse_width_us; i >= axis->max_pulse_width; i-=5)
		{
			pulse_width_us = i;
			servo_set_pulse_width_us(axis, pulse_width_us);

			pot_angle = potentiometers_read_angle(_axis);
			if (pot_angle >= (axis->min_angle * 0.5) + axis->offset_angle)
			{
				HAL_Delay(50);
			}
			else
			{
				HAL_Delay(5);
			}

			if (pot_angle >= (0 + axis->offset_angle))
			{
				axis->calib_zero_pulse_width = pulse_width_us;
				break;
			}
		}
	}
	axis->cpos = pot_angle;
	_servo_controller_scara->engaged[_axis] = 0;	// the sweep ended going the other way
}

// Brings joints onto their targets from their potentiometers: see settle.h. The base and elbow only,
// whose pulses are inverted: a smaller pulse is a bigger angle.
/* _dirs (or NULL): per axis, +1 or -1 lets settling push that joint only that way (a step > 0 raises the angle
   of both inverted joints), 0 both ways. A joint already past its target that way can't be pulled back without
   crossing its play, so it is left and counted as done. Returns a bit per axis that ended past its target
   (used by move_j_scara/move_js_scara to turn the joint round once more; not sent to the host). */
static int settle_axes(servo_t **_axes, int *_channels, float *_targets, int _count, float _seconds, int *_dirs)
{
	float trim[2] = {0.0, 0.0};
	for (int j = 0; j < _count; j++)
	{
		trim[j] = servo_pulse_for_angle(_axes[j], _targets[j]) - servo_get_pulse_width_us(_axes[j]);
	}
	int calm = 0;
	int ticks = (int)(_seconds * 1000.0 / SETTLE_TICK_MS);

	for (int i = 0; i < ticks && calm < SETTLE_CALM_TICKS; i++)
	{
		int all_in_band = 1;
		for (int j = 0; j < _count; j++)
		{
			int dir = _dirs ? _dirs[j] : 0;
			float error = _targets[j] - potentiometers_read_angle(_channels[j]);
			int past = (dir != 0) && (error * dir < -SETTLE_BAND_DEG);
			if (fabsf(error) > SETTLE_BAND_DEG && !past)
			{
				all_in_band = 0;
			}
			float step = settle_step(error, trim[j]);
			if (dir != 0 && step * dir < 0.0f)
			{
				step = 0.0f;
			}
			/* one-way settling: short of the target, push at least SETTLE_MIN_DIR_STEP_US a tick, so a servo
			   still on the far side of the play crosses it in about half a second whatever the error (the
			   proportional step is tiny when the link is only a little short) */
			if (dir != 0 && error * dir > SETTLE_BAND_DEG && step * dir < SETTLE_MIN_DIR_STEP_US
			    && fabsf(trim[j] + dir * SETTLE_MIN_DIR_STEP_US) <= SETTLE_LIMIT_US)
			{
				step = dir * SETTLE_MIN_DIR_STEP_US;
			}
			trim[j] += step;
			servo_set_pulse_width_us(_axes[j], servo_get_pulse_width_us(_axes[j]) - step);
		}
		calm = all_in_band ? calm + 1 : 0;
		HAL_Delay(SETTLE_TICK_MS);
	}

	int past_mask = 0;
	for (int j = 0; j < _count; j++)
	{
		int dir = _dirs ? _dirs[j] : 0;
		if (dir != 0 && (_targets[j] - potentiometers_read_angle(_channels[j])) * dir < -SETTLE_BAND_DEG)
		{
			past_mask |= 1 << j;
		}
	}
	return past_mask;
}

// the shortest T for a double-S move of _h degrees whose peak speed and acceleration stay inside
// MOVE_MAX_SPEED_DPS and MOVE_MAX_ACCEL_DPS2 (peak v = h / ((1-a)T), peak acc = h / (a(1-a)(1-b)T^2))
static float min_move_time(float _h, float _alpha, float _beta)
{
	float h = fabsf(_h);
	float t_v = h / ((1.0f - _alpha) * MOVE_MAX_SPEED_DPS);
	float t_a = sqrtf(h / (_alpha * (1.0f - _alpha) * (1.0f - _beta) * MOVE_MAX_ACCEL_DPS2));
	return (t_v > t_a) ? t_v : t_a;
}

/* One profiled move of one joint, then settling for _settle_s seconds (0: none), pushing only _settle_dir
   (settle_axes). Returns settle_axes' mask: 1 if the joint ended past its target. */
static int move_j_run(servo_controller_scara_t *_servo_controller_scara, int _axis, float _angle, float _T,
					  float _settle_s, int _settle_dir)
{
    // assign axis
    servo_t *axis = _servo_controller_scara->axis_0;
    switch (_axis)
    {
        case 1:
            axis = _servo_controller_scara->axis_1;
            break;
        case 2:
            axis = _servo_controller_scara->axis_2;
            break;
        case 3:
			axis = _servo_controller_scara->axis_3;
			break;
        default:
            break;
    }

    // start where the joint is, and from the pulse already being sent (settle.h, carried_offset_us)
    float cpos = axis->cpos;
    float offset_us = 0.0;
	if (_axis == 0 || _axis == 1)
	{
		cpos = potentiometers_read_angle(_axis);
		if (fabsf(_angle - cpos) <= SETTLE_BAND_DEG) cpos = _angle;	/* review: already there, as settling counts it */
		if (servo_get_pulse_width_us(axis) > 0)	// no pulse yet (power-up): nothing to carry
		{
			offset_us = servo_pulse_for_angle(axis, cpos) - servo_get_pulse_width_us(axis);
		}
	}

	float tpos = _angle;        // target position in degrees
	float h = tpos - cpos;     	// displacement in degrees
	float T = _T;				// time to complete move
	float alpha = 0.3;         	// acceleration phase duration as fraction of T
	float beta = 0.3;          	// jerk phase duration as fraction of T_a

	// base and elbow: no faster than the servos can follow
	if ((_axis == 0 || _axis == 1) && T < min_move_time(h, alpha, beta))
	{
		T = min_move_time(h, alpha, beta);
	}

	// calculate double-S profile parameters
	float T_a, T_j, v_max, a_max, j_max;
	calc_double_s_parameters(T, h, alpha, beta, &T_a, &T_j, &v_max, &a_max, &j_max);

	// ============================== MOVE ==============================

	float P;
	float I;
	float D;
	if (_axis == 0)
	{
		P = _servo_controller_scara->P_0;
		I = _servo_controller_scara->I_0;
		D = _servo_controller_scara->D_0;
	}
	else
	{
		P = _servo_controller_scara->P_1;
		I = _servo_controller_scara->I_1;
		D = _servo_controller_scara->D_1;
	}

	float error_sum = 0.0;
	float last_error = 0.0;
	float max_integral = 20.0;

	// one step per PWM frame, in real time: the last step lands on t = T exactly
	int samples = (int)(T / SERVO_FRAME_S + 0.5f);

	for (int i = 1; i <= samples; i++)
	{
	    float t = (i == samples) ? T : i * SERVO_FRAME_S;
	    servo_wait_frame(axis);

	    // desired position using double S-curve profile
	    float dpos = cpos + calc_double_s_value(T, h, T_a, T_j, v_max, a_max, j_max, t);

	    // ----- control loop -----
	    if (_axis == 0 || _axis == 1)
	    {
	        // error calculation
	        float pot_read = potentiometers_read_angle(_axis);
	        float error = dpos - pot_read;
	        if (fabsf(error) <= SETTLE_BAND_DEG) error = 0.0f;	/* review: no PID push inside the band settling accepts */

	        // PID terms
	        float P_term_move = error * P;

	        // integral term (with anti-windup)
	        error_sum += error * SERVO_FRAME_S;
	        if (error_sum > max_integral)
	            error_sum = max_integral;
	        else if (error_sum < -max_integral)
	            error_sum = -max_integral;

	        float I_term_move = error_sum * I;

	        // derivative term
	        float D_term_move = ((error - last_error) / SERVO_FRAME_S) * D;
	        last_error = error;

	        // PID output
	        float pid_output = P_term_move + I_term_move + D_term_move;

	        // limit PID output to +-5 degrees
	        float adjusted_error;
	        if (pid_output > 5)
	        {
	            adjusted_error = 5;
	        }
	        else if (pid_output < -5)
	        {
	            adjusted_error = -5;
	        }
	        else
	        {
	            adjusted_error = pid_output;
	        }

	        // set adjusted position with PID correction, less what is left of the start offset
	        float commanded = dpos + adjusted_error;
	        float progress = (h != 0.0f) ? (dpos - cpos) / h : 1.0f;
	        servo_set_pulse_width_us(axis, servo_pulse_for_angle(axis, commanded)
	                                       - carried_offset_us(offset_us, progress, h));
	        axis->cpos = commanded;
	    }
	    // ------------------------
	    else
	    {
	        // no PID correction for other axes
	        servo_set_angle(axis, dpos);
	    }
	}

	// ============================== SETTLING ==============================

	// get angle before settling
	float angle_before_settling = 0.0;
	if (_axis == 0 || _axis == 1)
	{
		angle_before_settling = potentiometers_read_angle(_axis);
	}

	// settling time (seconds) to reduce backlash
	int past_mask = 0;
	if (_axis == 0 || _axis == 1)
	{
		servo_t *settling[1] = {axis};
		int channels[1] = {_axis};
		float targets[1] = {tpos};
		int dirs[1] = {_settle_dir};
		past_mask = settle_axes(settling, channels, targets, 1, _settle_s, dirs);
	}

	// set last angle adjustment from settling
	if (_axis == 0 || _axis == 1)
	{
		axis->last_angle_adjustment = angle_before_settling - potentiometers_read_angle(_axis);
	}
	return past_mask;
}

/* One profiled move of all four joints, then settling of the base and elbow for _settle_s seconds (0: none),
   each pushing only its _settle_dirs way. Returns settle_axes' mask (bit 0 base, bit 1 elbow past target). */
static int move_js_run(servo_controller_scara_t *_servo_controller_scara,
					   float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T,
					   float _settle_s, int *_settle_dirs)
{
    servo_t *axis[AXES] = {_servo_controller_scara->axis_0,
						   _servo_controller_scara->axis_1,
						   _servo_controller_scara->axis_2,
						   _servo_controller_scara->axis_3};
	// input variables
    float cpos[AXES];
    float tpos[AXES] = {_angle_0, _angle_1, _angle_2, _angle_3};
    float offset_us[AXES] = {0.0, 0.0, 0.0, 0.0};	// the base's and elbow's start offsets

    float h[AXES];     	        // displacement in degrees
	float T = _T;				// time to complete move
	float alpha = 0.3;         	// acceleration phase duration as fraction of T
	float beta = 0.3;          	// jerk phase duration as fraction of T_a

    // calculate double-S profile parameters
	float T_a[AXES], T_j[AXES], v_max[AXES], a_max[AXES], j_max[AXES];

    for (int i = 0; i < AXES; i++)
    {
    	cpos[i] = axis[i]->cpos;

    	if (i == 0 || i == 1)
    	{
    		// start where the joint is, and from the pulse already being sent (settle.h, carried_offset_us)
    		cpos[i] = potentiometers_read_angle(i);
    		if (fabsf(tpos[i] - cpos[i]) <= SETTLE_BAND_DEG) cpos[i] = tpos[i];	/* review: already there */
    		if (servo_get_pulse_width_us(axis[i]) > 0)	// no pulse yet (power-up): nothing to carry
    		{
    			offset_us[i] = servo_pulse_for_angle(axis[i], cpos[i]) - servo_get_pulse_width_us(axis[i]);
    		}
    	}

        h[i] = tpos[i] - cpos[i];

        // base and elbow: no faster than the servos can follow (all four axes keep one T, so stay in step)
        if ((i == 0 || i == 1) && T < min_move_time(h[i], alpha, beta))
        {
        	T = min_move_time(h[i], alpha, beta);
        }
    }

    for (int i = 0; i < AXES; i++)
    {
    	calc_double_s_parameters(T, h[i], alpha, beta, &T_a[i], &T_j[i], &v_max[i], &a_max[i], &j_max[i]);
    }

	// ============================== MOVE ==============================

    // PID for axis 0 and 1
    float P[2] = {_servo_controller_scara->P_0, _servo_controller_scara->P_1};
	float I[2] = {_servo_controller_scara->I_0, _servo_controller_scara->I_1};
	float D[2] = {_servo_controller_scara->D_0, _servo_controller_scara->D_1};

	float error_sum[2] = {0.0, 0.0};
	float last_error[2] = {0.0, 0.0};
	float max_integral[2] = {20.0, 20.0};

	// one step per PWM frame, in real time: the last step lands on t = T exactly
	int samples = (int)(T / SERVO_FRAME_S + 0.5f);

	for (int i = 1; i <= samples; i++) {
        float t = (i == samples) ? T : i * SERVO_FRAME_S;
        servo_wait_frame(axis[0]);	// all four servos share TIM3
        for (int j = 0; j < AXES; j++)
        {
            float dpos = cpos[j]
                         + calc_double_s_value(T, h[j], T_a[j], T_j[j], v_max[j], a_max[j], j_max[j], t);

            // ----- control loop -----
            if (j == 0 || j == 1)
            {
            	// error calculation
				float pot_read = potentiometers_read_angle(j);
				float error = dpos - pot_read;
	        if (fabsf(error) <= SETTLE_BAND_DEG) error = 0.0f;	/* review: no PID push inside the band settling accepts */

				// PID terms
				float P_term_move = error * P[j];

				// integral term (with anti-windup)
				error_sum[j] += error * SERVO_FRAME_S;
				if (error_sum[j] > max_integral[j])
					error_sum[j] = max_integral[j];
				else if (error_sum[j] < -max_integral[j])
					error_sum[j] = -max_integral[j];

				float I_term_move = error_sum[j] * I[j];

				// derivative term
				float D_term_move = ((error - last_error[j]) / SERVO_FRAME_S) * D[j];
				last_error[j] = error;

				// PID output
				float pid_output = P_term_move + I_term_move + D_term_move;

				// limit PID output to +-5 degrees
				float adjusted_error;
				if (pid_output > 5)
				{
					adjusted_error = 5;
				}
				else if (pid_output < -5)
				{
					adjusted_error = -5;
				}
				else
				{
					adjusted_error = pid_output;
				}

				// set adjusted position with PID correction, less what is left of the start offset
				float commanded = dpos + adjusted_error;
				float progress = (h[j] != 0.0f) ? (dpos - cpos[j]) / h[j] : 1.0f;
				servo_set_pulse_width_us(axis[j], servo_pulse_for_angle(axis[j], commanded)
				                                  - carried_offset_us(offset_us[j], progress, h[j]));
				axis[j]->cpos = commanded;
            }
            // ------------------------
            else
            {
            	servo_set_angle(axis[j], dpos);
            }
        }

	}

	// ============================== SETTLING ==============================

	// get angle before settling
	float angle_before_settling[2] = {0.0, 0.0};
	angle_before_settling[0] = potentiometers_read_angle(0);
	angle_before_settling[1] = potentiometers_read_angle(1);

	// settling time (seconds) to reduce backlash
	int past_mask;
	{
		servo_t *settling[2] = {axis[0], axis[1]};
		int channels[2] = {0, 1};
		float targets[2] = {tpos[0], tpos[1]};
		past_mask = settle_axes(settling, channels, targets, 2, _settle_s, _settle_dirs);
	}

	// set last angle adjustment from settling
	axis[0]->last_angle_adjustment = angle_before_settling[0] - potentiometers_read_angle(0);
	axis[1]->last_angle_adjustment = angle_before_settling[1] - potentiometers_read_angle(1);
	return past_mask;
}

// ---------------------------------------- moves with the play taken up ----------------------------------------

// where a joint turns round to come back onto _target the approach way: past it by the play and a margin
static float approach_start(servo_controller_scara_t *_c, int _axis, float _target)
{
	float start = _target - _c->approach_dir[_axis] * (_c->play_deg[_axis] + APPROACH_MARGIN_DEG);
	float lo = (_axis == 0) ? APPROACH_BASE_MIN_DEG : APPROACH_ELBOW_MIN_DEG;
	float hi = (_axis == 0) ? APPROACH_BASE_MAX_DEG : APPROACH_ELBOW_MAX_DEG;
	return (start < lo) ? lo : (start > hi) ? hi : start;
}

// whether a turn round from _start may not cross the play (approach_start cut it short at a limit), so which
// side of the play the servo ends on is not known
static int short_turn(servo_controller_scara_t *_c, int _axis, float _target, float _start)
{
	return fabsf(_target - _start) < _c->play_deg[_axis] + APPROACH_MARGIN_DEG - 0.01f;
}

/* Whether a joint has to turn round before _target: it would end the move going the wrong way, or (when it
   isn't known which side of the play the servo is on) it wouldn't travel far enough the right way to take the
   play up on the way. */
static int needs_turn(servo_controller_scara_t *_c, int _axis, float _target)
{
	int dir = _c->approach_dir[_axis];
	if (dir == 0)
	{
		return 0;
	}
	float travel = (_target - potentiometers_read_angle(_axis)) * dir;
	if (_c->engaged[_axis])
	{
		return travel < -SETTLE_BAND_DEG;
	}
	return travel < _c->play_deg[_axis] + APPROACH_MARGIN_DEG;
}

void move_j_scara(servo_controller_scara_t *_servo_controller_scara, int _axis, float _angle, float _T)
{
	servo_controller_scara_t *c = _servo_controller_scara;
	if ((_axis != 0 && _axis != 1) || c->approach_dir[_axis] == 0)
	{
		move_j_run(c, _axis, _angle, _T, c->settling_time, 0);
		if (_axis == 0 || _axis == 1)
		{
			c->engaged[_axis] = 0;
		}
		return;
	}

	int dir = c->approach_dir[_axis];
	float start = approach_start(c, _axis, _angle);
	int turned = needs_turn(c, _axis, _angle);
	int past;
	if (turned)
	{
		move_j_run(c, _axis, start, _T, 0.0f, 0);
		past = move_j_run(c, _axis, _angle, APPROACH_S, c->settling_time, dir);
	}
	else
	{
		past = move_j_run(c, _axis, _angle, _T, c->settling_time, dir);
	}

	for (int retry = 0; past && retry < APPROACH_RETRIES; retry++)
	{
		turned = 1;
		move_j_run(c, _axis, start, APPROACH_S, 0.0f, 0);
		past = move_j_run(c, _axis, _angle, APPROACH_S, c->settling_time, dir);
	}
	c->engaged[_axis] = !(turned && short_turn(c, _axis, _angle, start));
}

void move_js_scara(servo_controller_scara_t *_servo_controller_scara,
		     	   float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T)
{
	servo_controller_scara_t *c = _servo_controller_scara;
	float target[2] = {_angle_0, _angle_1};
	int dirs[2] = {c->approach_dir[0], c->approach_dir[1]};

	// the base or elbow that has to turn round goes to its turning point first, with the other joints going
	// straight to their targets; then both come onto their targets together, the approach way
	float first[2] = {target[0], target[1]};
	int turned[2] = {0, 0};
	int turning = 0;
	for (int j = 0; j < 2; j++)
	{
		if (needs_turn(c, j, target[j]))
		{
			first[j] = approach_start(c, j, target[j]);
			turned[j] = 1;
			turning = 1;
		}
	}

	int past;
	if (turning)
	{
		move_js_run(c, first[0], first[1], _angle_2, _angle_3, _T, 0.0f, NULL);
		past = move_js_run(c, target[0], target[1], _angle_2, _angle_3, APPROACH_S, c->settling_time, dirs);
	}
	else
	{
		past = move_js_run(c, target[0], target[1], _angle_2, _angle_3, _T, c->settling_time, dirs);
	}

	for (int retry = 0; past && retry < APPROACH_RETRIES; retry++)
	{
		for (int j = 0; j < 2; j++)
		{
			first[j] = target[j];
			if (past & (1 << j))
			{
				first[j] = approach_start(c, j, target[j]);
				turned[j] = 1;
			}
		}
		move_js_run(c, first[0], first[1], _angle_2, _angle_3, APPROACH_S, 0.0f, NULL);
		past = move_js_run(c, target[0], target[1], _angle_2, _angle_3, APPROACH_S, c->settling_time, dirs);
	}
	for (int j = 0; j < 2; j++)
	{
		c->engaged[j] = (dirs[j] != 0)
		                && !(turned[j] && short_turn(c, j, target[j], approach_start(c, j, target[j])));
	}
}

// calculations
// ----------------------------------------------------------------------------------------------------

void calc_double_s_parameters(float T, float h, float alpha, float beta,
							  float *T_a, float *T_j,
							  float *v_max, float *a_max, float *j_max)
{
    // accel / decel duration    0 < alpha < 1/2
    *T_a = alpha * T;
    // jerk duration             0 < beta < 1/2
    *T_j = beta * (*T_a);
    // calculate maximum velocity
    *v_max = h / ((1 - alpha) * T);
    // calculate maximum acceleration
    *a_max = h / (alpha * (1 - alpha) * (1 - beta) * T * T);
    // calculate maximum jerk
    *j_max = h / (alpha * alpha * beta * (1 - alpha) * (1 - beta) * T * T * T);
}


float calc_double_s_value(float T, float h, float T_a, float T_j,
						  float v_max, float a_max, float j_max,
						  float t)
{
    // acceleration phase
    if (t >= 0 && t <= T_j)
    {
        return j_max * (pow(t, 3) / 6.0);
    }
    else if (t > T_j && t <= T_a - T_j)
    {
    	return (a_max / 6.0) * ((3 * pow(t, 2)) - (3 * T_j * t) + pow(T_j, 2));
    }
    else if (t > T_a - T_j && t <= T_a)
    {
    	return (v_max * (T_a / 2.0)) - (v_max * (T_a - t)) - (-j_max * (pow(T_a - t, 3) / 6.0));
    }
    // constant velocity phase
    else if (t > T_a && t <= T - T_a)
    {
    	return v_max * (T_a / 2.0) + (v_max * (t - T_a));
    }
    // deceleration phase
    else if (t > T - T_a && t <= T - T_a + T_j)
    {
    	return h - (v_max * (T_a / 2.0)) + v_max * (t - T + T_a) - (j_max * pow(t - T + T_a, 3) / 6.0);
    }
    else if (t > T - T_a + T_j && t <= T - T_j)
    {
    	return h - (v_max * (T_a / 2.0)) + v_max * (t - T + T_a)
               - ((a_max / 6.0) * (3 * pow(t - T + T_a, 2) - 3 * T_j * (t - T + T_a) + pow(T_j, 2)));
    }
    else if (t > T - T_j && t <= T)
    {
    	return h - (j_max * (pow(T - t, 3) / 6.0));
    }
    else
    {
    	return 999;
    }
}

// ----------------------------------------------------------------------------------------------------
