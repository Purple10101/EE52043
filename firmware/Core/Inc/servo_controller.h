/*
 * servo_controller.h
 *
 *  Created on: Feb 2, 2025
 *      Author: asus
 */

#ifndef INC_SERVO_CONTROLLER_H_
#define INC_SERVO_CONTROLLER_H_

#include <stdio.h>
#include <math.h>

#include "servo.h"

#define AXES 4
#define SAMPLING_TIME 1 	// milliseconds
#define SAMPLING_TIME_SETTLING 50

// base/elbow speed limits: a move's T is raised so its double-S peak speed and acceleration stay inside them.
// Guessed values, not measured on the servos.
#define MOVE_MAX_SPEED_DPS 150.0f
#define MOVE_MAX_ACCEL_DPS2 1000.0f

/*
 * Joint play: approach every base/elbow target from one side (firmware/README.md, "The play build").
 * approach_dir +1: the last motion onto a target is always upward in angle, so the servo ends pressed against
 * the same side of the play every time; -1 the other way; 0 off (the standard build). A move that would
 * end going the wrong way first goes past the target by the joint's play plus APPROACH_MARGIN_DEG, then comes
 * back onto it over APPROACH_S, and settling only pushes in the approach direction. The play build is compiled
 * with -DPLAY_APPROACH_DEFAULT=1; SCARA_SET_PLAY (command 8) changes it at run time in either build.
 */
#ifndef PLAY_APPROACH_DEFAULT
#define PLAY_APPROACH_DEFAULT 0
#endif
#define PLAY_BASE_DEG 4.5f				// measured 2026-11-12 (pot_check, the user pushing the link under hold)
#define PLAY_ELBOW_DEG 1.1f
#define APPROACH_MARGIN_DEG 1.0f		// past the play, so the servo is surely on the far side of it
#define APPROACH_S 0.4f					// the short move back onto the target
#define APPROACH_RETRIES 1				// a joint that overshoots its target is brought back round this many times
#define SETTLE_MIN_DIR_STEP_US 2.0f		// one-way settling pushes at least this a tick while short: crosses 4.5 deg of play in ~0.5 s
#define APPROACH_BASE_MIN_DEG -90.0f	// the turn-round point is kept inside these
#define APPROACH_BASE_MAX_DEG 67.5f
#define APPROACH_ELBOW_MIN_DEG 0.0f
#define APPROACH_ELBOW_MAX_DEG 139.0f

typedef struct {
	servo_t *axis_0;
	servo_t *axis_1;
	servo_t *axis_2;
	servo_t *axis_3;
} servo_controller_t;

typedef struct {
	servo_t *axis_0;
	servo_t *axis_1;
	servo_t *axis_2;
	servo_t *axis_3;
	float settling_time;
	float P_0; float I_0; float D_0;
	float P_1; float I_1; float D_1;
	int approach_dir[2];		// base, elbow: +1, -1 or 0 (off)
	float play_deg[2];
	int engaged[2];				// 1 once a move has ended going the approach way; 0 after power-up or calibrate
} servo_controller_scara_t;

void servo_controller_init(servo_controller_t *_servo_controller,
						   servo_t *_axis_0, servo_t *_axis_1, servo_t *_axis_2, servo_t *_axis_3);

void move_j(servo_controller_t *_servo_controller, int _axis, float _angle, float _T);
void move_js(servo_controller_t *_servo_controller,
			 float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T);

void servo_controller_scara_init(servo_controller_scara_t *_servo_controller_scara,
						   	     servo_t *_axis_0, servo_t *_axis_1, servo_t *_axis_2, servo_t *_axis_3,
						   	   	 float _settling_time,
								 float _P_0, float _I_0, float _D_0,
							 	 float _P_1, float _I_1, float _D_1);

void servo_controller_scara_reinit(servo_controller_scara_t *_servo_controller_scara,
								   float _settling_time,
								   float _P_0, float _I_0, float _D_0,
								   float _P_1, float _I_1, float _D_1);

void servo_controller_scara_auto_calibrate_axis(servo_controller_scara_t *_servo_controller_scara, int _axis);

// SCARA_SET_PLAY: _axis 0 (base) or 1 (elbow); _dir +1, -1 or 0 (off). Returns 0 for a bad axis or direction.
int servo_controller_scara_set_play(servo_controller_scara_t *_servo_controller_scara, int _axis,
									float _play_deg, int _dir);

void move_j_scara(servo_controller_scara_t *_servo_controller_scara, int _axis, float _angle, float _T);
void move_js_scara(servo_controller_scara_t *_servo_controller_scara,
		     	   float _angle_0, float _angle_1, float _angle_2, float _angle_3, float _T);

#endif /* INC_SERVO_CONTROLLER_H_ */
