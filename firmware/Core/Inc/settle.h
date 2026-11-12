/*
 * settle.h
 *
 * Bringing a joint onto its target after a move, from its potentiometer. Replaces the course's
 * settling step, which added a fixed 5 us while the raw reading was more than 0.5 deg off, without limit.
 *
 * Each tick the pulse moves in proportion to the error, so the joint comes in gently and doesn't step
 * past the target. Inside SETTLE_BAND_DEG nothing moves: the servo's own deadband is a few us, so a
 * smaller band would hunt. The total is limited, so a joint held against something isn't pushed harder
 * and harder, and the joint is left alone as soon as it has stayed in the band for SETTLE_CALM_TICKS.
 */

#ifndef INC_SETTLE_H_
#define INC_SETTLE_H_

#define SETTLE_TICK_MS 20
#define SETTLE_BAND_DEG 0.15f
#define SETTLE_GAIN_US_PER_DEG 1.5f		// per tick: about 13 us/deg, so 150 ms to take an error down to a third
#define SETTLE_MAX_STEP_US 3.0f
#define SETTLE_LIMIT_US 200.0f			// the base needed about 90 us at +60 deg
#define SETTLE_CALM_TICKS 5

/* The change in trim, us, for this tick: error_deg is target less reading, trim_us the total so far. */
float settle_step(float error_deg, float trim_us);

/*
 * Starting a move without a kick (2026-11-11). After a move the pulse being sent is not the one the joint's angle
 * maps to: settling has trimmed it, and the PID's last correction is in it. The course's code started the next
 * move from the mapped pulse (plus half the last settling change), so the pulse jumped by the difference in the
 * first millisecond: up to about 8 deg on the base. Now the move starts from the joint's measured angle with that
 * difference, the start offset, still applied, so its first pulse is the pulse already being sent, and the offset
 * fades out as the joint travels: fully over a move of CARRY_FADE_DEG or more, which then ends as before, and only
 * in part over a shorter one, which stays near where the offset was needed.
 */
#define CARRY_FADE_DEG 20.0f
#define CARRY_LIMIT_US 120.0f			/* review: what a short move keeps; with the PID's 5 deg it stays inside SETTLE_LIMIT_US */

/* The part of a move's start offset (us) to subtract from the pulse now: progress 0 to 1 along a move of
   distance_deg. All of it at the start; at the end, the part a move this short keeps. */
float carried_offset_us(float offset_us, float progress, float distance_deg);

#endif /* INC_SETTLE_H_ */
