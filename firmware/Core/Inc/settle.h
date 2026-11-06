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

#endif /* INC_SETTLE_H_ */
