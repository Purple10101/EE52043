/*
 * settle_test.c
 *
 * settle_step on a PC, against a model of a hobby servo on a joint: a deadband, gear backlash, a noisy
 * potentiometer reading and a pulse resolution. Compares it with the course's settling step.
 *
 *     gcc -I../Core/Inc settle_test.c ../Core/Src/settle.c -lm -o settle_test && ./settle_test
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "settle.h"

#define US_PER_DEG 11.56f
#define DEADBAND_US 4.0f
#define BACKLASH_DEG 0.6f
#define RUNS 2000

typedef struct {
	float servo_us;		/* where the servo is holding, us */
	float joint_deg;	/* the joint, after the backlash */
} model_t;

static float noise(float amplitude)
{
	return amplitude * (2.0f * rand() / (float)RAND_MAX - 1.0f);
}

/* The pulse is inverted, as on the base and elbow: a smaller pulse is a bigger angle. */
static void model_step(model_t *m, float pulse_us)
{
	if (fabsf(pulse_us - m->servo_us) > DEADBAND_US)
	{
		m->servo_us = pulse_us;
	}
	float wants = (1500.0f - m->servo_us) / US_PER_DEG;
	if (wants - m->joint_deg > BACKLASH_DEG / 2)
	{
		m->joint_deg = wants - BACKLASH_DEG / 2;
	}
	else if (m->joint_deg - wants > BACKLASH_DEG / 2)
	{
		m->joint_deg = wants + BACKLASH_DEG / 2;
	}
}

static float quantise(float us, float tick_us, int truncate)
{
	return truncate ? floorf(us / tick_us) * tick_us : roundf(us / tick_us) * tick_us;
}

/* Returns the final error, deg. new_way: settle_step at 0.1 us with a filtered reading; else the course. */
static float run(int new_way, float start_error, float target, int *ticks_used)
{
	model_t m;
	float pulse = 1500.0f - (target - start_error) * US_PER_DEG;
	m.servo_us = pulse;
	m.joint_deg = target - start_error;
	float trim = 0.0f;
	int calm = 0;
	int ticks = 0;
	int limit = new_way ? 1000 / SETTLE_TICK_MS : 1000 / 50;

	for (; ticks < limit; ticks++)
	{
		float raw_noise = new_way ? 0.03f : 0.5f;
		float error = target - (m.joint_deg + noise(raw_noise));
		if (new_way)
		{
			float step = settle_step(error, trim);
			trim += step;
			pulse = quantise(pulse - step, 0.1f, 0);
			calm = fabsf(error) <= SETTLE_BAND_DEG ? calm + 1 : 0;
			model_step(&m, pulse);
			if (calm >= SETTLE_CALM_TICKS)
			{
				break;
			}
		}
		else
		{
			if (error < -0.5f)
			{
				pulse = quantise(pulse + 5.0f, 3.0f, 1);
			}
			else if (error > 0.5f)
			{
				pulse = quantise(pulse - 5.0f, 3.0f, 1);
			}
			model_step(&m, pulse);
		}
	}
	*ticks_used = ticks;
	return target - m.joint_deg;
}

static void report(const char *name, int new_way)
{
	double sum = 0.0;
	double worst = 0.0;
	double time_ms = 0.0;
	srand(1);
	for (int i = 0; i < RUNS; i++)
	{
		int ticks;
		float e = run(new_way, noise(3.0f), noise(60.0f), &ticks);
		sum += e * e;
		worst = fabs(e) > worst ? fabs(e) : worst;
		time_ms += ticks * (new_way ? SETTLE_TICK_MS : 50);
	}
	printf("%-8s rms %.3f deg, worst %.3f deg, mean settling %.0f ms\n", name, sqrt(sum / RUNS), worst, time_ms / RUNS);
}

/* A move's start offset (carried_offset_us): the first pulse of a move is the one already being sent, the offset
   fades as the joint travels, fully over a long move and in part over a short one, and never jumps. */
static int check(const char *what, int ok)
{
	printf("%s %s\n", ok ? "ok  " : "FAIL", what);
	return ok ? 0 : 1;
}

static int carry_checks(void)
{
	int failed = 0;
	failed += check("a move starts with all of its offset: no kick",
	                fabsf(carried_offset_us(90.0f, 0.0f, 120.0f) - 90.0f) < 1e-4f);
	failed += check("a long move ends with none of it, as before",
	                fabsf(carried_offset_us(90.0f, 1.0f, 120.0f)) < 1e-4f);
	failed += check("so does a long move the other way",
	                fabsf(carried_offset_us(-90.0f, 1.0f, -40.0f)) < 1e-4f);
	failed += check("a 5 deg move keeps three quarters at its end",
	                fabsf(carried_offset_us(80.0f, 1.0f, 5.0f) - 60.0f) < 1e-3f);
	failed += check("a joint that doesn't move keeps all of it",
	                fabsf(carried_offset_us(80.0f, 1.0f, 0.0f) - 80.0f) < 1e-4f);
	failed += check("a wild offset still starts whole: no step at the start",
	                fabsf(carried_offset_us(900.0f, 0.0f, 50.0f) - 900.0f) < 1e-4f);
	failed += check("but what a move keeps at its end is limited",
	                fabsf(carried_offset_us(900.0f, 1.0f, 0.0f) - CARRY_LIMIT_US) < 1e-4f);
	failed += check("in both directions",
	                fabsf(carried_offset_us(-900.0f, 1.0f, 0.0f) + CARRY_LIMIT_US) < 1e-4f);
	failed += check("settling stops a trim at its limit",
	                settle_step(5.0f, SETTLE_LIMIT_US) == 0.0f && settle_step(-5.0f, -SETTLE_LIMIT_US) == 0.0f);
	failed += check("and never steps a trim beyond it back by the excess",
	                settle_step(5.0f, 260.0f) == 0.0f
	                && fabsf(settle_step(-5.0f, 260.0f) + SETTLE_MAX_STEP_US) < 1e-4f
	                && fabsf(settle_step(5.0f, -260.0f) - SETTLE_MAX_STEP_US) < 1e-4f);
	failed += check("but still limits a step that would cross it",
	                fabsf(settle_step(5.0f, 199.0f) - 1.0f) < 1e-4f);
	float last = carried_offset_us(150.0f, 0.0f, 60.0f), biggest = 0.0f;
	for (int i = 1; i <= 1000; i++)
	{
		float now = carried_offset_us(150.0f, i / 1000.0f, 60.0f);
		biggest = fmaxf(biggest, fabsf(now - last));
		last = now;
	}
	printf("     the largest change in one step of a 1000-step move: %.3f us\n", biggest);
	failed += check("it fades smoothly (no step over 0.2 us in 1000)", biggest < 0.2f);
	return failed;
}

int main(void)
{
	report("course", 0);
	report("new", 1);

	/* A joint that can't move: the trim stops at its limit. */
	float trim = 0.0f;
	for (int i = 0; i < 100000; i++)
	{
		trim += settle_step(30.0f, trim);
	}
	printf("held against a stop: trim ends at %.1f us (limit %.1f)\n", trim, SETTLE_LIMIT_US);
	int failed = fabsf(trim - SETTLE_LIMIT_US) < 0.01f ? 0 : 1;
	return failed + carry_checks();
}
