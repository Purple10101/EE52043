/*
 * potentiometers.c
 *
 *  Created on: Mar 27, 2025
 *      Author: asus
 */

#include "potentiometers.h"

uint16_t pot_samples[POT_WINDOW * POTS];	// circular DMA, channels interleaved
potentiometer_t pots[POTS];

void potentiometers_init()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)pot_samples, POT_WINDOW * POTS);

    // default values for white robot
//    pots[0].min_angle = -90.0;
//    pots[0].max_angle = 90.0;
//    pots[0].min_raw_value = 700.0;
//    pots[0].max_raw_value = 3300.0;
//
//    pots[1].min_angle = 0.0;
//    pots[1].max_angle = 90.0;
//    pots[1].min_raw_value = 720.0;
//    pots[1].max_raw_value = 2090.0;

    // default values for blue robot
	pots[0].min_angle = -90.0;
	pots[0].max_angle = 90.0;
	pots[0].min_raw_value = 712.0;
	pots[0].max_raw_value = 3403.0;

	pots[1].min_angle = 0.0;
	pots[1].max_angle = 90.0;
	pots[1].min_raw_value = 662.0;
	pots[1].max_raw_value = 2017.0;
}

float potentiometers_read_raw_value(int _channel)
{
    // range 0-4095, the mean of the window
    uint32_t sum = 0;
    for (int i = _channel; i < POT_WINDOW * POTS; i += POTS)
    {
        sum += pot_samples[i];
    }
    return (float)sum / POT_WINDOW;
}

float potentiometers_read_angle(int _channel)
{
    float angle = pots[_channel].min_angle
                + ((potentiometers_read_raw_value(_channel) - pots[_channel].min_raw_value)
                * (pots[_channel].max_angle - pots[_channel].min_angle))
                / (pots[_channel].max_raw_value - pots[_channel].min_raw_value);
    return angle;
}

