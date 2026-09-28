/*
 * potentiometers.c
 *
 *  Created on: Mar 27, 2025
 *      Author: Alex Clark
 */

#include "potentiometers.h"

uint16_t pot_raw_value[POTS];
potentiometer_t pots[POTS];

void potentiometers_init()
{
    HAL_ADC_Start_DMA(&hadc1, pot_raw_value, POTS);

    // enter calibration values here...
    pots[0].min_angle = -90.0;
    pots[0].max_angle = 90.0;
    pots[0].min_raw_value = 647.0;
    pots[0].max_raw_value = 3253.0;

    pots[1].min_angle = 0.0;
    pots[1].max_angle = 90.0;
    pots[1].min_raw_value = 717.0;
    pots[1].max_raw_value = 2117.0;
}

float potentiometers_read_raw_value(int _channel)
{
    // range 0-4095
    return pot_raw_value[_channel];
}

float potentiometers_read_angle(int _channel)
{
    float angle = pots[_channel].min_angle
                + ((pot_raw_value[_channel] - pots[_channel].min_raw_value)
                * (pots[_channel].max_angle - pots[_channel].min_angle))
                / (pots[_channel].max_raw_value - pots[_channel].min_raw_value);
    return angle;
}

