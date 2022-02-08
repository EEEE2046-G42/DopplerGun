/*
 * FFT.h
 *
 *  Created on: 14 Dec 2021
 *      Author: eeycm4
 */

#ifndef INC_FFT_H_
#define INC_FFT_H_

#include "math.h"
#include "stdint.h"
#include "stm32l4xx_hal.h"

// Include fft library
#include "arm_math.h"

#define FFT_SAMPLES 1024
#define FFT_SAMPLES_HALF (FFT_SAMPLES / 2)
#define SAMPLE_FREQ 17857

float getLargestFreq(uint16_t ADCoutput[]);

float getSpeed(uint16_t ADCoutput[]);

#endif /* INC_FFT_H_ */
