/*
 * fft.c
 *
 *  Created on: Dec 14, 2021
 *      Author: David
 */

#include "fft.h"

float getLargestFreq(uint16_t ADCoutput[])
{
	// Define necessary arrays
	float32_t complexFFT[FFT_SAMPLES], realFFT[FFT_SAMPLES_HALF],
			imagFFT[FFT_SAMPLES_HALF], angleFFT[FFT_SAMPLES_HALF],
			powerFFT[FFT_SAMPLES_HALF];

	// Define variables
	uint32_t ifftFlag = 0;
	arm_rfft_fast_instance_f32 S;
	uint32_t maxIndex = 0;
	float32_t maxValue;

	// Convert uint16_t to float
	float32_t input[FFT_SAMPLES];
	for (uint16_t i = 0; i < FFT_SAMPLES; i++)
		input[i] = (float32_t)input[i];

	// Initialise converter
	arm_status status = arm_rfft_fast_init_f32(&S, FFT_SAMPLES);
	//arm_status status = arm_rfft_1024_fast_init_f32(&S);
	// Run conversion
	arm_rfft_fast_f32(&S, input, complexFFT, ifftFlag);


	// compute magnitudes
	arm_cmplx_mag_squared_f32(complexFFT, powerFFT, FFT_SAMPLES_HALF);

	// find biggest frequency
	float32_t largest = 0;
	float index = 0;
	// Start at 1 (ignore DC)
	for (uint16_t i = 1; i < FFT_SAMPLES_HALF; i++)
		if (powerFFT[i] > largest)
			{
				largest = powerFFT[i];
				index = i;
			}

	// Convert index to frequency
	return index * SAMPLE_FREQ / FFT_SAMPLES;
}

float getSpeed(uint16_t ADCoutput[])
{
	// Get speed with FFT
	float f = getLargestFreq(ADCoutput);

	// Convert this to velocity
	return f * (float)300000000 / (2.0 * (float)10587000000);
}
