/*
 * fft.c
 *
 *  Created on: Dec 14, 2021
 *      Author: David
 */

#include "fft.h"

double FFT(uint16_t ADCoutput[])
{
	//double Frequencystrengths[FFT_NUM_FREQ];
	double Strongestfrequency = 0;
	double Speedmps;

	for (uint16_t k = 0; k < FFT_NUM_FREQ; k++)	{
		double Xk = 0;
		for (uint16_t n = 0; n < FFT_NUM_FREQ; n++)
		{
			Xk += (ADCoutput[2*n]*exp((-2*M_PI*n*k)/(FFT_NUM_FREQ * 0.5)) + exp((-2*M_PI*k)/FFT_NUM_FREQ)*ADCoutput[2*n+1]*exp((-2*M_PI*n*k)/(FFT_NUM_FREQ * 0.5)));
		}

		//Frequencystrengths[k] = Xk;

		// Ignore DC component
		if (Xk > Strongestfrequency && k != 0)
			Strongestfrequency = Xk;
	}
	//printf("strongest frequency is %lf", &Strongestfrequency);
	Speedmps = (Strongestfrequency*299792458)/(2*10587000000);

	return Speedmps;
}
