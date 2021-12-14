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
#include "fft.h"

#define FFT_NUM_FREQ 1024

double FFT(uint16_t ADCoutput[]);

#endif /* INC_FFT_H_ */
