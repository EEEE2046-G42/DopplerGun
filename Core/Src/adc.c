/*
 * adc.c
 *
 *  Created on: Dec 14, 2021
 *      Author: David
 */

#include "adc.h"

// Calibrates ADC
void ADC_Calibrate(ADC_HandleTypeDef *adc)
{
	HAL_ADCEx_Calibration_Start(adc, ADC_SINGLE_ENDED);
}

// Starts a measurement
void ADC_Measure(ADC_HandleTypeDef *adc)
{
	HAL_ADC_Start_DMA(adc, ADC_BUFFER, ADC_BUFFER_LENGTH);
}

// Handles a full DMA buffer
float ADC_HandleBufferFull(ADC_HandleTypeDef *adc, DMA_HandleTypeDef *dma)
{
	return 0 /*FFT(ADC_BUFFER)*/;
}
