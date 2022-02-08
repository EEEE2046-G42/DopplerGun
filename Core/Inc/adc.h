/*
 * adc.h
 *
 *  Created on: Dec 14, 2021
 *      Author: David
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#define ADC_BUFFER_LENGTH 1024
#define SAMPLE_FREQ 17857

#include "stdint.h"
#include "stdbool.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_dma.h"

// Define array in which to store ADC readings
uint16_t ADC_BUFFER[ADC_BUFFER_LENGTH];

void ADC_Calibrate(ADC_HandleTypeDef *adc);										// Calibrates ADC
void ADC_Measure(ADC_HandleTypeDef *adc);										// Starts a measurement
float ADC_HandleBufferFull(ADC_HandleTypeDef *adc, DMA_HandleTypeDef *dma);	// Handles a full DMA buffer

#endif /* INC_ADC_H_ */
