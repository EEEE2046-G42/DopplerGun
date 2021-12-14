/*
 * adc.h
 *
 *  Created on: Dec 14, 2021
 *      Author: David
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#define ADC_BUFFER_LENGTH 1024

#include "stdint.h"

// Define array in which to store ADC readings
uint16_t ADC_BUFFER[ADC_BUFFER_LENGTH];

// Handles a full DMA buffer
double ADC_HandleBufferFull();

#endif /* INC_ADC_H_ */
