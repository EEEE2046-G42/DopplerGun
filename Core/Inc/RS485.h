/*
 * RS485.h
 *
 *  Created on: Feb 3, 2022
 *      Author: David
 */

#ifndef SRC_RS485_H_
#define SRC_RS485_H_

#include "stdint.h"
#include "stm32l4xx_hal_uart.h"

uint8_t toBCD(const uint8_t in);
void transmit(UART_HandleTypeDef* uartHandle, const uint8_t in);

uint8_t toBCD(const uint8_t in)
{
	// Ensure input is < 100
	uint8_t constrained = in % 100;

	// Split input into constituent digits and set output
	uint8_t lower = constrained % 10;
	uint8_t upper = constrained / 10;

	return ((upper & 0x0F) << 4) + (lower & 0x0F);
}

void transmit(UART_HandleTypeDef* uartHandle, const uint8_t in)
{
	// Create buffer
	uint8_t buf = toBCD(in);

	HAL_UART_Transmit(uartHandle, buf, 1, 10);
}

#endif /* SRC_RS485_H_ */
