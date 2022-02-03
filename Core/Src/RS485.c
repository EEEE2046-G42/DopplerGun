/*
 * RS485.c
 *
 *  Created on: Feb 3, 2022
 *      Author: David
 */

//#include "RS485.h"

//uint8_t toBCD(const uint8_t in)
//{
//	// Ensure input is < 100
//	uint8_t constrained = in % 100;
//
//	// Split input into constituent digits and set output
//	unit8_t lower = constrained % 10;
//	uint8_t upper = constrained / 10;
//
//	out = lower & upper << 4;
//}
//
//void transmit(UART_HandleTypeDef* uartHandle, const uint8_t in)
//{
//	// Create buffer
//	uint8_t buf = toBCD(in);
//
//	HAL_UART_Transmit(uartHandle, buf, 1, 1000);
//}
