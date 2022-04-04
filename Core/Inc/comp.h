/*
 * comp.h
 *
 *  Created on: Nov 19, 2021
 *      Author: jessw
 */

//#include "Comparator.c"

#ifndef INC_COMP_H_
#define INC_COMP_H_

void COMP_IRQHandler(void);
float CalculateFrequency(float timeperiod);
float CalculateVelocity(float frequency);
//void DisplayVelocityOnLCD(float velocity);

#endif /* INC_COMP_H_ */
