/*
 * Comparator.c
 *
 *  Created on: Nov 19, 2021
 *      Author: jessw
 */
#include "comp.h"


float CalculateFrequency(float countSignalAboveVref)
{
	float frequency;
	float timeWindow=0.25;

	frequency=countSignalAboveVref/timeWindow; // calculate frequency from the number of interrupts in a set time

	return(frequency);
}

float CalculateVelocity(float frequency)
{
	float velocity;
	float transmittedfrequency, speedoflight;

	transmittedfrequency = 10.587e9; 	//set values of transmitted frequency and speed of light
	speedoflight = 3e8; 				//as they are constants

	velocity=((frequency*speedoflight)/(2*transmittedfrequency));	//calculate the velocity of object using frequency found and set values

	return(velocity);
}

