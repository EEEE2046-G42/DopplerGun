/*
 * LCD16x2.h
 *
 *  Created on: 2021-12-02
 *      Author: David Way
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "stm32l4xx_hal.h"
#include "stdbool.h"

// Macros to shorthand set high and low
#define SETHIGH(x) HAL_GPIO_WritePin(DisplayPins.LCD_GPIO, DisplayPins.x, 1)
#define SETLOW(x) HAL_GPIO_WritePin(DisplayPins.LCD_GPIO, DisplayPins.x, 0)
#define SET(x, y) HAL_GPIO_WritePin(DisplayPins.LCD_GPIO, DisplayPins.x, y)

// Delay for EN pin pulse
#define LCD_EN_Delay 50

// Rows and column of the LCD
#define LCD_ROWS 2
#define LCD_COLUMNS 16

// Indicator character
#define LCD_INDICATOR 0x7E

// Speed character length
#define LCD_SPEED_LENGTH 6

// Struct to contain parallel bus port and pin numbers
typedef struct
{
	GPIO_TypeDef * LCD_GPIO;
	uint16_t D4_PIN;
	uint16_t D5_PIN;
	uint16_t D6_PIN;
	uint16_t D7_PIN;
	uint16_t EN_PIN;
	uint16_t RS_PIN;
} DisplayPinConfig;

// Lists display states (speed units etc)
typedef enum
{
	MilesPerHour,
	KilometresPerHour,
	MetresPerSecond,
} EDisplayState;

// Global variables
EDisplayState LCD_displayState;				// Current display state
EDisplayState LCD_prevDisplayState;			// Previous display state
char LCD_prevSpeedStr[LCD_SPEED_LENGTH];	// Previously displayed speed string
float LCD_currentSpeed;						// Current speed in metres per second

// Function prototypes
void LCD_Init();                  					// Initialize LCD (4-bit)
void LCD_Clear();                 					// Clear display
void LCD_Command(unsigned char);      				// Send command to LCD
void LCD_SetData(unsigned char);     				// Set data bits
void LCD_SetCursor(unsigned char, unsigned char);  	// Set cursor pos
void LCD_WriteChar(char);        					// Write character to LCD at cursor pos
void LCD_WriteString(char*);     					// Write a string to LCD
void LCD_Home();									// Sets cursor to (0,0)

void LCD_PulseEN();				// Pulses enable pin
void LCD_usDelay(uint16_t);		// Short delay

void LCD_DisplayMenu();				// Update and display menu (unit selection and indicator)
void LCD_DisplaySpeed(const float);	// Update and display speed

void LCD_SetSpeedUnit(EDisplayState);		// Function to set speed unit and update display
void LCD_FormatSpeed(const float, char[]);	// Format speed given in m/s as string, output depending on selected unit

void LCD_ButtonHandler();	// Handles button presses

#endif /* DISPLAY_H_ */
