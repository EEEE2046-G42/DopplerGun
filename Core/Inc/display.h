/*
 * LCD16x2.h
 *
 *  Created on: 2021-12-02
 *      Author: David Way
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

#include "stm32l4xx_hal.h"

#define LCD_EN_Delay 50

// Rows and column of the LCD
#define LCD_ROWS 2
#define LCD_COLUMNS 16

// Indicator character
#define LCD_INDICATOR 0x7E

//
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
	Misc,
	MilesPerHour,
	KilometresPerHour,
	MetresPerSecond,
} EDisplayState;

// Global variables
EDisplayState LCD_displayState;

//-----[ Prototypes For All Functions ]-----

void LCD_Init();                  // Initialize The LCD For 4-Bit Interface
void LCD_Clear();                 // Clear The LCD Display
void LCD_CMD(unsigned char);      // Send Command To LCD
void LCD_DATA(unsigned char);     // Send 4-Bit Data To LCD
void LCD_Set_Cursor(unsigned char, unsigned char);  // Set Cursor Position
void LCD_Write_Char(char);        // Write Character To LCD At Current Position
void LCD_Write_String(char*);     // Write A String To LCD
void LCD_Home();

void LCD_PULSE_EN();
void LCD_usDelay(uint16_t);

void LCD_DisplayMenu();

#endif /* DISPLAY_H_ */