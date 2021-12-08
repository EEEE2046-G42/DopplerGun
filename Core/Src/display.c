/*
 * display.c
 *
 *  Created on: 2021-12-02
 *      Author: David Way
 */
#include "display.h"

const DisplayPinConfig DisplayPins =
{
	GPIOB,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_4,
	GPIO_PIN_3
};

// Macros to shorthand set high and low
#define SETHIGH(x) HAL_GPIO_WritePin(DisplayPins.LCD_GPIO, DisplayPins.x, 1)
#define SETLOW(x) HAL_GPIO_WritePin(DisplayPins.LCD_GPIO, DisplayPins.x, 0)


//-----[ Alphanumeric LCD 16x2 Routines ]-----

void LCD_usDelay(uint16_t t)
{
	// Do nothing loop
	for (uint16_t i = 0; i < t * 40; i++);
}

void LCD_PULSE_EN()
{
	// Send The EN Clock Signal
    SETHIGH(EN_PIN);
    LCD_usDelay(LCD_EN_Delay);
    SETLOW(EN_PIN);
    LCD_usDelay(LCD_EN_Delay);
}

// Set parallel bits
void LCD_DATA(unsigned char data)
{
    if(data & 0b0001)
    	SETHIGH(D4_PIN);
    else
    	SETLOW(D4_PIN);

    if(data & 0b0010)
    	SETHIGH(D5_PIN);
    else
    	SETLOW(D5_PIN);

    if(data & 0b0100)
    	SETHIGH(D6_PIN);
    else
    	SETLOW(D6_PIN);

    if(data & 0b1000)
    	SETHIGH(D7_PIN);
    else
    	SETLOW(D7_PIN);
}

void LCD_CMD(unsigned char a_CMD)
{
    // Select Command Register
	SETLOW(RS_PIN);

    // Move The Command Data To LCD
    LCD_DATA(a_CMD);

    // Pulse enable pin
    LCD_PULSE_EN();
}

void LCD_Clear()
{
	LCD_CMD(0x00);
    LCD_CMD(0x01);
    LCD_usDelay(50);
}

void LCD_Set_Cursor(unsigned char c, unsigned char r)
{
    unsigned char temp, lowerNib, upperNib;
    if(r == 0)
    {
      //temp  = 0b1000 + c; //0x80 is used to move the cursor
      temp  = 0x80 + c;
      upperNib = temp >> 4;
      lowerNib  = temp & 0x0F;
      LCD_CMD(upperNib);
      LCD_CMD(lowerNib);
    }
    else if(r == 1)
    {
      //temp  = 0b1000 + 0x40 + c;
      temp  = 0xC0 + c;
      upperNib = temp >> 4;
      lowerNib  = temp & 0x0F;
      LCD_CMD(upperNib);
      LCD_CMD(lowerNib);
    }

    HAL_Delay(1);
}

void LCD_ButtonHandler()
{
	LCD_displayState = LCD_displayState == 2 ? 0 : LCD_displayState + 1;
	//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
	LCD_DisplayMenu();
	LCD_DisplaySpeed(LCD_currentSpeed);
}

void LCD_Init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

    // IO Pin Configurations
	if(DisplayPins.LCD_GPIO == GPIOA)
	    __HAL_RCC_GPIOA_CLK_ENABLE();
	else if(DisplayPins.LCD_GPIO == GPIOB)
	    __HAL_RCC_GPIOB_CLK_ENABLE();
	else if(DisplayPins.LCD_GPIO == GPIOC)
		__HAL_RCC_GPIOC_CLK_ENABLE();
	else if(DisplayPins.LCD_GPIO == GPIOD)
		__HAL_RCC_GPIOD_CLK_ENABLE();

	// Set all pins LOW
	SETLOW(D4_PIN);
	SETLOW(D5_PIN);
	SETLOW(D6_PIN);
	SETLOW(D7_PIN);
	SETLOW(RS_PIN);
	SETLOW(EN_PIN);

	// Initialise GPIO pins as push-pull, no pullup, low-speed
	GPIO_InitStruct.Pin = DisplayPins.D4_PIN | DisplayPins.D5_PIN |
			DisplayPins.D6_PIN |DisplayPins.D7_PIN | DisplayPins.RS_PIN |
			DisplayPins.EN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(DisplayPins.LCD_GPIO, &GPIO_InitStruct);

	// 4-bit initialisation procedure

	/*// Set E and RS LOW
	SETLOW(RS_PIN);
	SETLOW(EN_PIN);

    LCD_DATA(0x00);		// Set data pins low
    HAL_Delay(150);		// Wait 150 ms

    LCD_CMD(0b0011);	// Send 0b0011
    HAL_Delay(2);		// Wait 2 ms

    LCD_CMD(0b0010);
    LCD_CMD(0b1000);	// Set number of lines and data length

    HAL_Delay(1);

    LCD_CMD(0b0010);
	LCD_CMD(0b1000);	// Set number of lines and data length

	HAL_Delay(1);

	LCD_CMD(0b0000);
	LCD_CMD(0b1110);	// Enable display, cursor, and blinking

	HAL_Delay(1);

	LCD_Clear();

	HAL_Delay(10);

	LCD_CMD(0b0000);
	LCD_CMD(0b0100);	// Set entry mode*/

	SETLOW(RS_PIN);
	SETLOW(EN_PIN);

	LCD_DATA(0x00);
	HAL_Delay(150);

	LCD_CMD(0x03);
	HAL_Delay(5);

	LCD_CMD(0x03);
	LCD_usDelay(150);

	LCD_CMD(0x03);
	LCD_CMD(0x02);

	LCD_CMD(0x02);
	LCD_CMD(0x08);

	LCD_CMD(0x00);
	LCD_CMD(0x0C);

	LCD_CMD(0x00);
	LCD_CMD(0x06);

	LCD_CMD(0x00);
	LCD_CMD(0x01);
}

void LCD_Write_Char(char data)
{
   char lowerNib,upperNib;
   lowerNib  = data & 0x0F;
   upperNib = (data & 0xF0) >> 4;

   SETHIGH(RS_PIN);

   LCD_DATA(upperNib);
   LCD_PULSE_EN();

   LCD_DATA(lowerNib);
   LCD_PULSE_EN();
}

void LCD_Write_String(char *str)
{
    int i;
    for(i=0;str[i]!='\0';i++)
       LCD_Write_Char(str[i]);
}

void LCD_Home()
{
	LCD_CMD(0x02);
}

void LCD_DisplayMenu()
{
	// Don't do anything if state is the same
	if (LCD_displayState == LCD_prevDisplayState)
		return;

	// Label locations
	const unsigned char mphx = 0;
	const unsigned char kmhx = 6;
	const unsigned char msx = 12;

	// Clear row
	LCD_Set_Cursor(0, 1);
	for (int i = 0; i < LCD_COLUMNS; i++)
		LCD_Write_Char(' ');

	// Print speed labels

	// mph
	LCD_Set_Cursor(mphx, 1);
	LCD_Write_Char(LCD_displayState == MilesPerHour ? LCD_INDICATOR : ' ');
	LCD_Write_String("mph");

	// kmh
	LCD_Set_Cursor(kmhx, 1);
	LCD_Write_Char(LCD_displayState == KilometresPerHour ? LCD_INDICATOR : ' ');
	LCD_Write_String("kmh");

	// m/s
	LCD_Set_Cursor(msx, 1);
	LCD_Write_Char(LCD_displayState == MetresPerSecond ? LCD_INDICATOR : ' ');
	LCD_Write_String("m/s");

	// Set prev value
	LCD_prevDisplayState = LCD_displayState;
}

void LCD_DisplaySpeed(const float metresPerSecond)
{
	LCD_currentSpeed = metresPerSecond;

	// Format speed as string
	char strSpeed[LCD_SPEED_LENGTH];
	LCD_FormatSpeed(metresPerSecond, strSpeed);

	// Only update speed on display if text has changed!
	bool hasChanged = false;
	for (int i = 0; i < LCD_SPEED_LENGTH; i++)
		if (strSpeed[i] != LCD_prevSpeedStr[i]) hasChanged = true;

	if (hasChanged)
	{
		// Set cursor position and clear row
		LCD_Set_Cursor(0, 0);
		for (int i = 0; i < LCD_COLUMNS; i++)
				LCD_Write_Char(' ');
		LCD_Set_Cursor(6, 0);

		// Print string
		LCD_Write_String(strSpeed);

		// Copy to prev value
		for (int i = 0; i < LCD_SPEED_LENGTH; i++)
			LCD_prevSpeedStr[i] = strSpeed[i];
	}
}

void LCD_FormatSpeed(const float metresPerSecond, char output[])
{
	float convertedSpeed;

	// Convert speed to output format
	switch (LCD_displayState)
	{
	case MilesPerHour:
		convertedSpeed = metresPerSecond * 2.23694;
		break;
	case KilometresPerHour:
		convertedSpeed = metresPerSecond * 3.6;
		break;
	case MetresPerSecond:
		convertedSpeed = metresPerSecond;
	}

	// Convert float to formatted string
	sprintf(output, "%.2f", convertedSpeed);
}
