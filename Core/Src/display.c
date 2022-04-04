/*
 * display.c
 *
 *  Created on: 2021-12-02
 *      Author: David Way
 */
#include "display.h"

// Set display port and pins
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

// Initialize LCD (4-bit)
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
	SETLOW(RS_PIN);
	SETLOW(EN_PIN);

	LCD_SetData(0x00);
	HAL_Delay(150);

	LCD_Command(0x03);
	HAL_Delay(5);

	LCD_Command(0x03);
	LCD_usDelay(150);

	LCD_Command(0x03);
	LCD_Command(0x02);

	LCD_Command(0x02);
	LCD_Command(0x08);

	LCD_Command(0x00);
	LCD_Command(0x0C);

	LCD_Command(0x00);
	LCD_Command(0x06);

	LCD_Command(0x00);
	LCD_Command(0x01);
}

// Clear display
void LCD_Clear()
{
	LCD_Command(0x00);
    LCD_Command(0x01);
    LCD_usDelay(50);
}

// Send command to LCD
void LCD_Command(unsigned char a_CMD)
{
    // Select Command Register
	SETLOW(RS_PIN);

    // Move The Command Data To LCD
    LCD_SetData(a_CMD);

    // Pulse enable pin
    LCD_PulseEN();
}

// Set data bits
void LCD_SetData(unsigned char data)
{
	SET(D4_PIN, data & 0b0001);
	SET(D5_PIN, data & 0b0010);
	SET(D6_PIN, data & 0b0100);
	SET(D7_PIN, data & 0b1000);
}

// Set cursor pos
void LCD_SetCursor(unsigned char c, unsigned char r)
{
    unsigned char temp, lowerNib, upperNib;
    if(r == 0)
    {
      //temp  = 0b1000 + c; //0x80 is used to move the cursor
      temp  = 0x80 + c;
      upperNib = temp >> 4;
      lowerNib  = temp & 0x0F;
      LCD_Command(upperNib);
      LCD_Command(lowerNib);
    }
    else if(r == 1)
    {
      //temp  = 0b1000 + 0x40 + c;
      temp  = 0xC0 + c;
      upperNib = temp >> 4;
      lowerNib  = temp & 0x0F;
      LCD_Command(upperNib);
      LCD_Command(lowerNib);
    }

    HAL_Delay(1);
}

// Write character to LCD at cursor pos
void LCD_WriteChar(char data)
{
   char lowerNib,upperNib;
   lowerNib  = data & 0x0F;
   upperNib = (data & 0xF0) >> 4;

   SETHIGH(RS_PIN);

   LCD_SetData(upperNib);
   LCD_PulseEN();

   LCD_SetData(lowerNib);
   LCD_PulseEN();
}

// Write a string to LCD
void LCD_WriteString(char *str)
{
    int i;
    for(i=0;str[i]!='\0';i++)
       LCD_WriteChar(str[i]);
}

// Sets cursor to (0,0)
void LCD_Home()
{
	LCD_Command(0x02);
}

// Pulses enable pin
void LCD_PulseEN()
{
    SETHIGH(EN_PIN);
    LCD_usDelay(LCD_EN_Delay);
    SETLOW(EN_PIN);
    LCD_usDelay(LCD_EN_Delay);
}

// Short delay
void LCD_usDelay(uint16_t t)
{
	// Do nothing loop
	for (uint16_t i = 0; i < t * 40; i++);
}

// Update and display menu (unit selection and indicator)
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
	LCD_SetCursor(0, 1);
	for (int i = 0; i < LCD_COLUMNS; i++)
		LCD_WriteChar(' ');

	// Print speed labels

	// mph
	LCD_SetCursor(mphx, 1);
	LCD_WriteChar(LCD_displayState == MilesPerHour ? LCD_INDICATOR : ' ');
	LCD_WriteString("mph");

	// kmh
	LCD_SetCursor(kmhx, 1);
	LCD_WriteChar(LCD_displayState == KilometresPerHour ? LCD_INDICATOR : ' ');
	LCD_WriteString("kmh");

	// m/s
	LCD_SetCursor(msx, 1);
	LCD_WriteChar(LCD_displayState == MetresPerSecond ? LCD_INDICATOR : ' ');
	LCD_WriteString("m/s");

	// Set prev value
	LCD_prevDisplayState = LCD_displayState;
}

// Update and display speed
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
		// Set cursor position and clear row (excluding final col)
		LCD_SetCursor(0, 0);
		for (int i = 0; i < LCD_COLUMNS - 1; i++)
				LCD_WriteChar(' ');
		LCD_SetCursor(6, 0);

		// Print string
		LCD_WriteString(strSpeed);

		// Copy to prev value
		for (int i = 0; i < LCD_SPEED_LENGTH; i++)
			LCD_prevSpeedStr[i] = strSpeed[i];
	}
}

// Function to set speed unit and update display
void LCD_SetSpeedUnit(EDisplayState unit)
{
	LCD_displayState = unit;
	LCD_DisplayMenu();
	LCD_DisplaySpeed(LCD_currentSpeed);
}


// Format speed given in m/s as string, output depending on selected unit
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

// Handles button presses
void LCD_ButtonHandler()
{
	// Increment speed unit
	LCD_SetSpeedUnit(LCD_displayState == 2 ? 0 : LCD_displayState + 1);
}

