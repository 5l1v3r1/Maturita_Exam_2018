#pragma once

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdio.h>
#include <string.h>

	// Define some device constants
	#define LCD_CHR  1 // Mode - Sending data
	#define LCD_CMD  0 // Mode - Sending command

	#define LINE1  0x80 // 1st line
	#define LINE2  0xC0	// 2nd line
	#define LINE3  0x90 // 3nd line
	#define LINE4  0xD0 // 4nd line

	#define LCD_BACKLIGHT   0x08 // On
		 // LCD_BACKLIGHT = 0x00    Off

	#define ENABLE  0b00000100 // Enable bit

	#define LCD_WIDTH  16 // Maximum characters per line

	class LCD_I2C
	{
		public:
			LCD_I2C(int addr);
			~LCD_I2C();
			void ClrLcd(void); // clr LCD return home
			void Write(const char *s, int line);
		private:
			int fd;
			void lcd_byte(int bits, int mode);
			void lcd_toggle_enable(int bits);
	};