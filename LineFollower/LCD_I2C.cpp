#include "LCD_I2C.h"

//namespace Robot_v1
//{
	LCD_I2C::LCD_I2C(int addr)
	{
		fd = wiringPiI2CSetup(addr);

		// Initialise display
		lcd_byte(0x33, LCD_CMD); // Initialise
		lcd_byte(0x32, LCD_CMD); // Initialise
		lcd_byte(0x06, LCD_CMD); // Cursor move direction
		lcd_byte(0x0C, LCD_CMD); // 0x0F On, Blink Off
		lcd_byte(0x28, LCD_CMD); // Data length, number of lines, font size
		lcd_byte(0x01, LCD_CMD); // Clear display
		delayMicroseconds(500);
	}

	// clr lcd go home loc 0x80
	void LCD_I2C::ClrLcd(void)
	{
		lcd_byte(0x01, LCD_CMD);
		lcd_byte(0x02, LCD_CMD);
	}

	// this allows use of any size string
	void LCD_I2C::Write(const char *s, int line)
	{
		size_t size = strlen(s);

		lcd_byte(line, LCD_CMD);

		if (size > LCD_WIDTH)
			size = LCD_WIDTH;

		for (size_t i = 0; i < size; i++)
			lcd_byte(s[i], LCD_CHR);
	}

	void LCD_I2C::lcd_byte(int bits, int mode)
	{
		int bits_high, bits_low;

		// Send byte to data pins
		// bits = the data
		// mode = 1 for data
		//        0 for command
		bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT;
		bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT;

		// High bits
		wiringPiI2CWrite(fd, bits_high);
		lcd_toggle_enable(bits_high);

		// Low bits
		wiringPiI2CWrite(fd, bits_low);
		lcd_toggle_enable(bits_low);
	}

	void LCD_I2C::lcd_toggle_enable(int bits)
	{
		// Toggle enable pin on LCD display
		delayMicroseconds(500);
		wiringPiI2CReadReg8(fd, (bits | ENABLE));
		delayMicroseconds(500);
		wiringPiI2CReadReg8(fd, (bits & ~ENABLE));
		delayMicroseconds(500);
	}

	LCD_I2C::~LCD_I2C()
	{
		
	}
//}