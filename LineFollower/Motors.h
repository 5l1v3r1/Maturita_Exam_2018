#pragma once

#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

// Motors
#define MOTOR_1_1A	(16)
#define MOTOR_1_2A	(25)
#define MOTOR_2_1A	(21)
#define MOTOR_2_2A	(20)

// Encoders
#define ENCODER_M1_A_PIN	(24)
#define ENCODER_M1_B_PIN	(23)
#define ENCODER_M2_A_PIN	(6)
#define ENCODER_M2_B_PIN	(5)

class Motors
{
	public:
		Motors();
		~Motors();
		void setMotor1(int16_t speed);
		void setMotor2(int16_t speed);
		volatile long double Motor1_d = 0.0;
		volatile long double Motor2_d = 0.0;
	private:
};
