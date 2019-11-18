#include "Motors.h"

#define PWM_RANGE	100

	Motors::Motors()
	{
		// Motor1 set 100Hz PWM output
		if (softPwmCreate(MOTOR_1_1A, 0, PWM_RANGE) == -1)
		{
			perror("Cannot init PWM");
			exit(EXIT_FAILURE);
		}
		if (softPwmCreate(MOTOR_1_2A, 0, PWM_RANGE) == -1)
		{
			perror("Cannot init PWM");
			exit(EXIT_FAILURE);
		}

		// Motor2 set 100Hz PWM output
		if (softPwmCreate(MOTOR_2_1A, 0, PWM_RANGE) == -1)
		{
			perror("Cannot init PWM");
			exit(EXIT_FAILURE);
		}
		if (softPwmCreate(MOTOR_2_2A, 0, PWM_RANGE) == -1)
		{
			perror("Cannot init PWM");
			exit(EXIT_FAILURE);
		}
	}

	Motors::~Motors()
	{

	}

	int16_t map(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max)
	{
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}

	void Motors::setMotor1(int16_t percent)
	{
		if (percent <= 0)
		{
			softPwmWrite(MOTOR_1_2A, abs(percent));
			softPwmWrite(MOTOR_1_1A, 0);
		}
		else
		{
			softPwmWrite(MOTOR_1_2A, 0);
			softPwmWrite(MOTOR_1_1A, percent);
		}
	}

	void Motors::setMotor2(int16_t percent)
	{
		if (percent <= 0)
		{
			softPwmWrite(MOTOR_2_2A, abs(percent));
			softPwmWrite(MOTOR_2_1A, 0);
		}
		else
		{
			softPwmWrite(MOTOR_2_2A, 0);
			softPwmWrite(MOTOR_2_1A, percent);
		}
	}