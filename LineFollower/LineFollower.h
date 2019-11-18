#pragma once

#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>

#define	LF_NUM_OF_SENSORS	(7)

#define	LF_MUX_A_PIN		(8)
#define	LF_MUX_B_PIN		(7)
#define	LF_MUX_C_PIN		(1)
#define	LF_MUX_Z_PIN		(12)

class LineFollower
{
public:
	LineFollower();
	~LineFollower();
	unsigned int Calibration();
	unsigned int readSensor(int id);
	unsigned int LF_Sensors_buff[LF_NUM_OF_SENSORS];
};