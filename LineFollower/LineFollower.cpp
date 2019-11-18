#include "LineFollower.h"

LineFollower::LineFollower()
{
	// Init MUX pins
	pinMode(LF_MUX_A_PIN, OUTPUT);	// address
	pinMode(LF_MUX_B_PIN, OUTPUT);	// address
	pinMode(LF_MUX_C_PIN, OUTPUT);	// address
	pinMode(LF_MUX_Z_PIN, INPUT);	// signal
}

unsigned int LineFollower::Calibration()
{
	unsigned int calibWhite, calibBlack, threshold, count = 0, avg = 0;
	unsigned int X[7];
	bool calibDone = false;

	do {
		for (int i = 0; i < 7; i++)
		{
			X[i] = readSensor(i);
		}

		printf("LF_LLLS = %d us\n", X[0]);
		printf("LF_LLS = %d us\n", X[1]);
		printf("LF_LS = %d us\n", X[2]);
		printf("LF_MS = %d us\n", X[3]);
		printf("LF_RS = %d us\n", X[4]);
		printf("LF_RRS = %d us\n", X[5]);
		printf("LF_RRRS = %d us\n\n", X[6]);

		// find max value for black
		calibBlack = 0;
		calibWhite = 0;
		for (int i = 0; i < 7; i++)
		{
			if (X[i] >= calibBlack)
			{
				calibWhite += calibBlack;
				calibBlack = X[i];			
			}
			else
			{
				calibWhite += X[i];
			}
		}

		// averange another values for white
		/*calibWhite = 0;
		for (int i = 0; i < 7; i++)
		{
			if (X[i] != calibBlack)
				calibWhite += X[i];
		}*/
		calibWhite /= 6;

		/*calibWhite = LF_LLLS;
		calibWhite += LF_LLS;
		calibWhite += LF_LS;
		calibWhite += LF_RS;
		calibWhite += LF_RRS;
		calibWhite += LF_RRRS;
		calibWhite = calibWhite / 6;
		//calibBlack = LF_MS;*/
		threshold = ((calibBlack - calibWhite) / 2 + calibWhite);

		printf("calibWhite = %d\n", calibWhite);
		printf("calibBlack = %d\n", calibBlack);
		printf("threshold = %d\n\n", threshold);

		if ((threshold > 0) && (threshold < 16384))
		{
			calibDone = true;
			avg += threshold;
			count++;
		}
		else
		{
			calibDone = false;
		}

		delay(10);
	} while ((!calibDone) || (count < 50));
	avg /= count;

	return avg;
}

unsigned int LineFollower::readSensor(int id)
{
	unsigned int timeStart, timeEnd;

	// set address pins
	digitalWrite(LF_MUX_A_PIN, (id & 0x01));
	digitalWrite(LF_MUX_B_PIN, (id & 0x02));
	digitalWrite(LF_MUX_C_PIN, (id & 0x04));

	delayMicroseconds(100);

	// set Z pin to output, state HIGH
	system("gpio export 12 out");
	pinMode(LF_MUX_Z_PIN, OUTPUT);
	digitalWrite(LF_MUX_Z_PIN, HIGH);

	delayMicroseconds(100);

	// set Z pin to input, state HI-IMPEDANCE
	system("gpio export 12 in");
	pinMode(LF_MUX_Z_PIN, INPUT);
	timeStart = micros();
	while (digitalRead(LF_MUX_Z_PIN));	// waiting for I/O line to go low
	timeEnd = micros();

	return (timeEnd - timeStart);
}

LineFollower::~LineFollower()
{


}