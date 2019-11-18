//	Global
#include <wiringPi.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

//	My
#include "LCD_I2C.h"
#include "LineFollower.h"
#include "Motors.h"
#include "SoundCard.h"
#include "MachineLearning/machine_learning.hpp"
#include "IMU.h"

//	IMU
// Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
// Find yours here: http://www.magnetic-declination.com/
// Mine is: 17° 49' 33.3" E which is +4° 34' => +4.566667°
// If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
#define IMU_INT_PIN				(17)
#define IMU_declinationAngle	(4.0f + (34.0f / 60.0f))
IMU *mIMU;

//	Encoder
#define ROBOT_WHEEL_DIST		  (0.115)	// [m]
#define ROBOT_ENCODER_CONST		(0.00012)	// [m]
Motors *mMotors;
void ENCODER_M1_A_FN();
void ENCODER_M1_B_FN();
void ENCODER_M2_A_FN();
void ENCODER_M2_B_FN();

//	Sound card
SoundCard *mSoundCard;

//	Log file
FILE *f_linefollower_log;

//	LCD display
LCD_I2C *mLCD;

int main(void)
{
	MachineLearning *neuron_site;
	char ch = 0, buf[LCD_WIDTH];
	float X[7];

	//	init wiringPi
	if (wiringPiSetupSys() == -1)
	{
		perror("Cannot init wiringPi");
		exit(EXIT_FAILURE);
	}

	//	create log file
	f_linefollower_log = fopen("file_linefollower_log.txt", "w");
	fprintf(f_linefollower_log, "X0;X1;X2;X3;X4;X5;X6;Motor1_speed[%];Motor2_speed[%];ENCODER_M1[m];ENCODER_M2[m]\n");

	//	init motors
	mMotors = new Motors();
	if (wiringPiISR(ENCODER_M1_A_PIN, INT_EDGE_BOTH, ENCODER_M1_A_FN) < 0)
	{
		perror("Cannot init M1_A_ISR");
		exit(EXIT_FAILURE);
	}
	if (wiringPiISR(ENCODER_M1_B_PIN, INT_EDGE_BOTH, ENCODER_M1_B_FN) < 0)
	{
		perror("Cannot init M1_B_ISR");
		exit(EXIT_FAILURE);
	}
	if (wiringPiISR(ENCODER_M2_A_PIN, INT_EDGE_BOTH, ENCODER_M2_A_FN) < 0)
	{
		perror("Cannot init M2_A_ISR");
		exit(EXIT_FAILURE);
	}
	if (wiringPiISR(ENCODER_M2_B_PIN, INT_EDGE_BOTH, ENCODER_M2_B_FN) < 0)
	{
		perror("Cannot init M2_B_ISR");
		exit(EXIT_FAILURE);
	}

	//	init LineFollower sensors
	LineFollower *mLF = new LineFollower();
	auto mLF_threshold = mLF->Calibration();

	//	init soundcard
	mSoundCard = new SoundCard();
	mSoundCard->Open("/home/pi/start.mp3");
	while (mSoundCard->Play() == MPG123_OK);

	//	init LCD
	mLCD = new LCD_I2C(0x3F);

	//	init IMU
	//mIMU = new IMU();
	//delay(100);

	//	set stdin non-blocking
	fcntl(0, F_SETFL, O_NONBLOCK);

	//	init neuron site
	neuron_site = new MachineLearning(3, new uint[3] { 7, 5, 2 }, 0);
	
	//	set weights
	neuron_site->weights[0][0][0] = -2.255463f;
	neuron_site->weights[0][1][0] = -1.897195f;
	neuron_site->weights[0][2][0] = 2.353793f;
	neuron_site->weights[0][3][0] = -0.8203334f;
	neuron_site->weights[0][4][0] = 2.062405f;
	neuron_site->weights[0][5][0] = -1.339868f;
	neuron_site->weights[0][6][0] = 2.170322f;
	neuron_site->weights[0][7][0] = 2.935545f;
	neuron_site->weights[0][8][0] = 1.183003f;

	neuron_site->weights[1][0][0] = -0.2567006f; neuron_site->weights[1][0][1] = -0.1599264f; neuron_site->weights[1][0][2] = 0.05654039f; neuron_site->weights[1][0][3] = 0.1041764f; neuron_site->weights[1][0][4] = 0.5997388f; neuron_site->weights[1][0][5] = -0.2585668f; neuron_site->weights[1][0][6] = 0.2609726f; neuron_site->weights[1][0][7] = 0.03803285f; neuron_site->weights[1][0][8] = -0.2478838f;
	neuron_site->weights[1][1][0] = 0.6135998f; neuron_site->weights[1][1][1] = 1.059701f; neuron_site->weights[1][1][2] = -1.650844f; neuron_site->weights[1][1][3] = 0.7898874f; neuron_site->weights[1][1][4] = 1.699022f; neuron_site->weights[1][1][5] = -0.8410522f; neuron_site->weights[1][1][6] = 1.427616f; neuron_site->weights[1][1][7] = -0.2573749f; neuron_site->weights[1][1][8] = -0.2619244f;
	neuron_site->weights[1][2][0] = -0.5646847f; neuron_site->weights[1][2][1] = -1.754406f; neuron_site->weights[1][2][2] = 1.169441f; neuron_site->weights[1][2][3] = 0.3934665f; neuron_site->weights[1][2][4] = -1.208777f; neuron_site->weights[1][2][5] = 1.270509f; neuron_site->weights[1][2][6] = -1.105578f; neuron_site->weights[1][2][7] = 0.09866305f; neuron_site->weights[1][2][8] = -0.2219399f;
	neuron_site->weights[1][3][0] = 0.1829518f; neuron_site->weights[1][3][1] = 0.2200905f; neuron_site->weights[1][3][2] = -0.2706444f; neuron_site->weights[1][3][3] = -0.2182662f; neuron_site->weights[1][3][4] = 0.5362803f; neuron_site->weights[1][3][5] = -0.4913092f; neuron_site->weights[1][3][6] = 0.3658705f; neuron_site->weights[1][3][7] = -0.2661656f; neuron_site->weights[1][3][8] = -0.193917f;
	neuron_site->weights[1][4][0] = -0.4267969f; neuron_site->weights[1][4][1] = 0.09001755f; neuron_site->weights[1][4][2] = 0.4649961f; neuron_site->weights[1][4][3] = -0.4314389f; neuron_site->weights[1][4][4] = -0.2467905f; neuron_site->weights[1][4][5] = -0.1869671f; neuron_site->weights[1][4][6] = -0.306278f; neuron_site->weights[1][4][7] = -0.1827863f; neuron_site->weights[1][4][8] = 0.05521876f;

	neuron_site->weights[2][0][0] = -0.01342729f; neuron_site->weights[2][0][1] = -1.094443f; neuron_site->weights[2][0][2] = 0.03232949f; neuron_site->weights[2][0][3] = 0.6937188f; neuron_site->weights[2][0][4] = -0.7215374f;
	neuron_site->weights[2][1][0] = 0.2706088f; neuron_site->weights[2][1][1] = -0.3644964f; neuron_site->weights[2][1][2] = -1.081221f; neuron_site->weights[2][1][3] = -0.6328434f; neuron_site->weights[2][1][4] = -0.1321037f;

	//	LCD print Start info
	mLCD->Write("LineFollower", LINE1);
	mLCD->Write("Martin Horvath", LINE2);
	delay(3000);
	mLCD->ClrLcd();

	//	Loop
	while (ch != 'q')
	{
		// LineFollower
		for (int n = 0; n < 7; n++)
		{
			if (mLF->readSensor(n) >= mLF_threshold)	X[n] = 1.0f;
			else                                        X[n] = 0.0f;
		}
		neuron_site->clearOutputs();
		neuron_site->getOutput(X);
		auto Motor1 = (int16_t) roundf(neuron_site->output[neuron_site->last_layer_index][0] * 100.0f);
		auto Motor2 = (int16_t) (roundf(neuron_site->output[neuron_site->last_layer_index][1] * 100.0f));
		mMotors->setMotor1(Motor1);
		mMotors->setMotor2(Motor2);
		
		// read Quit character
		read(0, &ch, 1);

		// Log linefollower data
		fprintf(f_linefollower_log, "%1.0f;%1.0f;%1.0f;%1.0f;%1.0f;%1.0f;%1.0f;%1.2f;%1.2f;%1.3f;%1.3f\n", X[0], X[1], X[2], X[3], X[4], X[5], X[6], neuron_site->output[neuron_site->last_layer_index][0], neuron_site->output[neuron_site->last_layer_index][1], mMotors->Motor1_d, mMotors->Motor2_d);

		// LCD
		mLCD->ClrLcd();	// Clear LCD
		sprintf(buf, "%1.0f;%1.0f;%1.0f;%1.0f;%1.0f;%1.0f;%1.0f", X[0], X[1], X[2], X[3], X[4], X[5], X[6]);
		puts(buf);
		mLCD->Write(buf, LINE1);
		sprintf(buf, "%1.2f;%1.2f", neuron_site->output[neuron_site->last_layer_index][0], neuron_site->output[neuron_site->last_layer_index][1]);
		puts(buf);
		mLCD->Write(buf, LINE2);
		//sprintf(buf, "TEMP = %d°C", mIMU->getTemp());
		//mLCD->Write(buf, LINE3);
	}

	fclose(f_linefollower_log);

	return 0;
}

void ENCODER_M1_A_FN()
{
	mMotors->Motor1_d += ROBOT_ENCODER_CONST;
}

void ENCODER_M1_B_FN()
{
	mMotors->Motor1_d += ROBOT_ENCODER_CONST;
}

void ENCODER_M2_A_FN()
{
	mMotors->Motor2_d += ROBOT_ENCODER_CONST;
}

void ENCODER_M2_B_FN()
{
	mMotors->Motor2_d += ROBOT_ENCODER_CONST;
}

/*void alarmWakeup(int sig_num)	// Every 100ms (10Hz) save nav data
{
	if (sig_num == SIGALRM)
	{
		uint8_t sysStat, gyroStat, accelStat, magStat;
		mIMU->getCalibration(&sysStat, &gyroStat, &accelStat, &magStat);
		//printf("sysStat = %d, gyroStat = %d, accelStat = %d, magStat = %d\n", sysStat, gyroStat, accelStat, magStat);

		if (gyroStat == 3)
		{
			IMU::Vector3 gyro = mIMU->getVector(mIMU->VECTOR_GYROSCOPE);
			mIMU->gyro_z_integral = (0.050f * (mIMU->gyro_z_last + gyro.z));
			mIMU->gyro_z_last = gyro.z;
		}
		else
		{
			mIMU->gyro_z_integral = 0;
		}

		//	Log encoders data
		//auto Alfa = (mMotors->Motor1_d - mMotors->Motor2_d) / ROBOT_WHEEL_DIST;
		//auto s = (mMotors->Motor1_d + mMotors->Motor2_d) / 2.0;
		//auto ds = s - s_last;
		//if (ds != 0)
		//{
			//nav_X += ds * cos(Alfa);
			//nav_Y += ds * sin(Alfa);
			fprintf(f_out, "%f;%f;%f;0.100\n", mMotors->Motor1_d, mMotors->Motor2_d, mIMU->gyro_z_integral);
		//}
		//s_last = s;
	}
}*/

/*int voltageMonitor(void)
{
	// low battery voltage
	if (digitalRead(BATTERY_ALARM_PIN) == LOW)
	{
		// play track

		// Text warning
		for (int i = 0; i < 5; i++)
			printf("Battery is low !!!\n");

		// LCD display message
		mLCD->Write("Battery is low !", LINE4);

		return 1;
	}

	// low solar voltage
	if (digitalRead(SOLAR_ALARM_PIN) == LOW)
	{
		// play track

		// Text warning
		for (int i = 0; i < 5; i++)
			printf("Solar is low !!!\n");

		// LCD display message
		mLCD->Write("Solar is low !", LINE4);

		return 2;
	}

	return 0;
}*/