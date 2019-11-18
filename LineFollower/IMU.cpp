#include "IMU.h"

IMU::IMU(uint8_t address, adafruit_bno055_opmode_t mode)
{
	fd = wiringPiI2CSetup(address);

	/* Switch to config mode (just in case since this is the default) */
	setMode(OPERATION_MODE_CONFIG);

	/* Reset */
	int count = 0;
	wiringPiI2CWriteReg8(fd, BNO055_SYS_TRIGGER_ADDR, 0x20);
	while (wiringPiI2CReadReg8(fd, BNO055_CHIP_ID_ADDR) != BNO055_ID)
	{
		if (count >= 255) break;
		delay(10);
		count++;
	}

	/* Set to normal power mode */
	wiringPiI2CWriteReg8(fd, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	delay(10);

	/* Interrupt enable */
	/*wiringPiI2CWriteReg8(fd, BNO055_PAGE_ID_ADDR, 0x01);		// Select page 1
	wiringPiI2CWriteReg8(fd, BNO055_INT_EN, 0x04);				// GYRO_AM
	wiringPiI2CWriteReg8(fd, BNO055_INT_MSK, 0x04);				// GYRO_AM
	wiringPiI2CWriteReg8(fd, BNO055_GYRO_INT_SETING, 0x04);		// GYRO_SETING: enable Z-axis, Filtred
	wiringPiI2CWriteReg8(fd, BNO055_GYRO_AM_THRES, 0x01);		// 1deg/s
	wiringPiI2CWriteReg8(fd, BNO055_GYRO_AM_SET, 0x00);*/		// Awake Duration = 0 = 8 samples, Slope Samples = (0 + 1) * 4 = 4

	//wiringPiI2CWriteReg8(fd, BNO055_INT_EN, 0x40);				// ACC_AM
	//wiringPiI2CWriteReg8(fd, BNO055_INT_MSK, 0x40);				// ACC_AM
	//wiringPiI2CWriteReg8(fd, BNO055_ACC_INT_SETTINGS, 0x1F);	// axis X + Y + Z, AM_DUR = 4 decimal
	//wiringPiI2CWriteReg8(fd, BNO055_ACC_AM_THRES, 0x05);

	/* Select page 0 */
	wiringPiI2CWriteReg8(fd, BNO055_PAGE_ID_ADDR, 0x00);

	/* Set the output units:
	Orientation = Windows
	Temperature = Celsius
	Euler = Degrees
	Gyro = Degrees/s
	Accelerometer = m/s^2
	*/
	wiringPiI2CWriteReg8(fd, BNO055_UNIT_SEL_ADDR, 0x00);

	/* Configure axis mapping (see section 3.4) */
	wiringPiI2CWriteReg8(fd, BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P1); // P0-P7, Default is P1
	wiringPiI2CWriteReg8(fd, BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P1); // P0-P7, Default is P1

	/* Use internal osc */
	wiringPiI2CWriteReg8(fd, BNO055_SYS_TRIGGER_ADDR, 0x00);
	delay(10);

	/* Set the requested operating mode (see section 3.3) */
	setMode(mode);
}

IMU::~IMU()
{

}

void IMU::setMode(adafruit_bno055_opmode_t mode)
{
	_mode = mode;
	wiringPiI2CWriteReg8(fd, BNO055_OPR_MODE_ADDR, _mode);
	delay(30);
}

void IMU::setExtCrystalUse(bool usextal)
{
	adafruit_bno055_opmode_t modeback = _mode;

	/* Switch to config mode (just in case since this is the default) */
	setMode(OPERATION_MODE_CONFIG);

	wiringPiI2CWriteReg8(fd, BNO055_PAGE_ID_ADDR, 0);
	if (usextal) {
		wiringPiI2CWriteReg8(fd, BNO055_SYS_TRIGGER_ADDR, 0x80);
	}
	else {
		wiringPiI2CWriteReg8(fd, BNO055_SYS_TRIGGER_ADDR, 0x00);
	}
	delay(10);

	/* Set the requested operating mode (see section 3.3) */
	setMode(modeback);
}

void IMU::resetInterrupt(void)
{
	adafruit_bno055_opmode_t modeback = _mode;

	/* Switch to config mode (just in case since this is the default) */
	setMode(OPERATION_MODE_CONFIG);

	wiringPiI2CWriteReg8(fd, BNO055_PAGE_ID_ADDR, 0);
	wiringPiI2CWriteReg8(fd, BNO055_SYS_TRIGGER_ADDR, 0x40);

	/* Set the requested operating mode (see section 3.3) */
	setMode(modeback);
}

void IMU::getSystemStatus(uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
	wiringPiI2CWriteReg8(fd, BNO055_PAGE_ID_ADDR, 0);

	/* System Status (see section 4.3.58)
	---------------------------------
	0 = Idle
	1 = System Error
	2 = Initializing Peripherals
	3 = System Iniitalization
	4 = Executing Self-Test
	5 = Sensor fusio algorithm running
	6 = System running without fusion algorithms */

	if (system_status != 0)
		*system_status = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_SYS_STAT_ADDR);

	/* Self Test Results (see section )
	--------------------------------
	1 = test passed, 0 = test failed

	Bit 0 = Accelerometer self test
	Bit 1 = Magnetometer self test
	Bit 2 = Gyroscope self test
	Bit 3 = MCU self test

	0x0F = all good! */

	if (self_test_result != 0)
		*self_test_result = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_SELFTEST_RESULT_ADDR);

	/* System Error (see section 4.3.59)
	---------------------------------
	0 = No error
	1 = Peripheral initialization error
	2 = System initialization error
	3 = Self test result failed
	4 = Register map value out of range
	5 = Register map address out of range
	6 = Register map write error
	7 = BNO low power mode not available for selected operat ion mode
	8 = Accelerometer power mode not available
	9 = Fusion algorithm configuration error
	A = Sensor configuration error */

	if (system_error != 0)
		*system_error = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_SYS_ERR_ADDR);

	delay(200);
}

void IMU::getRevInfo(adafruit_bno055_rev_info_t* info)
{
	uint8_t a, b;

	memset(info, 0, sizeof(adafruit_bno055_rev_info_t));

	/* Check the accelerometer revision */
	info->accel_rev = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_ACCEL_REV_ID_ADDR);

	/* Check the magnetometer revision */
	info->mag_rev = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_MAG_REV_ID_ADDR);

	/* Check the gyroscope revision */
	info->gyro_rev = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_GYRO_REV_ID_ADDR);

	/* Check the SW revision */
	info->bl_rev = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_BL_REV_ID_ADDR);

	a = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_SW_REV_ID_LSB_ADDR);
	b = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_SW_REV_ID_MSB_ADDR);
	info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}

void IMU::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
	uint8_t calData = (uint8_t) wiringPiI2CReadReg8(fd, BNO055_CALIB_STAT_ADDR);
	if (sys != NULL) {
		*sys = (calData >> 6) & 0x03;
	}
	if (gyro != NULL) {
		*gyro = (calData >> 4) & 0x03;
	}
	if (accel != NULL) {
		*accel = (calData >> 2) & 0x03;
	}
	if (mag != NULL) {
		*mag = calData & 0x03;
	}
}

int8_t IMU::getTemp(void)
{
	int8_t temp = (int8_t)(wiringPiI2CReadReg8(fd, BNO055_TEMP_ADDR));
	if ((temp >= -40) && (temp <= 85))
		return temp;
	else
		return (uint8_t) 0xFF;
}

IMU::Vector3 IMU::getVector(adafruit_vector_type_t vector_type)
{
	Vector3 xyz;
	uint8_t buffer[6];
	memset(buffer, 0, 6);

	int16_t x, y, z;
	x = y = z = 0;

	/* Read vector data (6 bytes) */
	wiringPiI2CWrite(fd, vector_type);
	if (read(fd, buffer, 6) == 6)
	{
		x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
		y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
		z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
	}

	/* Convert the value to an appropriate range (section 3.6.4) */
	/* and assign the value to the Vector type */
	switch (vector_type)
	{
	case VECTOR_MAGNETOMETER:
		/* 1uT = 16 LSB */
	case VECTOR_GYROSCOPE:
		/* 1dps = 16 LSB */
	case VECTOR_EULER:
		/* 1 degree = 16 LSB */
		xyz.x = ((float)x) / 16.0f;
		xyz.y = ((float)y) / 16.0f;
		xyz.z = ((float)z) / 16.0f;
		break;
	case VECTOR_ACCELEROMETER:
	case VECTOR_LINEARACCEL:
	case VECTOR_GRAVITY:
		/* 1m/s^2 = 100 LSB */
		xyz.x = ((float)x) / 100.0f;
		xyz.y = ((float)y) / 100.0f;
		xyz.z = ((float)z) / 100.0f;
		break;
	}

	return xyz;
}

IMU::Quaternion IMU::getQuat(void)
{
	uint8_t buffer[8];
	memset(buffer, 0, 8);

	int16_t x, y, z, w;
	x = y = z = w = 0;

	/* Read quat data (8 bytes) */
	/* Read vector data (6 bytes) */
	wiringPiI2CWrite(fd, BNO055_QUATERNION_DATA_W_LSB_ADDR);
	if (read(fd, buffer, 8) == 8)
	{
		w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
		x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
		y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
		z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);
	}

	/* Assign to Quaternion */
	/* See http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
	3.6.5.5 Orientation (Quaternion)  */
	const float scale = (1.0 / (1 << 14));
	Quaternion quat;

	quat.w = scale * w;
	quat.x = scale * x;
	quat.y = scale * y;
	quat.z = scale * z;

	return quat;
}

bool IMU::getSensorOffsets(uint8_t* calibData)
{
	uint8_t system, gyro, accel, mag;

	if (isFullyCalibrated(&system, &gyro, &accel, &mag))
	{
		adafruit_bno055_opmode_t lastMode = _mode;
		setMode(OPERATION_MODE_CONFIG);

		wiringPiI2CWrite(fd, ACCEL_OFFSET_X_LSB_ADDR);
		read(fd, calibData, NUM_BNO055_OFFSET_REGISTERS);

		setMode(lastMode);
		return true;
	}
	return false;
}

bool IMU::getSensorOffsets(adafruit_bno055_offsets_t &offsets_type)
{
	uint8_t system, gyro, accel, mag;

	if (isFullyCalibrated(&system, &gyro, &accel, &mag))
	{
		adafruit_bno055_opmode_t lastMode = _mode;
		setMode(OPERATION_MODE_CONFIG);
		delay(25);

		offsets_type.accel_offset_x = (wiringPiI2CReadReg8(fd, ACCEL_OFFSET_X_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, ACCEL_OFFSET_X_LSB_ADDR));
		offsets_type.accel_offset_y = (wiringPiI2CReadReg8(fd, ACCEL_OFFSET_Y_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, ACCEL_OFFSET_Y_LSB_ADDR));
		offsets_type.accel_offset_z = (wiringPiI2CReadReg8(fd, ACCEL_OFFSET_Z_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, ACCEL_OFFSET_Z_LSB_ADDR));

		offsets_type.gyro_offset_x = (wiringPiI2CReadReg8(fd, GYRO_OFFSET_X_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, GYRO_OFFSET_X_LSB_ADDR));
		offsets_type.gyro_offset_y = (wiringPiI2CReadReg8(fd, GYRO_OFFSET_Y_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, GYRO_OFFSET_Y_LSB_ADDR));
		offsets_type.gyro_offset_z = (wiringPiI2CReadReg8(fd, GYRO_OFFSET_Z_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, GYRO_OFFSET_Z_LSB_ADDR));

		offsets_type.mag_offset_x = (wiringPiI2CReadReg8(fd, MAG_OFFSET_X_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, MAG_OFFSET_X_LSB_ADDR));
		offsets_type.mag_offset_y = (wiringPiI2CReadReg8(fd, MAG_OFFSET_Y_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, MAG_OFFSET_Y_LSB_ADDR));
		offsets_type.mag_offset_z = (wiringPiI2CReadReg8(fd, MAG_OFFSET_Z_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, MAG_OFFSET_Z_LSB_ADDR));

		offsets_type.accel_radius = (wiringPiI2CReadReg8(fd, ACCEL_RADIUS_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, ACCEL_RADIUS_LSB_ADDR));
		offsets_type.mag_radius = (wiringPiI2CReadReg8(fd, MAG_RADIUS_MSB_ADDR) << 8) | (wiringPiI2CReadReg8(fd, MAG_RADIUS_LSB_ADDR));

		setMode(lastMode);
		return true;
	}
	return false;
}

void IMU::setSensorOffsets(const uint8_t* calibData)
{
	adafruit_bno055_opmode_t lastMode = _mode;
	setMode(OPERATION_MODE_CONFIG);
	delay(25);

	/* A writeLen() would make this much cleaner */
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_X_LSB_ADDR, calibData[6]);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_X_MSB_ADDR, calibData[7]);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_Y_LSB_ADDR, calibData[8]);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_Y_MSB_ADDR, calibData[9]);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_Z_LSB_ADDR, calibData[10]);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_Z_MSB_ADDR, calibData[11]);

	wiringPiI2CWriteReg8(fd, MAG_OFFSET_X_LSB_ADDR, calibData[12]);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_X_MSB_ADDR, calibData[13]);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_Y_LSB_ADDR, calibData[14]);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_Y_MSB_ADDR, calibData[15]);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_Z_LSB_ADDR, calibData[16]);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_Z_MSB_ADDR, calibData[17]);

	wiringPiI2CWriteReg8(fd, ACCEL_RADIUS_LSB_ADDR, calibData[18]);
	wiringPiI2CWriteReg8(fd, ACCEL_RADIUS_MSB_ADDR, calibData[19]);

	wiringPiI2CWriteReg8(fd, MAG_RADIUS_LSB_ADDR, calibData[20]);
	wiringPiI2CWriteReg8(fd, MAG_RADIUS_MSB_ADDR, calibData[21]);

	setMode(lastMode);
}

void IMU::setSensorOffsets(const adafruit_bno055_offsets_t &offsets_type)
{
	adafruit_bno055_opmode_t lastMode = _mode;
	setMode(OPERATION_MODE_CONFIG);
	delay(25);

	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
	wiringPiI2CWriteReg8(fd, ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
	wiringPiI2CWriteReg8(fd, GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

	wiringPiI2CWriteReg8(fd, MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
	wiringPiI2CWriteReg8(fd, MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

	wiringPiI2CWriteReg8(fd, ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
	wiringPiI2CWriteReg8(fd, ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

	wiringPiI2CWriteReg8(fd, MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
	wiringPiI2CWriteReg8(fd, MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

	setMode(lastMode);
}

bool IMU::isFullyCalibrated(uint8_t* system, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
	getCalibration(system, gyro, accel, mag);
	if (*system < 3 || *gyro < 3 || *accel < 3 || *mag < 3)
		return false;

	return true;
}

float IMU::radians(float angle)
{
	return ((angle * (float) M_PI) / 180.0f);
}

float IMU::degress(float angle)
{
	return ((angle * 180.0f) / (float) M_PI);
}
