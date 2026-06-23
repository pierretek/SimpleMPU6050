#include "SimpleMPU6050.h"

//forward declarations
static double wrap(double angle, double limit);
static void applyComplementaryFilter(double& a1, double& a2, double& aGrav,
	double acc1, double acc2, double accGrav,
	double g1, double g2, double gGrav,
	double dt, bool isStill);


/*****************************
* ESSENTIALS SECTION
******************************/

//Wakes up the sensor and optionally calibrates it as well
void MPU6050::begin(bool autoCalibrate) {
	Wire.begin();

	//Waking from sleep mode
	Wire.beginTransmission(MPU_ADDR);
	Wire.write(PWR_MGMT_1);
	Wire.write(0x00);
	Wire.endTransmission();

	//calibrates the sensor if the user chooses to
	delay(50);
	if (autoCalibrate) calibrate();
}

//Read a register from the MPU6050
bool MPU6050::registerRead(const uint8_t addr, uint8_t* data, const uint8_t numBytes) {

	Wire.beginTransmission(MPU_ADDR);
	Wire.write(addr);
	Wire.endTransmission(false);

	if (Wire.requestFrom(MPU_ADDR, numBytes) < 1) return false;

	int i{0};
	while (Wire.available()) {
		data[i++] = Wire.read();
	}

	return true;
}

//Write to a register from the MPU6050
bool MPU6050::registerWrite(const uint8_t addr, const uint8_t* data, const uint8_t numBytes) {

	uint8_t dataFrame[numBytes + 1];
	dataFrame[0] = addr;

	for (uint i{1}; i <= numBytes; i++) {
		dataFrame[i] = data[i - 1];
	}

	Wire.beginTransmission(MPU_ADDR);
	Wire.write(dataFrame, numBytes + 1);
	Wire.endTransmission();

	return true;

}


/*****************************
* CONFIGURATION/SETUP SECTION
******************************/

//Finds error offset values and the gravity axis
void MPU6050::calibrate(const int samples) {

	long gyroX{}, gyroY{}, gyroZ{};
	long accelX{}, accelY{}, accelZ{};

	for (uint32_t i{0}; i < samples; i++) {
		accelX += readRawAccelX();
		accelY += readRawAccelY();
		accelZ += readRawAccelZ();

		gyroX += readRawGyroX();
		gyroY += readRawGyroY();
		gyroZ += readRawGyroZ();
		delay(2); // delay between samples
	}


	//calculating the gyroscope error
	errorGyroX = (float)gyroX / samples;
	errorGyroY = (float)gyroY / samples;
	errorGyroZ = (float)gyroZ / samples;

	//calculating the acceleration error
	errorAccelX = (float)accelX / samples;
	errorAccelY = (float)accelY / samples;
	errorAccelZ = (float)accelZ / samples;

	//accounting for gravity
	long maxAccel = max({abs(accelX), abs(accelY), abs(accelZ)});
	int16_t oneG = getOneG();

	if (abs(accelX) == maxAccel) {
		gravityAxis = X;
		errorAccelX -= (accelX > 0) ? oneG : -oneG;
	}
	if (abs(accelY) == maxAccel) {
		gravityAxis = Y;
		errorAccelY -= (accelY > 0) ? oneG : -oneG;
	}
	if (abs(accelZ) == maxAccel) {
		gravityAxis = Z;
		errorAccelZ -= (accelZ > 0) ? oneG : -oneG;
	}

	//printing calibration results
	Serial.printf("--- Sensor Calibrated After %d Samples ---\n", samples);
	Serial.printf("Gyro Err: X: %.2f, Y: %.2f, Z: %.2f\n", errorGyroX, errorGyroY, errorGyroZ);
	Serial.printf("Accel Err: X: %.2f, Y: %.2f, Z: %.2f\n", errorAccelX, errorAccelY, errorAccelZ);

}

//Set the acceleration sensitivity as the +-maximum reading
void MPU6050::setMaxAccelScale(const AccelScale maxScale) {

	uint8_t temp;
	accelScale = maxScale;

	switch (maxScale) {
		case MPU_ACCEL_SCALE_2G:
			temp = 0x00;
			break;
		case MPU_ACCEL_SCALE_4G:
			temp = 0x08;
			break;
		case MPU_ACCEL_SCALE_8G:
			temp = 0x10;
			break;
		case MPU_ACCEL_SCALE_16G:
			temp = 0x18;
			break;
	}

	registerWrite(ACCEL_CONFIG, &temp, 1);
}

//Set the gyroscope sensitivity as the +-maximum reading
void MPU6050::setMaxGyroScale(const GyroScale maxScale) {

	uint8_t temp;
	gyroScale = maxScale;

	switch (maxScale) {
		case MPU_GYRO_SCALE_250DS:
			temp = 0x00;
			break;
		case MPU_GYRO_SCALE_500DS:
			temp = 0x08;
			break;
		case MPU_GYRO_SCALE_1000DS:
			temp = 0x10;
			break;
		case MPU_GYRO_SCALE_2000DS:
			temp = 0x18;
			break;
	}

	registerWrite(GYRO_CONFIG, &temp, 1);
}

//Sets the digital lowpass filter, smoothens readings but adds a delay
//Only accepts values [0-6] inclusive, DPLF is turned off at 0
void MPU6050::setDPLF(uint8_t bandwidth) {

	//rough values, refer to datasheet
	switch (bandwidth) {  	 //Bandwidth(Hz), delay(ms)
		case 0:  	 	 //260, 0
		case 1: 		 //185, 2
		case 2: 	 	 //95,  3
		case 3: 		 //43,  5
		case 4: 	 	 //20,  8
		case 5: 	 	 //10,  13
		case 6:			 //5,   19
			registerWrite(CONFIG, &bandwidth, 1);
			break;

		default: break; //Do nothing if invalid
	}

	enableDLPF = !(bandwidth == 0 || bandwidth == 7);
}

//Sets the sensor sampling rate in Hz, accepts any rate between 1 and max
// DPLF = off, max = 8kHz
// DPLF = on, max = 1kHz
void MPU6050::setSampleRate(uint16_t rate) {
	uint16_t maxRate{enableDLPF ? 1000u : 8000u};

	rate = constrain(rate, 1, maxRate);

	uint8_t temp = maxRate / rate - 1;

	registerWrite(SMPRT_DIV, &temp, 1);
}

/************************
* ACCELERATION SECTION
*************************/

//returns the raw register value of the x-axis accelerometer
int16_t MPU6050::readRawAccelX() {
	uint8_t temp[2];
	registerRead(ACCEL_XOUT_H, temp, 2);
	return(temp[0] << 8 | temp[1]);
}

//returns the raw register value of the y-axis accelerometer
int16_t MPU6050::readRawAccelY() {
	uint8_t temp[2];
	registerRead(ACCEL_YOUT_H, temp, 2);
	return(temp[0] << 8 | temp[1]);
}

//returns the raw register value of the z-axis accelerometer
int16_t MPU6050::readRawAccelZ() {
	uint8_t temp[2];
	registerRead(ACCEL_ZOUT_H, temp, 2);
	return(temp[0] << 8 | temp[1]);
}

//Returns a 3D struct of the raw register values of the accelerometer
Vector3D<int16_t> MPU6050::readRawAccel() {

	Vector3D<int16_t> result{};
	result.x = readRawAccelX();
	result.y = readRawAccelY();
	result.z = readRawAccelZ();

	return result;
}

//Returns the x-axis accelerometer reading in g's
double MPU6050::readAccelX() {

	int16_t reading = readRawAccelX() - errorAccelX;

	return scaleAccel(reading);
}

//Returns the y-axis accelerometer reading in g's
double MPU6050::readAccelY() {

	int16_t reading = readRawAccelY() - errorAccelY;

	return scaleAccel(reading);
}

//Returns the z-axis accelerometer reading in g's
double MPU6050::readAccelZ() {

	int16_t reading = readRawAccelZ() - errorAccelZ;

	return scaleAccel(reading);
}

//Returns a 3D struct of the accelerometer readings in all three axis (g's)
Vector3D<double> MPU6050::readAccel() {
	Vector3D<double> result;

	result.x = readAccelX();
	result.y = readAccelY();
	result.z = readAccelZ();
	return result;
}

/************************
* GYROSCOPE SECTION
*************************/

//returns the raw register value of the x-axis gyroscope
int16_t MPU6050::readRawGyroX() {
	uint8_t temp[2];
	registerRead(GYRO_XOUT_H, temp, 2);
	return(temp[0] << 8 | temp[1]);
}

//returns the raw register value of the y-axis gyroscope
int16_t MPU6050::readRawGyroY() {
	uint8_t temp[2];
	registerRead(GYRO_YOUT_H, temp, 2);
	return(temp[0] << 8 | temp[1]);
}

//returns the raw register value of the z-axis gyroscope
int16_t MPU6050::readRawGyroZ() {
	uint8_t temp[2];
	registerRead(GYRO_ZOUT_H, temp, 2);
	return(temp[0] << 8 | temp[1]);
}

//Returns a 3D struct of the raw register values of the gyroscope
Vector3D<int16_t> MPU6050::readRawGyro() {

	Vector3D<int16_t> result{};
	result.x = readRawGyroX();
	result.y = readRawGyroY();
	result.z = readRawGyroZ();

	return result;
}

//Returns the y-axis gyroscope reading in degrees per second
double MPU6050::readGyroX() {

	int16_t reading = readRawGyroX() - errorGyroX;

	return scaleGyro(reading);

}

//Returns the y-axis gyroscope reading in degrees per second
double MPU6050::readGyroY() {

	int16_t reading = readRawGyroY() - errorGyroY;

	return scaleGyro(reading);
}

//Returns the z-axis gyroscope reading in degrees per second
double MPU6050::readGyroZ() {

	int16_t reading = readRawGyroZ() - errorGyroZ;

	return scaleGyro(reading);
}

//Returns a 3D struct of the gyroscope readings in all three axis (degrees/second)
Vector3D<double> MPU6050::readGyro() {
	Vector3D<double> result;

	result.x = readGyroX();
	result.y = readGyroY();
	result.z = readGyroZ();
	return result;
}

/************************
* TEMPERATURE SECTION
*************************/

//returns the raw register value of the temperature sensor
int16_t MPU6050::readRawTemp() {
	uint8_t temp[2];
	registerRead(TEMP_OUT_H, temp, 2);

	return (temp[0] << 8 | temp[1]);
}

//Returns temperature reading in C
double MPU6050::readTemp() {
	return ((readRawTemp() / 340.0) + 36.53);
}

/*************************
* ANGLE SECTION
**************************/

//Updates the currentAngle using a complementary filter
//call this function as frequently as possible to keep the readings accurate
void MPU6050::updateAngles() {
	unsigned long now = millis();
	if (lastAngleUpdate == 0) { lastAngleUpdate = now; return; }
	double dt = (now - lastAngleUpdate) / 1000.0;
	lastAngleUpdate = now;

	Vector3D<double> g = readGyro();
	if (fabs(g.x) < MPU_GYRO_NOISE_THRESHOLD) g.x = 0;
	if (fabs(g.y) < MPU_GYRO_NOISE_THRESHOLD) g.y = 0;
	if (fabs(g.z) < MPU_GYRO_NOISE_THRESHOLD) g.z = 0;
	bool isStill = (g.x == 0 && g.y == 0 && g.z == 0);

	Vector3D<double> a = readAccel();

	//applying the filter differently depending on which axis is the gravity axis
	switch (gravityAxis) {
		case Z: applyComplementaryFilter(currentAngle.x, currentAngle.y, currentAngle.z, a.y, a.x, a.z, g.x, g.y, g.z, dt, isStill); break;
		case Y: applyComplementaryFilter(currentAngle.x, currentAngle.z, currentAngle.y, a.z, a.x, a.y, g.x, g.z, g.y, dt, isStill); break;
		case X: applyComplementaryFilter(currentAngle.y, currentAngle.z, currentAngle.x, a.z, a.y, a.x, g.y, g.z, g.x, dt, isStill); break;
	}
}

//Returns a 3D struct of the current angles in all three axis (degrees)
Vector3D<double> MPU6050::readAngles() {
	return currentAngle;
}

//Returns the current x-axis angle in degrees 
double MPU6050::readAngleX() {
	return currentAngle.x;
}

//Returns the current y-axis angle in degrees 
double MPU6050::readAngleY() {
	return currentAngle.y;
}

//Returns the current z-axis angle in degrees 
double MPU6050::readAngleZ() {
	return currentAngle.z;
}

//Returns the current yaw angle in degrees 
//This angle is always susceptible to drifting due to sensor limitations
double MPU6050::readYaw() {
	switch (gravityAxis) {
		case X: return currentAngle.x;
		case Y: return currentAngle.y;
		case Z: return currentAngle.z;
		default: return 0;
	}
}

//Returns the current pitch angle in degrees 
double MPU6050::readPitch() {
	switch (gravityAxis) {
		case X: return currentAngle.y;
		case Y: return currentAngle.z;
		case Z: return currentAngle.x;
		default: return 0;
	}
}

//Returns the current roll angle in degrees 
double MPU6050::readRoll() {
	switch (gravityAxis) {
		case X: return currentAngle.z;
		case Y: return currentAngle.x;
		case Z: return currentAngle.y;
		default: return 0;
	}
}

/************************
* MISC HELPER FUNCTIONS
*************************/

//helper function to return the raw accelerometer readings of 1G force
int16_t MPU6050::getOneG() {
	switch (accelScale) {
		case MPU_ACCEL_SCALE_2G: return 16384;
		case MPU_ACCEL_SCALE_4G: return 8192;
		case MPU_ACCEL_SCALE_8G: return 4096;
		case MPU_ACCEL_SCALE_16G: return 2048;
		default: return 16384;
	}
}

//helper function to scale a raw gyroscope reading into degrees/second
double MPU6050::scaleGyro(int16_t reading) {
	switch (gyroScale) {
		case MPU_GYRO_SCALE_250DS: return reading / 131.0;
		case MPU_GYRO_SCALE_500DS: return reading / 65.5;
		case MPU_GYRO_SCALE_1000DS: return reading / 32.8;
		case MPU_GYRO_SCALE_2000DS: return reading / 16.4;
		default:  return reading / 131.0;
	}
}

//helper function to scale a raw accelerometer reading into g's
double MPU6050::scaleAccel(int16_t reading) {
	switch (accelScale) {
		case MPU_ACCEL_SCALE_2G: return reading / 16384.0;
		case MPU_ACCEL_SCALE_4G: return reading / 8192.0;
		case MPU_ACCEL_SCALE_8G: return reading / 4096.0;
		case MPU_ACCEL_SCALE_16G: return reading / 2048.0;
		default: return reading / 16384.0;
	}
}

//helper function to wrap angles
static double wrap(double angle, double limit) {
	while (angle < -limit) angle += 2 * limit;
	while (angle > limit) angle -= 2 * limit;
	return angle;
}

//massive helper function to help with the complementary filter
static void applyComplementaryFilter(double& a1, double& a2, double& aGrav,
	double acc1, double acc2, double accGrav,
	double g1, double g2, double gGrav,
	double dt, bool isStill) {

	double sg = accGrav < 0 ? -1 : 1;
	double accelA1 = atan2(acc1, sg * sqrt(accGrav * accGrav + acc2 * acc2)) * RAD_TO_DEG;
	double accelA2 = -atan2(acc2, sqrt(accGrav * accGrav + acc1 * acc1)) * RAD_TO_DEG;
	const double T = MPU_GYRO_FACTOR;
	a1 = wrap(T * (accelA1 + wrap(a1 + g1 * dt - accelA1, 180)) + (1 - T) * accelA1, 180);
	a2 = wrap(T * (accelA2 + wrap(a2 + sg * g2 * dt - accelA2, 90)) + (1 - T) * accelA2, 90);
	if (!isStill) aGrav += gGrav * dt;
}


