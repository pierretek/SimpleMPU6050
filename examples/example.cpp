//Simple program to demonstrate reading values from the SimpleMPU6050 library
//Reads the accelerometer, gyroscope, and angle values then prints them to the serial monitor

#include <Arduino.h>
#include "SimpleMPU6050.h"

#define COOLDOWN 500 //cooldown in ms
uint64_t lastTime{};

MPU6050 mpu; //Declare the mpu object

void printDebugInfo();

void setup() {
	Serial.begin(9600);

	//wake up the mpu
	mpu.begin();

	//set the digital lowpass filer to smoothen readings [0-6]
	mpu.setDPLF(3);

	//Calibrate the mpu to find error offset values
	//DO NOT MOVE WHILE CALIBRATING
	mpu.calibrate();
}


void loop() {

	//Call this as frequently as possible to keep the angle readings accurate
	mpu.updateAngles();

	if (millis() - lastTime > COOLDOWN) {
		lastTime = millis();

		//readAccel() returns a 3D struct of the accelerometer values in g's
		auto a = mpu.readAccel();
		Serial.printf("Accel: X: %.3f Y: %.3f Z: %.3f\n",
			a.x, a.y, a.z);

		//readGyro() returns a 3D struct of the gyroscope values in deg/s
		auto g = mpu.readGyro();
		Serial.printf("Gyro: X: %.3f Y: %.3f Z: %.3f\n",
			g.x, g.y, g.z);

		//readTemp() returns the temperature reading in C
		Serial.printf("Temp: %.3fC\n",
			mpu.readTemp());

		//readAngles() returns a 3D struct of the current angles in degrees
		auto angles = mpu.readAngles();
		Serial.printf("Angles X: %.2f Y: %.2f Z: %.2f\n",
			angles.x, angles.y, angles.z);

	}

}
