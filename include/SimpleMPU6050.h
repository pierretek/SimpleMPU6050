#pragma once

#include <Arduino.h>
#include <Wire.h>

//Weight given to gyro vs accel when blending angle estimates
//Closer to 1.0 = trust gyro more (smoother, but drifts more over time)
//Closer to 0.0 = trust accel more (less drift, but noisier)
#define MPU_GYRO_FACTOR 0.90

// noise threshold in deg/s for the gyroscope, tune this to ignore small noise
#define MPU_GYRO_NOISE_THRESHOLD 0.3  

//Handy struct for 3d elements
template <typename T>
struct Vector3D {
	T x{};
	T y{};
	T z{};
};

//full scale range of gyroscope in deg/s
enum GyroScale { 
	MPU_GYRO_SCALE_250DS,
	MPU_GYRO_SCALE_500DS,
	MPU_GYRO_SCALE_1000DS,
	MPU_GYRO_SCALE_2000DS,
};

//full scale range of accelerometer in g's
enum AccelScale { 
	MPU_ACCEL_SCALE_2G,
	MPU_ACCEL_SCALE_4G,
	MPU_ACCEL_SCALE_8G,
	MPU_ACCEL_SCALE_16G,
};

class MPU6050 {

	private:

	const uint8_t MPU_ADDR{0x68};

	enum MPU6050Registers {
		ACCEL_XOUT_H = 0x3B, //Acceleration
		ACCEL_XOUT_L = 0x3C,
		ACCEL_YOUT_H = 0x3D,
		ACCEL_YOUT_L = 0x3E,
		ACCEL_ZOUT_H = 0x3F,
		ACCEL_ZOUT_L = 0x40,

		TEMP_OUT_H = 0x41, //Temperature
		TEMP_OUT_L = 0x42,

		GYRO_XOUT_H = 0x43, //Gyroscope
		GYRO_XOUT_L = 0x44,
		GYRO_YOUT_H = 0x45,
		GYRO_YOUT_L = 0x46,
		GYRO_ZOUT_H = 0x47,
		GYRO_ZOUT_L = 0x48,

		PWR_MGMT_1 = 0x6B, //Config and management
		SMPRT_DIV = 0x19,
		CONFIG = 0x1A,
		ACCEL_CONFIG = 0x1C,
		GYRO_CONFIG = 0x1B
	};

	//config values
	GyroScale gyroScale{};
	AccelScale accelScale{};

	//Error offsets
	float errorGyroX{};
	float errorGyroY{};
	float errorGyroZ{};

	float errorAccelX{};
	float errorAccelY{};
	float errorAccelZ{};

	//needed for angle calculations
	enum GravityAxis { X, Y, Z };
	GravityAxis gravityAxis{Z};
	Vector3D<double> currentAngle{};
	unsigned long lastAngleUpdate{};

	bool enableDLPF{};

	//Helper functions to read/write from registers
	bool registerRead(const uint8_t addr, uint8_t* data, const uint8_t numBytes);
	bool registerWrite(const uint8_t addr, const uint8_t* data, const uint8_t numBytes);

	public:

	//Setup functions
	void begin(bool autoCalibrate = false);
	void calibrate(const int samples = 200);
	void setDPLF(uint8_t bandwidth);
	void setSampleRate(uint16_t rate);

	//Set sensor sensitivies (only accepts 0,1,2,3)
	void setMaxGyroScale(const GyroScale maxScale);
	void setMaxAccelScale(const AccelScale maxScale);

	//Reading raw sensor values
	int16_t readRawGyroX();
	int16_t readRawGyroY();
	int16_t readRawGyroZ();
	Vector3D<int16_t> readRawGyro();

	int16_t readRawAccelX();
	int16_t readRawAccelY();
	int16_t readRawAccelZ();
	Vector3D<int16_t> readRawAccel();

	int16_t readRawTemp();

	//Reading scaled sensor values
	double readGyroX();
	double readGyroY();
	double readGyroZ();
	Vector3D<double> readGyro();

	double readAccelX();
	double readAccelY();
	double readAccelZ();
	Vector3D<double> readAccel();

	double readTemp();

	//Angle calculation functions
	void updateAngles();
	double readAngleX();
	double readAngleY();
	double readAngleZ();

	double readRoll();
	double readPitch();
	double readYaw();

	Vector3D<double> readAngles();

	//Miscelanous helper functions
	int16_t getOneG();
	double scaleGyro(int16_t reading);
	double scaleAccel(int16_t reading);

};
