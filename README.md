# SimpleMPU6050 &nbsp; 
<!-- <img src="https://github.com/user-attachments/assets/f0baf001-bbd4-4cb8-9915-fd82dad5cd95" width="40" height="40" align="top" /> -->

A simple and lightweight library for reading values from a MPU6050 IMU
 
 ## Table of Contents
- [SimpleMPU6050  ](#simplempu6050-)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Features](#features)
  - [Limitations](#limitations)
- [Usage](#usage)
  - [Brief Example](#brief-example)
  - [Configuring the MPU6050](#configuring-the-mpu6050)
    - [Digital Lowpass Filter](#digital-lowpass-filter)
    - [Sensor Sampling rate](#sensor-sampling-rate)
    - [Maximum Sensor Reading Scale](#maximum-sensor-reading-scale)
    - [Error Offsets and Gravity Direction](#error-offsets-and-gravity-direction)
  - [Reading the MPU6050](#reading-the-mpu6050)
    - [Reading Acceleration](#reading-acceleration)
    - [Reading Gyroscope](#reading-gyroscope)
    - [Reading Temperature](#reading-temperature)
    - [Reading angles](#reading-angles)
- [Footnotes](#footnotes)
- [External Resources](#external-resources)


## Overview
The MPU6050 is a popular 6‑DOF motion sensor that measures acceleration and angular velocity in all three dimensions. This library aims to provide a simple interface for reading these values while remaining compatible with all Arduino compatible platforms.

## Features 
- Can read the **gyroscope** and **accelerometer** values in `deg/s` and `g's` respectively
- Can configure the **sampling rate** of **both sensors**
- Can enable the **digital lowpass filter** to smooth readings
- Can configure the **maximum range** of the **gyroscope** and **accelerometer**
- Can calibrate the sensor to find error offset values and automatically subtract them from future readings
- Can calculate the **current angle** of the IMU across all three dimensions
  

## Limitations 
- Only supports one MPU as of now, with default I2C address (`0x68`)
- Cannot interface with other components as an I2C master
- Cannot use the FIFO buffer
- Cannot use the interrupt pin
- Cannot use power-saving modes
- Cannot use cycle mode or the low‑power accelerometer mode


# Usage 

## Brief Example
```c++
#include "SimpleMPU6050.h"

MPU6050 mpu; //declare the sensor

void setup() {
  Serial.begin(9600);

  //wake up and calibrate the sensor
  mpu.begin();
  mpu.calibrate();
}

void loop() {
  //Call this frequently to keep the angle readings accurate
  mpu.updateAngles();

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
``` 

## Configuring the MPU6050
This library supports configuring several key parameters of the MPU6050 sensor. 
The configurable parameters include:
- **Digital lowpass filter**
- **Sensor sampling rate (Hz)**
- **Maximum sensor reading scale**
- **Error Offsets and Gravity Direction**


### Digital Lowpass Filter
The **digital lowpass filter** can be configured to filter out **high frequency noise** from the sensor readings. You must use `.setDLPF(uint8_t bandwidth)`, where `bandwidth` is between 0-6 inclusive. The table for the bandwidth values is shown below. The digital lowpass filter also introduces a *small delay* to the sensor readings.

| Bandwidth Value | Filter Bandwidth (Hz) | Reading Delay (ms) |
| --------------- | --------------------- | ------------------ |
| 0               | <260                  | 0                  |
| 1               | <185                  | 2                  |
| 2               | <95                   | 3                  |
| 3               | <43                   | 5                  |
| 4               | <20                   | 8                  |
| 5               | <10                   | 13                 |
| 6               | <5                    | 19                 |

> [!WARNING] 
> The numbers in this table are simplified, refer to the register map for the actual specific values.[^1]

### Sensor Sampling rate 
The **sampling rate** of the gyroscope and the accelerometer sensors can be adjusted using the `.setSampleRate(uint16_t rate)` function. Where `rate` is the sampling frequency of the sensors in `Hz`.

> [!NOTE]
> The maximum sampling rate of the sensor depends on whether the **digital lowpass filter** is on. If the digital lowpass filter is set to **0** (default), the filter is **off** and the maximum sampling rate is **8kHz**. Otherwise, the filter is on, and the maximum sampling rate is **1kHz**. Refer to the register map for more details. [^2]

### Maximum Sensor Reading Scale
To adjust the sensitivity and the maximum readings of the sensor, you can adjust the **reading scales** of the **gyroscope** and the **accelerometer**. This is done using `.setMaxGyroScale(GyroScale maxScale)` and `.setMaxAccelScale(AccelScale maxScale)` respectively. Where `GyroScale` and `AccelScale` are enumerators which allow one of four different values to be passed to the sensor. The scaling values are displayed in the tables below

Gyroscope reading scale
| GyroScale Enum        | Reading Scale (deg/s) |
| --------------------- | --------------------- |
| MPU_GYRO_SCALE_250DS  | ±250                  |
| MPU_GYRO_SCALE_500DS  | ±500                  |
| MPU_GYRO_SCALE_1000DS | ±1000                 |
| MPU_GYRO_SCALE_2000DS | ±2000                 |

Accelerometer reading scale
| AccelScale Enum     | Reading Scale (g) |
| ------------------- | ----------------- |
| MPU_ACCEL_SCALE_2G  | ±2                |
| MPU_ACCEL_SCALE_4G  | ±4                |
| MPU_ACCEL_SCALE_8G  | ±8                |
| MPU_ACCEL_SCALE_16G | ±16               |

### Error Offsets and Gravity Direction
One of the most important features of this library is the `.calibrate()` function. This function samples the sensors hundreds of times while the **sensor** is at **rest**, and finds the average error offset to subtract from future readings. This helps keep future readings **accurate** by eliminating the sensor error offset. Additionally, this function automatically detects the **direction** of the **gravity vector** which is **crucial** for the **angle readings**.
> [!WARNING]
> 1. To ensure the offsets are accurate, `.calibrate()` must be called **after** using either `.setMaxGyroScale()` or `.setMaxAccelScale()`
> 2. **Do not** move the sensor while it is calibrating

## Reading the MPU6050
This library provides a simple interface for reading sensor values from the MPU6050. The **gyroscope**, **accelerometer**, and **temperature** readings are directly accessible in their raw and scaled forms. Additionally, the current angle values in all three dimensions are accessible, including the roll, pitch, and yaw values.

> [!IMPORTANT]
> This library provides a simple struct called `Vector3D` to handle sending **3 dimensional information**. `Vector3D` has three members; **x**, **y**, and **z**. Here is a simplified Example:
> ``` 
> Vector3D a = readSensor();
> //a.x = x value of sensor
> //a.y = y value of sensor
> //a.z = z value of sensor
> ```

### Reading Acceleration
The **raw acceleration** values can be read using these four functions, they are unitless values.  
The final function returns the raw readings across **all three dimensions**.
```c++
int16_t readRawAccelX();
int16_t readRawAccelY();
int16_t readRawAccelZ();
Vector3D<int16_t> readRawGyro();
```

The **scaled acceleration** values can be read using these four functions, they are returned in `g's`.  
The final function returns the scaled readings across **all three dimensions**.
```c++
double readAccelX();
double readAccelY();
double readAccelZ();
Vector3D<double> readAccel();
```

### Reading Gyroscope
The **raw gyroscope** values can be read using these four functions, they are unitless values.  
The final function returns the raw readings across **all three dimensions**.
```c++
int16_t readRawGyroX();
int16_t readRawGyroY();
int16_t readRawGyroZ();
Vector3D<int16_t> readRawGyro();
```

The **scaled gyroscope** values can be read using these four functions, they are returned in `deg/s`.  
The final function returns the scaled readings across **all three dimensions**.
```c++
double readGyroX();
double readGyroY();
double readGyroZ();
Vector3D<double> readGyro();
```
### Reading Temperature
The raw and scaled temperature readings are accessible from the following two functions. Keep in mind the raw value is **unitless**, while the scaled value is in **°C**. The formula for computing the scaled value from the raw value can be found in the datasheet.[^3]
```c++
int16_t readRawTemp();
double readTemp();
```

### Reading angles
This library provides some useful functions for reading the **current angle** of the sensor across all three dimensions. This is by far the most complex feature and makes use of a Complementary Filter to achieve usable readings. 

To improve **reading accuracy**, there are two key configurable values in the header file of this library:
```c++
//Weight given to gyro vs accel when blending angle estimates
//Closer to 1.0 = trust gyro more (smoother, but drifts more over time)
//Closer to 0.0 = trust accel more (less drift, but noisier)
#define MPU_GYRO_FACTOR 0.90

// noise threshold in deg/s for the gyroscope, tune this to ignore small noise
#define MPU_GYRO_NOISE_THRESHOLD 0.3  
```

Next, the following functions allow you to access the current angle readings of the sensor in degrees:
```c++
double readAngleX();
double readAngleY();
double readAngleZ();
Vector3D<double> readAngles();

double readRoll();
double readPitch();
double readYaw();
```
> [!WARNING]
> 1. The angle calculations depend on the gravity axis determined from the `.calibrate()` function, make sure you calibrate your sensor in `setup()`
> 2. Call `.updateAngles()` in `loop()` as frequently as possible to improve the reading accuracy


# Footnotes
[^1]: Details for `CONFIG` register ([link](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf#page=13))  
[^2]: Details for `SMPRT_DIV` register ([link](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf#page=12))  
[^3]: Details for the temperature reading formula ([link](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf#page=31))


# External Resources
- [MPU6050 Register map and Descriptions](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf)
- [Extensive MPU6050 Article](https://mjwhite8119.github.io/Robots/mpu6050)
- [Arduino and MPU6050 Tutorial](https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/#h-overview)

---
_thanks for reading_


