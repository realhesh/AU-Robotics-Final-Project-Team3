#ifndef MPU6050_YAW_H
#define MPU6050_YAW_H

#include <Wire.h>
#include <Arduino.h>

class MPU6050Yaw {
public:
    // Algorithm selection
    bool useMadgwick = true;

    // Madgwick parameters
    float beta = 0.27;

    // Mahony parameters
    float Kp = 0.5;
    float Ki = 0.0;

    // Gyro calibration samples
    static const int NUM_CALIBRATION_SAMPLES = 2000;

    MPU6050Yaw(bool useMadgwick = true);
    void begin();
    void update();
    float getYaw();

private:
    static const uint8_t MPU6050_ADDR = 0x68;

    // Quaternion
    float q0, q1, q2, q3;
    float eInt[3];

    // Gyro calibration offsets
    float gyroXoffset, gyroYoffset, gyroZoffset;

    // Timing
    unsigned long lastUpdate;
    float deltaT;

    // Core functions
    void readMPU6050(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
    void calibrateGyro();
    void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az);
    void MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az);
    float invSqrt(float x);
};

#endif
