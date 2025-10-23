#include "MPU6050Yaw.h"

MPU6050Yaw::MPU6050Yaw(bool useMadgwickFlag) {
    useMadgwick = useMadgwickFlag;
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
    eInt[0] = eInt[1] = eInt[2] = 0.0f;
    gyroXoffset = gyroYoffset = gyroZoffset = 0.0f;
    lastUpdate = 0;
    deltaT = 0;
}

void MPU6050Yaw::begin() {
    Wire.begin();

    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    // Gyro ±250°/s
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // Accel ±2g
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission(true);

    delay(100);
    calibrateGyro();
    lastUpdate = micros();
}

void MPU6050Yaw::update() {
    int16_t ax, ay, az, gx, gy, gz;
    readMPU6050(&ax, &ay, &az, &gx, &gy, &gz);

    float accelX = ax / 16384.0;
    float accelY = ay / 16384.0;
    float accelZ = az / 16384.0;

    float gyroX = (gx / 131.0 - gyroXoffset) * DEG_TO_RAD;
    float gyroY = (gy / 131.0 - gyroYoffset) * DEG_TO_RAD;
    float gyroZ = (gz / 131.0 - gyroZoffset) * DEG_TO_RAD;

    unsigned long now = micros();
    deltaT = (now - lastUpdate) / 1000000.0f;
    lastUpdate = now;

    if (useMadgwick)
        MadgwickUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
    else
        MahonyUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
}

float MPU6050Yaw::getYaw() {
    return atan2(2.0f * (q0 * q3 + q1 * q2),
                 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}

void MPU6050Yaw::readMPU6050(int16_t* ax, int16_t* ay, int16_t* az,
                             int16_t* gx, int16_t* gy, int16_t* gz) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);

    *ax = Wire.read() << 8 | Wire.read();
    *ay = Wire.read() << 8 | Wire.read();
    *az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // skip temp
    *gx = Wire.read() << 8 | Wire.read();
    *gy = Wire.read() << 8 | Wire.read();
    *gz = Wire.read() << 8 | Wire.read();
}

void MPU6050Yaw::calibrateGyro() {
    long sumX = 0, sumY = 0, sumZ = 0;

    for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        readMPU6050(&ax, &ay, &az, &gx, &gy, &gz);
        sumX += gx; sumY += gy; sumZ += gz;
        delay(2);
    }

    gyroXoffset = (sumX / (float)NUM_CALIBRATION_SAMPLES) / 131.0;
    gyroYoffset = (sumY / (float)NUM_CALIBRATION_SAMPLES) / 131.0;
    gyroZoffset = (sumZ / (float)NUM_CALIBRATION_SAMPLES) / 131.0;
}

void MPU6050Yaw::MadgwickUpdate(float gx, float gy, float gz,
                                float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2;
    float q0q0, q1q1, q2q2, q3q3;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;

    q0 += qDot1 * deltaT;
    q1 += qDot2 * deltaT;
    q2 += qDot3 * deltaT;
    q3 += qDot4 * deltaT;

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void MPU6050Yaw::MahonyUpdate(float gx, float gy, float gz,
                              float ax, float ay, float az) {
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez;
    float qa, qb, qc;

    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    if (Ki > 0.0f) {
        eInt[0] += ex * deltaT;
        eInt[1] += ey * deltaT;
        eInt[2] += ez * deltaT;
    } else {
        eInt[0] = eInt[1] = eInt[2] = 0.0f;
    }

    gx += Kp * ex + Ki * eInt[0];
    gy += Kp * ey + Ki * eInt[1];
    gz += Kp * ez + Ki * eInt[2];

    qa = q0; qb = q1; qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz) * (0.5f * deltaT);
    q1 += (qa * gx + qc * gz - q3 * gy) * (0.5f * deltaT);
    q2 += (qa * gy - qb * gz + q3 * gx) * (0.5f * deltaT);
    q3 += (qa * gz + qb * gy - qc * gx) * (0.5f * deltaT);

    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

float MPU6050Yaw::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}
