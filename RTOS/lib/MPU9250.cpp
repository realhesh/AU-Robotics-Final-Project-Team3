#include "MPU9250.h"
#include <math.h>

MPUYaw::MPUYaw(uint8_t mpu_addr, uint8_t mag_addr, uint8_t sda, uint8_t scl)
    : mpuAddress(mpu_addr), magAddress(mag_addr), sda_pin(sda), scl_pin(scl) {
    calibrated = false;
    yaw = gyro_yaw = mag_heading = 0.0;
    init_gz = init_mag_heading = 0.0;
    lastTime = 0;
    alpha = 0.98;  
    mag_decl = 0.0;//set to electra
    
    for (int i = 0; i < 3; i++) hard_iron[i] = 0.0;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            soft_iron[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
}

bool MPUYaw::begin() {
    Wire.begin(sda_pin, scl_pin, 400000);
    setupMPU();
    setupMag();
    delay(1000);
    lastTime = millis();
    return true;
}

void MPUYaw::writeRegister(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t MPUYaw::readRegister(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, (uint8_t)1);
    return Wire.read();
}

void MPUYaw::readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* dest) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, count);
    for (uint8_t i = 0; i < count; i++) dest[i] = Wire.read();
}

void MPUYaw::setupMPU() {
    writeRegister(mpuAddress, 0x6B, 0x00); // Wake up
    writeRegister(mpuAddress, 0x1A, 0x03); 
    writeRegister(mpuAddress, 0x1B, 0x18); 
    writeRegister(mpuAddress, 0x1C, 0x10); 
    writeRegister(mpuAddress, 0x37, 0x02); // Bypass enable for magnetometer
}

void MPUYaw::setupMag() {
    writeRegister(magAddress, 0x0A, 0x00); // Power down
    delay(10);
    writeRegister(magAddress, 0x0A, 0x16); // Continuous measurement mode 2
    delay(10);
}

float MPUYaw::readGyroZ() {
    uint8_t rawData[2];
    readBytes(mpuAddress, 0x47, 2, rawData);
    int16_t gz = ((int16_t)rawData[0] << 8) | rawData[1];
    return gz * GYRO_SCALE;
}

void MPUYaw::readAccel(float &ax, float &ay, float &az) {
    uint8_t buffer[6];
    readBytes(mpuAddress, 0x3B, 6, buffer);
    int16_t x = ((int16_t)buffer[0] << 8) | buffer[1];
    int16_t y = ((int16_t)buffer[2] << 8) | buffer[3];
    int16_t z = ((int16_t)buffer[4] << 8) | buffer[5];
    ax = x * ACCEL_SCALE;
    ay = y * ACCEL_SCALE;
    az = z * ACCEL_SCALE;
}

void MPUYaw::readMag(float &mx, float &my, float &mz) {
    if (!(readRegister(magAddress, 0x02) & 0x01)) return;
    uint8_t buffer[7];
    readBytes(magAddress, 0x03, 7, buffer);
    int16_t x = ((int16_t)buffer[1] << 8) | buffer[0];
    int16_t y = ((int16_t)buffer[3] << 8) | buffer[2];
    int16_t z = ((int16_t)buffer[5] << 8) | buffer[4];
    mx = x * MAG_SCALE;
    my = y * MAG_SCALE;
    mz = z * MAG_SCALE;
}

float MPUYaw::wrapAngle(float angle) {
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

void MPUYaw::calibrateGyro() {
    Serial.println("Calibrating Gyro...");
    const int samples = 200;
    float sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += readGyroZ();
        delay(5);
    }
    init_gz = sum / samples;
    Serial.print("Gyro bias: "); Serial.println(init_gz);
}

void MPUYaw::calibrateMag() {
    Serial.println("Calibrating Magnetometer...");
    const int samples = 100;
    float sum_heading = 0;
    for (int i = 0; i < samples; i++) {
        float mx, my, mz;
        readMag(mx, my, mz);
        // Apply hard and soft iron calibration
        float hi_cal[3] = { mx - hard_iron[0], my - hard_iron[1], mz - hard_iron[2] };
        float mx_cal = soft_iron[0][0]*hi_cal[0] + soft_iron[0][1]*hi_cal[1] + soft_iron[0][2]*hi_cal[2];
        float my_cal = soft_iron[1][0]*hi_cal[0] + soft_iron[1][1]*hi_cal[1] + soft_iron[1][2]*hi_cal[2];
        
        sum_heading += atan2(my_cal, mx_cal) * 180.0 / M_PI;
        delay(10);
    }
    init_mag_heading = sum_heading / samples;
    Serial.print("Initial heading: "); Serial.println(init_mag_heading);
}

bool MPUYaw::update() {
    if (!calibrated) {
        calibrateGyro();
        calibrateMag();
        calibrated = true;
        lastTime = millis();
        return false;
    }

    
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    
    
    if (dt > 1.0 || dt <= 0) dt = 0.01;

    // Integrate gyro
    float gyroZ = readGyroZ() - init_gz;
    gyro_yaw += gyroZ * dt;
    gyro_yaw = wrapAngle(gyro_yaw);

    // Read magnetometer
    float mx, my, mz;
    readMag(mx, my, mz);
    
    // Apply hard iron calibration
    float hi_cal[3] = { mx - hard_iron[0], my - hard_iron[1], mz - hard_iron[2] };
    
    // Apply soft iron calibration
    mx = soft_iron[0][0]*hi_cal[0] + soft_iron[0][1]*hi_cal[1] + soft_iron[0][2]*hi_cal[2];
    my = soft_iron[1][0]*hi_cal[0] + soft_iron[1][1]*hi_cal[1] + soft_iron[1][2]*hi_cal[2];
    mz = soft_iron[2][0]*hi_cal[0] + soft_iron[2][1]*hi_cal[1] + soft_iron[2][2]*hi_cal[2];

    // Read accelerometer
    float ax, ay, az;
    readAccel(ax, ay, az);
    
    // Calculate roll and pitch
    float roll = atan2(ay, az);
    float pitch = atan2(-ax, sqrt(ay*ay + az*az));
    
    // not sure what this does but it seems important
  // TODO: understand it later
    float mx_comp = mx * cos(pitch) + mz * sin(pitch);
    float my_comp = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
    
    mag_heading = atan2(my_comp, mx_comp) * 180.0 / M_PI;
    mag_heading += mag_decl;
    mag_heading -= init_mag_heading;
    mag_heading = wrapAngle(mag_heading);

    // Complementary filter
    yaw = alpha * gyro_yaw + (1.0 - alpha) * mag_heading;
    yaw = wrapAngle(yaw);

    return true;
}

float MPUYaw::getYaw() { 
    return yaw; 
}

void MPUYaw::setAlpha(float a) {
    if (a >= 0.0 && a <= 1.0) alpha = a;
}

void MPUYaw::setMagneticDeclination(float decl) {
    mag_decl = decl;
}

void MPUYaw::setHardIronOffset(float x, float y, float z) {
    hard_iron[0] = x;
    hard_iron[1] = y;
    hard_iron[2] = z;
}

void MPUYaw::setSoftIronMatrix(float matrix[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            soft_iron[i][j] = matrix[i][j];
        }
    }
}
