#ifndef MPU9250_H
#define MPU9250_H

#include <Wire.h>

class MPUYaw {
public:
    MPUYaw(uint8_t mpu_addr = 0x68, uint8_t mag_addr = 0x0C, uint8_t sda = 21, uint8_t scl = 22);
    bool begin();
    bool update();
    float getYaw();

private:
    uint8_t mpuAddress, magAddress;
    uint8_t sda_pin, scl_pin;

    
    bool calibrated;
    float init_gz;
    float init_mag_heading;

    
    float yaw, gyro_yaw, mag_heading;

    
    const float hard_iron[3] = { 0, 0, 0};
    const float soft_iron[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} };
    const float mag_decl = 0; // set to electra
    const float alpha = 0.975;

    
    const float GYRO_SCALE = 2000.0 / 32768.0; 
    const float MAG_SCALE = 4912.0 / 32760.0; 

    
    void writeRegister(uint8_t addr, uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t addr, uint8_t reg);
    void readBytes(uint8_t addr, uint8_t reg, uint8_t count, uint8_t* dest);

    void setupMPU();
    void setupMag();
    void calibrateGyro();
    void calibrateMag();

    float readGyroZ();
    void readMag(float &mx, float &my, float &mz);
};

#endif
