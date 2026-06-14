#ifndef IMU_MPU9250_H
#define IMU_MPU9250_H

#include <Arduino.h>
#include <Wire.h>

class MPU9250 {
public:
    MPU9250(uint8_t address = 0x68);
    bool begin();
    bool readAccelGyro(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
    bool readMag(float& mx, float& my, float& mz);
private:
    uint8_t addr;
    void writeRegister(uint8_t reg, uint8_t val);
    uint8_t readRegister(uint8_t reg);
    bool readBytes(uint8_t reg, uint8_t* buf, uint8_t len);
};

#endif
