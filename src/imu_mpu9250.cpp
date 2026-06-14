#include "imu_mpu9250.h"

// MPU9250 / AK8963 minimal driver
// Note: not feature-complete; intended for teaching and basic fusion usage.

#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_INT_PIN_CFG 0x37
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_GYRO_XOUT_H 0x43

#define AK8963_ADDR 0x0C
#define AK8963_CNTL1 0x0A
#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03

MPU9250::MPU9250(uint8_t address) : addr(address) {}

void MPU9250::writeRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

uint8_t MPU9250::readRegister(uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)addr, 1);
    if (Wire.available()) return Wire.read();
    return 0;
}

bool MPU9250::readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)addr, (int)len);
    uint8_t i=0;
    while (Wire.available() && i<len) buf[i++] = Wire.read();
    return (i==len);
}

bool MPU9250::begin() {
    Wire.begin();
    // wake up
    writeRegister(MPU9250_PWR_MGMT_1, 0x00);
    delay(100);
    // enable I2C bypass to access AK8963 directly
    writeRegister(MPU9250_INT_PIN_CFG, 0x02);
    delay(10);

    // configure AK8963 for continuous measurement 16-bit 100Hz (0x16)
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(AK8963_CNTL1);
    Wire.write(0x16);
    Wire.endTransmission();
    delay(10);
    return true;
}

static int16_t toInt16(uint8_t hi, uint8_t lo) {
    return (int16_t)((hi << 8) | lo);
}

bool MPU9250::readAccelGyro(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    uint8_t buf[14];
    if (!readBytes(MPU9250_ACCEL_XOUT_H, buf, 14)) return false;
    int16_t axr = toInt16(buf[0], buf[1]);
    int16_t ayr = toInt16(buf[2], buf[3]);
    int16_t azr = toInt16(buf[4], buf[5]);
    int16_t gxr = toInt16(buf[8], buf[9]);
    int16_t gyr = toInt16(buf[10], buf[11]);
    int16_t gzr = toInt16(buf[12], buf[13]);

    // convert to g and deg/s using common scalings (assuming default ±2g, ±250dps)
    ax = (float)axr / 16384.0f;
    ay = (float)ayr / 16384.0f;
    az = (float)azr / 16384.0f;
    gx = (float)gxr / 131.0f;
    gy = (float)gyr / 131.0f;
    gz = (float)gzr / 131.0f;
    return true;
}

bool MPU9250::readMag(float& mx, float& my, float& mz) {
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(AK8963_ST1);
    Wire.endTransmission(false);
    Wire.requestFrom((int)AK8963_ADDR, 7);
    if (Wire.available() < 7) return false;
    uint8_t st1 = Wire.read();
    if (!(st1 & 0x01)) return false;
    uint8_t xl = Wire.read();
    uint8_t xh = Wire.read();
    uint8_t yl = Wire.read();
    uint8_t yh = Wire.read();
    uint8_t zl = Wire.read();
    uint8_t zh = Wire.read();
    uint8_t tmps = Wire.read();
    int16_t xr = (int16_t)((xh << 8) | xl);
    int16_t yr = (int16_t)((yh << 8) | yl);
    int16_t zr = (int16_t)((zh << 8) | zl);
    // raw units; scaling can be applied if calibration available
    mx = (float)xr;
    my = (float)yr;
    mz = (float)zr;
    return true;
}
