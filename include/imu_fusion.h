#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#include <Arduino.h>
#include "imu_mpu9250.h"
#include "MadgwickAHRS.h"

class IMUFusion {
public:
    IMUFusion();
    bool begin();
    // call periodically to update filter; returns true when valid
    bool update();
    // heading in degrees 0..360
    float getHeading();
    // expose raw sensor reads for diagnostics
    bool readRaw(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz);
    // magnetometer calibration (non-blocking): start, stop, progress
    bool startMagCalibration(unsigned int durationSeconds = 10);
    void stopMagCalibration();
    bool isMagCalibrating() const;
    // returns 0..100 progress percent
    int getMagCalProgress() const;
    void loadMagCalibration();
    void saveMagCalibration();
private:
    MPU9250 sensor;
    Madgwick filter;
    unsigned long lastMicros;
    float lastHeading;
    float magOffsetX = 0.0f;
    float magOffsetY = 0.0f;
    float magOffsetZ = 0.0f;
    // calibration state
    bool magCalActive = false;
    unsigned long magCalEndMs = 0;
    unsigned long magCalStartMs = 0;
    float magMinX, magMinY, magMinZ;
    float magMaxX, magMaxY, magMaxZ;
    unsigned long magSampleCount = 0;
};

#endif
