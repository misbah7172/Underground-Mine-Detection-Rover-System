#include "imu_fusion.h"
#include <Preferences.h>
#include <Arduino.h>

#if USE_IMU

IMUFusion::IMUFusion() : sensor(), filter(0.08f), lastMicros(0), lastHeading(0.0f) {}

bool IMUFusion::begin() {
    if (!sensor.begin()) return false;
    filter.begin(100.0f);
    lastMicros = micros();
    loadMagCalibration();
    return true;
}

bool IMUFusion::update() {
    unsigned long now = micros();
    float dt = (now - lastMicros) / 1000000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    lastMicros = now;

    float ax, ay, az, gx, gy, gz, mx, my, mz;
    if (!sensor.readAccelGyro(ax, ay, az, gx, gy, gz)) return false;
    sensor.readMag(mx, my, mz); // optional

    // if calibration active, collect samples
    if (magCalActive) {
        if (mx < magMinX) magMinX = mx;
        if (my < magMinY) magMinY = my;
        if (mz < magMinZ) magMinZ = mz;
        if (mx > magMaxX) magMaxX = mx;
        if (my > magMaxY) magMaxY = my;
        if (mz > magMaxZ) magMaxZ = mz;
        magSampleCount++;
        // finish if time elapsed
        if (millis() >= magCalEndMs) {
            magCalActive = false;
            // compute offsets
            magOffsetX = (magMaxX + magMinX) * 0.5f;
            magOffsetY = (magMaxY + magMinY) * 0.5f;
            magOffsetZ = (magMaxZ + magMinZ) * 0.5f;
            saveMagCalibration();
        }
    }

    // apply magnetometer calibration offsets
    mx -= magOffsetX; my -= magOffsetY; mz -= magOffsetZ;

    // Madgwick expects gyro in deg/s, accel in g, mag in arbitrary units
    filter.update(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
    lastHeading = filter.getYaw();
    if (lastHeading < 0) lastHeading += 360.0f;
    return true;
}

float IMUFusion::getHeading() {
    return lastHeading;
}

bool IMUFusion::readRaw(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz) {
    bool ok = sensor.readAccelGyro(ax, ay, az, gx, gy, gz);
    sensor.readMag(mx, my, mz);
    return ok;
}

bool IMUFusion::calibrateMagnetometer(unsigned int durationSeconds) {
    // legacy blocking method: start, collect, save
    return startMagCalibration(durationSeconds);
}

bool IMUFusion::startMagCalibration(unsigned int durationSeconds) {
    if (durationSeconds < 1) durationSeconds = 1;
    magCalActive = true;
    magCalStartMs = millis();
    magCalEndMs = magCalStartMs + (unsigned long)durationSeconds * 1000UL;
    magMinX = magMinY = magMinZ = 1e6f;
    magMaxX = magMaxY = magMaxZ = -1e6f;
    magSampleCount = 0;
    return true;
}

void IMUFusion::stopMagCalibration() {
    if (!magCalActive) return;
    magCalActive = false;
    if (magSampleCount > 0) {
        magOffsetX = (magMaxX + magMinX) * 0.5f;
        magOffsetY = (magMaxY + magMinY) * 0.5f;
        magOffsetZ = (magMaxZ + magMinZ) * 0.5f;
        saveMagCalibration();
    }
}

bool IMUFusion::isMagCalibrating() const { return magCalActive; }

int IMUFusion::getMagCalProgress() const {
    if (!magCalActive) return 0;
    unsigned long now = millis();
    if (now >= magCalEndMs) return 100;
    unsigned long total = magCalEndMs - magCalStartMs;
    if (total == 0) return 0;
    unsigned long elapsed = now - magCalStartMs;
    int pct = (int)((elapsed * 100UL) / total);
    if (pct < 0) pct = 0; if (pct > 100) pct = 100;
    return pct;
}

void IMUFusion::loadMagCalibration() {
    Preferences prefs;
    prefs.begin("magcal", true);
    magOffsetX = prefs.getFloat("ox", 0.0f);
    magOffsetY = prefs.getFloat("oy", 0.0f);
    magOffsetZ = prefs.getFloat("oz", 0.0f);
    prefs.end();
}

void IMUFusion::saveMagCalibration() {
    Preferences prefs;
    prefs.begin("magcal", false);
    prefs.putFloat("ox", magOffsetX);
    prefs.putFloat("oy", magOffsetY);
    prefs.putFloat("oz", magOffsetZ);
    prefs.end();
}

#else

// Stubs when IMU is disabled
IMUFusion::IMUFusion() {}
bool IMUFusion::begin() { return false; }
bool IMUFusion::update() { return false; }
float IMUFusion::getHeading() { return 0.0f; }
bool IMUFusion::readRaw(float& ax, float& ay, float& az, float& gx, float& gy, float& gz, float& mx, float& my, float& mz) { return false; }
bool IMUFusion::startMagCalibration(unsigned int durationSeconds) { return false; }
void IMUFusion::stopMagCalibration() {}
bool IMUFusion::isMagCalibrating() const { return false; }
int IMUFusion::getMagCalProgress() const { return 0; }
void IMUFusion::loadMagCalibration() {}
void IMUFusion::saveMagCalibration() {}

#endif
