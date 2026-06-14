#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

class IMUCompass {
public:
    IMUCompass();
    bool begin();
    // returns heading in degrees 0..360
    float readHeading();
    // perform simple calibration routine (placeholder)
    void calibrate();
private:
    Adafruit_HMC5883_Unified mag;
    float offset; // simple offset calibration
};

#endif
