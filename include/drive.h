#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>
#include "motor.h"

class Drive {
public:
    Drive(Motor& leftMotor, Motor& rightMotor);
    void begin();
    // left and right speeds -255..255
    void setSpeed(int16_t left, int16_t right);
    void stop();
    void emergencyStop();

private:
    Motor& left;
    Motor& right;
    unsigned long lastCommandMillis;
    const unsigned long commandTimeout = 5000; // stop if no commands
};

#endif
