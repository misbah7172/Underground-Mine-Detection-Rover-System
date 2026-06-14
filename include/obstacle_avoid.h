#ifndef OBSTACLE_AVOID_H
#define OBSTACLE_AVOID_H

#include <Arduino.h>
#include "ultrasonic.h"

class ObstacleAvoider {
public:
    ObstacleAvoider(UltrasonicSensor& left, UltrasonicSensor& right, float thresholdCm);
    void begin();
    // returns true if avoidance action requested; fills left/right speed suggestions
    bool checkAndGetAvoidance(int16_t &leftOut, int16_t &rightOut);

private:
    UltrasonicSensor& usLeft;
    UltrasonicSensor& usRight;
    float thresh;
    unsigned long lastCheck;
    const unsigned long checkInterval = 100;
};

#endif
