#ifndef POWER_H
#define POWER_H

#include <Arduino.h>

class PowerManager {
public:
    PowerManager(int adcPin, float dividerRatio, float fullV, float lowV);
    void begin();
    float readVoltage();
    float readPercentage();
    bool isBatteryLow();
private:
    int pin;
    float ratio;
    float fullVoltage;
    float lowVoltage;
};

#endif
