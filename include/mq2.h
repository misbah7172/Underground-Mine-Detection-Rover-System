#ifndef MQ2_H
#define MQ2_H

#include <Arduino.h>

class MQ2Sensor {
public:
    MQ2Sensor(int pin);
    void begin();
    float readPpm(); // approximate value scaled 0..1
    bool isSmokeDetected(float threshold=0.4);
private:
    int pin;
    int readRaw();
};

#endif
