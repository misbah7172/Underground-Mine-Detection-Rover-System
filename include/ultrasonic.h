#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class UltrasonicSensor {
private:
    int trigPin;
    int echoPin;
    
public:
    UltrasonicSensor(int trig, int echo);
    void init();
    float getDistance();
    void printDistance();
};

#endif
