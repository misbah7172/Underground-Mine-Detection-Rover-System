#ifndef ROVER_H
#define ROVER_H

#include "ultrasonic.h"
#include "dht11_sensor.h"

struct RoverData {
    float distance;
    float temperature;
    float humidity;
};

class MineDetectionRover {
private:
    UltrasonicSensor* ultrasonic;
    DHT11Sensor* dht11;
    
public:
    MineDetectionRover();
    ~MineDetectionRover();
    void init();
    RoverData collectData();
    void printData(RoverData data);
    void updateActuators(RoverData data);
};

#endif
