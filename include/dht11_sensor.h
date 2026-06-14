#ifndef DHT11_SENSOR_H
#define DHT11_SENSOR_H

#include <Adafruit_Sensor.h>
#include <DHT_U.h>

class DHT11Sensor {
private:
    DHT_Unified dht;
    int pin;
    uint32_t delayMS;
    
public:
    DHT11Sensor(int sensorPin, uint8_t type);
    void init();
    float getTemperature();
    float getHumidity();
    void printData();
};

#endif
