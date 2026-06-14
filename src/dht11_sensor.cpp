#include "dht11_sensor.h"
#include "config.h"

DHT11Sensor::DHT11Sensor(int sensorPin, uint8_t type) : dht(sensorPin, type), pin(sensorPin), delayMS(1000) {}

void DHT11Sensor::init() {
    dht.begin();
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    delayMS = sensor.min_delay / 1000;
}

float DHT11Sensor::getTemperature() {
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        return NAN;
    }
    return event.temperature;
}

float DHT11Sensor::getHumidity() {
    sensors_event_t event;
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        return NAN;
    }
    return event.relative_humidity;
}

void DHT11Sensor::printData() {
    delay(delayMS);
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
        Serial.println("Error reading temperature!");
    } else {
        Serial.print("Temperature: ");
        Serial.print(event.temperature);
        Serial.println(" C");
    }
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        Serial.println("Error reading humidity!");
    } else {
        Serial.print("Humidity: ");
        Serial.print(event.relative_humidity);
        Serial.println(" %");
    }
}
