#include "rover.h"
#include "config.h"
#include <Arduino.h>

MineDetectionRover::MineDetectionRover() {
    ultrasonic = new UltrasonicSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    dht11 = new DHT11Sensor(DHT11_PIN, DHT_TYPE);
}

MineDetectionRover::~MineDetectionRover() {
    delete ultrasonic;
    delete dht11;
}

void MineDetectionRover::init() {
    Serial.begin(BAUD_RATE);
    delay(1000);
    
    Serial.println("Initializing Mine Detection Rover...");
    ultrasonic->init();
    dht11->init();
    // Configure actuator pins
    pinMode(HEATER_LED_PIN, OUTPUT);
    pinMode(COOLER_LED_PIN, OUTPUT);
    digitalWrite(HEATER_LED_PIN, LOW);
    digitalWrite(COOLER_LED_PIN, LOW);
    Serial.println("Initialization complete");
}

RoverData MineDetectionRover::collectData() {
    RoverData data;
    data.distance = ultrasonic->getDistance();
    data.temperature = dht11->getTemperature();
    data.humidity = dht11->getHumidity();
    return data;
}

void MineDetectionRover::printData(RoverData data) {
    Serial.println("--- Sensor Data ---");
    Serial.print("Distance: ");
    Serial.print(data.distance);
    Serial.println(" cm");
    Serial.print("Temperature: ");
    Serial.print(data.temperature);
    Serial.println(" C");
    Serial.print("Humidity: ");
    Serial.print(data.humidity);
    Serial.println(" %");
}

void MineDetectionRover::updateActuators(RoverData data) {
    if (isnan(data.temperature)) {
        return;
    }
    if (data.temperature < TEMP_HEATER_THRESHOLD) {
        digitalWrite(HEATER_LED_PIN, HIGH);
        digitalWrite(COOLER_LED_PIN, LOW);
        Serial.println("Actuator: Heater ON, Cooler OFF");
    } else if (data.temperature > TEMP_COOLER_THRESHOLD) {
        digitalWrite(HEATER_LED_PIN, LOW);
        digitalWrite(COOLER_LED_PIN, HIGH);
        Serial.println("Actuator: Heater OFF, Cooler ON");
    } else {
        digitalWrite(HEATER_LED_PIN, LOW);
        digitalWrite(COOLER_LED_PIN, LOW);
        Serial.println("Actuator: Heater OFF, Cooler OFF");
    }
}
