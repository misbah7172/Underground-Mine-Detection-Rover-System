#include <Arduino.h>
#include "gps.h"
#include "config.h"

GPSModule gps(Serial2);
unsigned long lastPrint = 0;

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Phase 5: GPS Parsing and Navigation - Test");
    gps.begin(9600);
}

void loop() {
    gps.feed();
    if (millis() - lastPrint > 2000) {
        lastPrint = millis();
        GPSData d = gps.getData();
        if (d.valid) {
            Serial.print("Fix: "); Serial.print(d.latitude, 6); Serial.print(", "); Serial.print(d.longitude, 6);
            Serial.print("  Sats:"); Serial.print(d.satellites);
            Serial.print("  Hdop:"); Serial.print(d.hdop);
            Serial.print("  Speed(kph):"); Serial.print(d.speedKph);
            Serial.println();
        } else {
            Serial.println("No GPS fix yet");
        }
    }
}
