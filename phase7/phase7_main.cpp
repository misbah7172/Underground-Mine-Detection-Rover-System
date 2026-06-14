#include <Arduino.h>
#include "metal_detector.h"
#include "config.h"
#include "gps.h"

MetalDetector detector(METAL_DETECTOR_PIN);
GPSModule gps(Serial2);
unsigned long lastCheck = 0;

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Phase 7: Metal Detector Integration - Test");
    detector.begin();
    gps.begin(9600);
}

void loop() {
    if (millis() - lastCheck > 500) {
        lastCheck = millis();
        gps.feed();
        GPSData d = gps.getData();
        bool hit = detector.checkDebounced();
        int raw = detector.readAnalog();
        if (hit) {
            Serial.println("Metal detected (debounced)");
            double lat = d.valid ? d.latitude : 0.0;
            double lon = d.valid ? d.longitude : 0.0;
            detector.logEvent("METAL", lat, lon);
        }
        Serial.print("Analog raw: "); Serial.println(raw);
    }
}
