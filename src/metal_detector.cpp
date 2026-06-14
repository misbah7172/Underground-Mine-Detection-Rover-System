#include "metal_detector.h"
#include "config.h"

MetalDetector::MetalDetector(int p) : pin(p) {}

void MetalDetector::begin() {
    pinMode(pin, INPUT);
}

bool MetalDetector::isMetalDetected() {
    return digitalRead(pin) == HIGH;
}

int MetalDetector::readAnalog() {
    // if analog-capable, allow reading
    return analogRead(pin);
}

bool MetalDetector::checkDebounced() {
    static unsigned long lastChange = 0;
    static bool lastState = false;
    bool current = isMetalDetected();
    unsigned long now = millis();
    const unsigned long DEBOUNCE_MS = 150;
    if (current != lastState) {
        lastChange = now;
    }
    if ((now - lastChange) > DEBOUNCE_MS) {
        if (current != lastState) {
            lastState = current;
            if (lastState) return true; // rising edge
        }
    }
    return false;
}

void MetalDetector::logEvent(const char* tag, double lat, double lon) {
    // Simple serial CSV log: timestamp,tag,lat,lon,raw
    unsigned long t = millis();
    int raw = readAnalog();
    Serial.print(t); Serial.print(",");
    Serial.print(tag); Serial.print(",");
    Serial.print(lat,6); Serial.print(",");
    Serial.print(lon,6); Serial.print(",");
    Serial.println(raw);
}
