#ifndef METAL_DETECTOR_H
#define METAL_DETECTOR_H

#include <Arduino.h>

class MetalDetector {
public:
    MetalDetector(int pin);
    void begin();
    bool isMetalDetected();
    int readAnalog();
    // debounced detection with smoothing
    bool checkDebounced();
    // log event with GPS timestamp
    void logEvent(const char* tag, double lat, double lon);
private:
    int pin;
};

#endif
