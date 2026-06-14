#ifndef GPS_H
#define GPS_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

struct GPSData {
    double latitude;
    double longitude;
    double altitude;
    double speedKph;
    double courseDeg;
    double hdop;
    int satellites;
    bool valid;
};

class GPSModule {
public:
    GPSModule(HardwareSerial& serial);
    void begin(unsigned long baud);
    void feed();
    GPSData getData();
    bool hasFix();
    // Filtering and calibration
    void setFilterWindowSize(size_t n);
    void resetFilter();
private:
    HardwareSerial& hwSerial;
    TinyGPSPlus gps;
    unsigned long lastFeedMillis;
    // simple sliding window filter
    size_t windowSize = 5;
    size_t idx = 0;
    bool bufferFull = false;
    double latBuf[16];
    double lonBuf[16];
};

#endif
