#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <Arduino.h>

class SdLogger {
public:
    SdLogger();
    bool begin();
    void logLine(const String& line);
private:
    bool initialized;
};

#endif
