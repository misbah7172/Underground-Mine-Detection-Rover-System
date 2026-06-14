#include "sd_logger.h"
#include "config.h"
#include <SD.h>
#include <SPI.h>

SdLogger::SdLogger() : initialized(false) {}

bool SdLogger::begin() {
    if (initialized) return true;
    if (!SD.begin(SD_CS_PIN)) {
        initialized = false;
        return false;
    }
    initialized = true;
    return true;
}

void SdLogger::logLine(const String& line) {
    if (!initialized) return;
    const char* filename = "telemetry.log";
    // rotate if needed
    File f0 = SD.open(filename, FILE_READ);
    if (f0) {
        uint32_t size = f0.size();
        f0.close();
        if (size >= SD_LOG_MAX_BYTES) {
            // rotate backups
            for (int i = SD_LOG_BACKUPS - 1; i >= 0; --i) {
                String src = String(filename) + (i == 0 ? "" : "." + String(i));
                String dst = String(filename) + "." + String(i + 1);
                if (SD.exists(dst.c_str())) SD.remove(dst.c_str());
                if (SD.exists(src.c_str())) SD.rename(src.c_str(), dst.c_str());
            }
        }
    }

    File f = SD.open(filename, FILE_APPEND);
    if (!f) return;
    f.println(line);
    f.close();
}
