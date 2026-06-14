#include "imu.h"

IMUCompass::IMUCompass() : mag(12345), offset(0.0f) {}

bool IMUCompass::begin() {
    Wire.begin();
    if (!mag.begin()) {
        return false;
    }
    return true;
}

float IMUCompass::readHeading() {
    sensors_event_t event;
    mag.getEvent(&event);
    float heading = atan2(event.magnetic.y, event.magnetic.x) * 180.0 / M_PI;
    heading += offset;
    if (heading < 0) heading += 360.0;
    if (heading >= 360.0) heading -= 360.0;
    return heading;
}

void IMUCompass::calibrate() {
    // placeholder: user should collect min/max and compute offsets
    offset = 0.0f;
}
