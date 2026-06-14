#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <Arduino.h>
#include "gps.h"
#include "drive.h"

struct GeoPoint { double lat; double lon; };

class WaypointNavigator {
public:
    WaypointNavigator(Drive& drive);
    void begin();
    // set next goal
    void setGoal(double lat, double lon, double radiusMeters=2.0);
    // provide current gps data and returns true when goal reached
    // update now accepts current GPS and heading (degrees) and returns true when reached
    bool update(const GPSData& current, float headingDegrees, float dt);

private:
    Drive& driveRef;
    GeoPoint goal;
    double goalRadius;
    bool hasGoal;
    unsigned long lastUpdate;

    double distanceToGoal(const GeoPoint& a, const GeoPoint& b);
    double bearingToGoal(const GeoPoint& a, const GeoPoint& b);
    double normalizeAngle(double ang);
    // heading PID
    float headingKp = 2.0f;
    float headingKi = 0.01f;
    float headingKd = 0.1f;
    float headingIntegral = 0.0f;
    float headingPrevError = 0.0f;
    float desiredHeading = 0.0f;
    // recovery
    double lastDistance = -1.0;
    unsigned long stuckStart = 0;
    bool inRecovery = false;
    unsigned long recoveryStart = 0;
};

#endif
