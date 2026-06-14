#include "navigator.h"
#include <math.h>

#define EARTH_RADIUS_M 6371000.0

WaypointNavigator::WaypointNavigator(Drive& drive) : driveRef(drive), hasGoal(false), lastUpdate(0) {}

void WaypointNavigator::begin() {
    headingIntegral = 0.0f;
    headingPrevError = 0.0f;
}

void WaypointNavigator::setGoal(double lat, double lon, double radiusMeters) {
    goal.lat = lat;
    goal.lon = lon;
    goalRadius = radiusMeters;
    hasGoal = true;
}

bool WaypointNavigator::update(const GPSData& current, float headingDegrees, float dt) {
    if (!hasGoal || !current.valid) return false;

    GeoPoint cur{current.latitude, current.longitude};
    double dist = distanceToGoal(cur, goal);

    if (dist <= goalRadius) {
        driveRef.stop();
        hasGoal = false;
        return true;
    }
    double desiredBearing = bearingToGoal(cur, goal);
    desiredHeading = desiredBearing;

    // recovery check: if not making progress (distance not decreasing) for a period, initiate recovery
    if (lastDistance < 0) {
        lastDistance = dist;
        stuckStart = millis();
    } else {
        if (dist < lastDistance - 0.5) {
            // progress made
            lastDistance = dist;
            stuckStart = millis();
        } else {
            // no progress
            if (millis() - stuckStart > 7000 && !inRecovery) {
                inRecovery = true;
                recoveryStart = millis();
            }
        }
    }

    if (inRecovery) {
        // recovery maneuver: back up and rotate for a short period
        unsigned long t = millis() - recoveryStart;
        if (t < 1200) {
            driveRef.setSpeed(-120, -120); // back up
        } else if (t < 2600) {
            driveRef.setSpeed(120, -120); // spin
        } else {
            inRecovery = false;
            lastDistance = dist;
            stuckStart = millis();
        }
        return false;
    }

    // compute heading error in range -180..180
    double err = desiredHeading - headingDegrees;
    if (err > 180.0) err -= 360.0;
    if (err < -180.0) err += 360.0;

    // PID-style heading correction using local state
    if (dt > 0.0f) {
        headingIntegral += (float)err * dt;
        headingIntegral = constrain(headingIntegral, -100.0f, 100.0f);
    }
    float derivative = (dt > 0.0f) ? ((float)err - headingPrevError) / dt : 0.0f;
    float steer = headingKp * (float)err + headingKi * headingIntegral + headingKd * derivative;
    headingPrevError = (float)err;

    int16_t baseSpeed = 130;
    int16_t left = (int16_t)constrain(baseSpeed + steer, -255, 255);
    int16_t right = (int16_t)constrain(baseSpeed - steer, -255, 255);
    driveRef.setSpeed(left, right);
    return false;
}

double WaypointNavigator::distanceToGoal(const GeoPoint& a, const GeoPoint& b) {
    double lat1 = a.lat * M_PI / 180.0;
    double lat2 = b.lat * M_PI / 180.0;
    double dlat = (b.lat - a.lat) * M_PI / 180.0;
    double dlon = (b.lon - a.lon) * M_PI / 180.0;
    double sin_dlat = sin(dlat/2.0);
    double sin_dlon = sin(dlon/2.0);
    double aa = sin_dlat*sin_dlat + cos(lat1)*cos(lat2)*sin_dlon*sin_dlon;
    double c = 2 * atan2(sqrt(aa), sqrt(1-aa));
    return EARTH_RADIUS_M * c;
}

double WaypointNavigator::bearingToGoal(const GeoPoint& a, const GeoPoint& b) {
    double lat1 = a.lat * M_PI / 180.0;
    double lat2 = b.lat * M_PI / 180.0;
    double dLon = (b.lon - a.lon) * M_PI / 180.0;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(dLon);
    double brng = atan2(y, x) * 180.0 / M_PI;
    return fmod((brng + 360.0), 360.0);
}

double WaypointNavigator::normalizeAngle(double ang) {
    while (ang > 180.0) ang -= 360.0;
    while (ang < -180.0) ang += 360.0;
    return ang;
}
