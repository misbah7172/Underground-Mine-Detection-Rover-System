#ifndef SPIRAL_SEARCH_H
#define SPIRAL_SEARCH_H

#include <Arduino.h>
#include <vector>
#include "navigator.h"

struct Waypoint { double lat; double lon; };

class SpiralSearch {
public:
    SpiralSearch(double centerLat, double centerLon, double stepMeters, double maxRadiusMeters);
    // generate waypoints (GPS lat/lon) in an outward Archimedean spiral approximated by waypoints
    std::vector<Waypoint> generate();
private:
    double cLat, cLon;
    double step; // meters per revolution increment
    double maxR;
    // helpers
    double metersToDegreesLat(double meters);
    double metersToDegreesLon(double meters, double lat);
};

#endif
