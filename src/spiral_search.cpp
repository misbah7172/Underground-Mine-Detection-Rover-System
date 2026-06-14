#include "spiral_search.h"
#include <math.h>

SpiralSearch::SpiralSearch(double centerLat, double centerLon, double stepMeters, double maxRadiusMeters)
    : cLat(centerLat), cLon(centerLon), step(stepMeters), maxR(maxRadiusMeters) {}

double SpiralSearch::metersToDegreesLat(double meters) {
    return meters / 111320.0;
}

double SpiralSearch::metersToDegreesLon(double meters, double lat) {
    return meters / (111320.0 * cos(lat * M_PI / 180.0));
}

std::vector<Waypoint> SpiralSearch::generate() {
    std::vector<Waypoint> list;
    double a = 0.0; // angle
    double theta = 0.0;
    double dr = step / (2.0 * M_PI); // radial increment per radian
    while (true) {
        double r = dr * theta; // radius in meters
        if (r > maxR) break;
        double x = r * cos(theta);
        double y = r * sin(theta);
        double dLat = metersToDegreesLat(y);
        double dLon = metersToDegreesLon(x, cLat + dLat);
        Waypoint w{ cLat + dLat, cLon + dLon };
        list.push_back(w);
        theta += 0.5; // step angle
    }
    return list;
}
