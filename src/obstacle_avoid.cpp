#include "obstacle_avoid.h"
#include "config.h"

ObstacleAvoider::ObstacleAvoider(UltrasonicSensor& left, UltrasonicSensor& right, float thresholdCm)
    : usLeft(left), usRight(right), thresh(thresholdCm), lastCheck(0) {}

void ObstacleAvoider::begin() {
    // nothing to init beyond sensors
}

bool ObstacleAvoider::checkAndGetAvoidance(int16_t &leftOut, int16_t &rightOut) {
    if (millis() - lastCheck < checkInterval) return false;
    lastCheck = millis();

    float ldist = usLeft.getDistance();
    float rdist = usRight.getDistance();

    // default no avoidance
    leftOut = 0; rightOut = 0;

    bool leftBlocked = (ldist > 0 && ldist < thresh);
    bool rightBlocked = (rdist > 0 && rdist < thresh);

    if (leftBlocked && rightBlocked) {
        // both blocked: back off and turn
        leftOut = -120; rightOut = -120;
        // after backing, turn right
    } else if (leftBlocked) {
        // turn right
        leftOut = 120; rightOut = -120;
    } else if (rightBlocked) {
        // turn left
        leftOut = -120; rightOut = 120;
    } else {
        return false;
    }

    return true;
}
