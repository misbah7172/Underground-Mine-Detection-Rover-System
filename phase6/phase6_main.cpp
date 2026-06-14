#include <Arduino.h>
#include "gps.h"
#include "navigator.h"
#include "motor.h"
#include "drive.h"
#include "config.h"
#include "imu.h"

// motor channels
#define CHAN_ML_A 0
#define CHAN_ML_B 1
#define CHAN_MR_A 2
#define CHAN_MR_B 3

Motor motorL(MOTOR_L_PWM_A, MOTOR_L_PWM_B, CHAN_ML_A, CHAN_ML_B);
Motor motorR(MOTOR_R_PWM_A, MOTOR_R_PWM_B, CHAN_MR_A, CHAN_MR_B);
Drive drive(motorL, motorR);

GPSModule gps(Serial2);
WaypointNavigator navigator(drive);
IMUFusion imu;

// Example goal - set to test coordinates; replace with actual target
const double GOAL_LAT = 37.4219983;
const double GOAL_LON = -122.084;

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Phase 6: Waypoint Autonomous Movement - Demo");
    drive.begin();
    gps.begin(9600);
    navigator.begin();
    if (!imu.begin()) {
        Serial.println("IMU init failed: fusion unavailable");
    } else {
        Serial.println("IMU fusion initialized");
    }
    navigator.setGoal(GOAL_LAT, GOAL_LON, 5.0);
}

void loop() {
    static unsigned long lastMicros = micros();
    unsigned long nowMicros = micros();
    float dt = (nowMicros - lastMicros) / 1000000.0f;
    if (dt <= 0.0f) dt = 0.05f;
    lastMicros = nowMicros;

    gps.feed();
    GPSData d = gps.getData();
    if (d.valid) {
        imu.update();
        float heading = imu.getHeading();
        bool arrived = navigator.update(d, heading, dt);
        if (arrived) {
            Serial.println("Arrived at goal");
            // stop and wait
            drive.stop();
            while (1) delay(1000);
        }
    }
    // non-blocking delay - small sleep
    delay(50);
}
