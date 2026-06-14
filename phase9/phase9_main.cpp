#include <Arduino.h>
#include "spiral_search.h"
#include "navigator.h"
#include "gps.h"
#include "motor.h"
#include "drive.h"
#include "imu_fusion.h"
#include "config.h"

// motor channels
#define CHAN_ML_A 0
#define CHAN_ML_B 1
#define CHAN_MR_A 2
#define CHAN_MR_B 3

Motor motorL(MOTOR_L_PWM_A, MOTOR_L_PWM_B, CHAN_ML_A, CHAN_ML_B);
Motor motorR(MOTOR_R_PWM_A, MOTOR_R_PWM_B, CHAN_MR_A, CHAN_MR_B);
Drive drive(motorL, motorR);

GPSModule gps(Serial2);
IMUFusion imu;
WaypointNavigator navigator(drive);

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Phase 9: Spiral Search Demo");
    drive.begin();
    gps.begin(9600);
    imu.begin();
    navigator.begin();
}

void loop() {
    // Wait until GPS fix
    gps.feed();
    GPSData d = gps.getData();
    if (!d.valid) {
        Serial.println("Waiting for GPS fix...");
        delay(2000);
        return;
    }

    Serial.println("Generating spiral waypoints around current position...");
    SpiralSearch spiral(d.latitude, d.longitude, 5.0, 50.0); // 5m step, 50m max
    auto wps = spiral.generate();
    Serial.print("Waypoints: "); Serial.println(wps.size());

    for (size_t i=0;i<wps.size();i++) {
        Waypoint w = wps[i];
        navigator.setGoal(w.lat, w.lon, 3.0);
        Serial.print("Heading to WP "); Serial.println(i);
        bool reached = false;
        unsigned long lastMicros = micros();
        while (!reached) {
            unsigned long now = micros();
            float dt = (now - lastMicros) / 1000000.0f;
            lastMicros = now;
            gps.feed();
            GPSData cur = gps.getData();
            if (cur.valid) {
                imu.update();
                float heading = imu.getHeading();
                reached = navigator.update(cur, heading, dt);
            }
            delay(100);
        }
        Serial.print("Reached WP "); Serial.println(i);
        delay(500);
    }

    Serial.println("Spiral complete");
    while (1) delay(1000);
}
