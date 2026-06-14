#include <Arduino.h>
#include "motor.h"
#include "drive.h"
#include "config.h"

// LEDC channels assignment (4 channels used)
#define CHAN_ML_A 0
#define CHAN_ML_B 1
#define CHAN_MR_A 2
#define CHAN_MR_B 3

// Create motor objects
Motor motorL(MOTOR_L_PWM_A, MOTOR_L_PWM_B, CHAN_ML_A, CHAN_ML_B);
Motor motorR(MOTOR_R_PWM_A, MOTOR_R_PWM_B, CHAN_MR_A, CHAN_MR_B);
Drive drive(motorL, motorR);

unsigned long lastPeriodicMillis = 0;
const unsigned long statusInterval = 1000;

void safeStopIfEStop() {
    if (digitalRead(E_STOP_PIN) == LOW) {
        drive.emergencyStop();
        Serial.println("E-STOP triggered: motors stopped");
        while (digitalRead(E_STOP_PIN) == LOW) {
            delay(50);
        }
        Serial.println("E-STOP released");
    }
}

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Phase 1: Basic Rover Movement");
    drive.begin();
    drive.stop();
}

// Non-blocking simple control: read serial commands or run simple demo
void loop() {
    safeStopIfEStop();

    if (Serial.available()) {
        char c = Serial.read();
        switch (c) {
            case 'w': // forward
                drive.setSpeed(150, 150);
                break;
            case 's': // backward
                drive.setSpeed(-120, -120);
                break;
            case 'a': // left
                drive.setSpeed(-80, 80);
                break;
            case 'd': // right
                drive.setSpeed(80, -80);
                break;
            case 'x': // stop
                drive.stop();
                break;
            default:
                break;
        }
    }

    // periodic status print
    if (millis() - lastPeriodicMillis > statusInterval) {
        lastPeriodicMillis = millis();
        Serial.println("Status: running");
    }
}
