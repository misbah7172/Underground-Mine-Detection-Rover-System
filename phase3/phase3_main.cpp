#include <Arduino.h>
#include "motor.h"
#include "drive.h"
#include "ultrasonic.h"
#include "obstacle_avoid.h"
#include "config.h"

// motor channels
#define CHAN_ML_A 0
#define CHAN_ML_B 1
#define CHAN_MR_A 2
#define CHAN_MR_B 3

Motor motorL(MOTOR_L_PWM_A, MOTOR_L_PWM_B, CHAN_ML_A, CHAN_ML_B);
Motor motorR(MOTOR_R_PWM_A, MOTOR_R_PWM_B, CHAN_MR_A, CHAN_MR_B);
Drive drive(motorL, motorR);

UltrasonicSensor usLeft(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
UltrasonicSensor usRight(ULTRASONIC_TRIG_PIN+2, ULTRASONIC_ECHO_PIN+2); // ensure separate pins in real wiring
ObstacleAvoider avoider(usLeft, usRight, 40.0);

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Phase 3: Obstacle Avoidance (sim)");
    drive.begin();
    usLeft.init();
    usRight.init();
    avoider.begin();
}

unsigned long lastLoop = 0;

void loop() {
    int16_t lcmd=0, rcmd=0;
    if (avoider.checkAndGetAvoidance(lcmd, rcmd)) {
        drive.setSpeed(lcmd, rcmd);
    } else {
        // default patrol forward
        drive.setSpeed(100, 100);
    }

    if (millis() - lastLoop > 1000) {
        lastLoop = millis();
        Serial.println("Driving..." );
    }
}
