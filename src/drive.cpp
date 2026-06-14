#include "drive.h"
#include "config.h"

Drive::Drive(Motor& leftMotor, Motor& rightMotor)
    : left(leftMotor), right(rightMotor), lastCommandMillis(0) {}

void Drive::begin() {
    left.begin(MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    right.begin(MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    pinMode(E_STOP_PIN, INPUT_PULLUP); // active LOW
}

void Drive::setSpeed(int16_t leftSpd, int16_t rightSpd) {
    lastCommandMillis = millis();
    left.setSpeed(leftSpd);
    right.setSpeed(rightSpd);
}

void Drive::stop() {
    left.stop();
    right.stop();
}

void Drive::emergencyStop() {
    stop();
    // additional hooks can be added here
}
