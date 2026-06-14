#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd, float outMin, float outMax)
    : Kp(kp), Ki(ki), Kd(kd), integrator(0.0f), prevError(0.0f), outMin(outMin), outMax(outMax) {}

void PIDController::setTunings(float kp, float ki, float kd) {
    Kp = kp; Ki = ki; Kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    outMin = min; outMax = max;
}

float PIDController::update(float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    integrator += error * dt;
    float derivative = 0.0f;
    if (dt > 0.0f) derivative = (error - prevError) / dt;
    float out = Kp * error + Ki * integrator + Kd * derivative;
    if (out > outMax) out = outMax;
    if (out < outMin) out = outMin;
    prevError = error;
    return out;
}
