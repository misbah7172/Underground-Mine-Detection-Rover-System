#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outMin=-255, float outMax=255);
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    float update(float setpoint, float measured, float dt);
private:
    float Kp, Ki, Kd;
    float integrator;
    float prevError;
    float outMin, outMax;
};

#endif
