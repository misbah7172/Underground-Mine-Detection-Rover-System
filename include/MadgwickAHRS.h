#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

#include <Arduino.h>

class Madgwick {
public:
    Madgwick(float beta = 0.1f);
    void begin(float sampleFreq);
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
    void getQuaternion(float& q0, float& q1, float& q2, float& q3);
    float getYaw();
    float getPitch();
    float getRoll();
private:
    float beta; // algorithm gain
    float q0, q1, q2, q3;
};

#endif
