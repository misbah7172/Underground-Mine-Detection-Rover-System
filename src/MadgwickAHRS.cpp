#include "MadgwickAHRS.h"
#include <math.h>

Madgwick::Madgwick(float beta_) : beta(beta_), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}

void Madgwick::begin(float sampleFreq) {
    (void)sampleFreq;
}

void Madgwick::getQuaternion(float& _q0, float& _q1, float& _q2, float& _q3) {
    _q0 = q0; _q1 = q1; _q2 = q2; _q3 = q3;
}

float Madgwick::getYaw() {
    return atan2(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f / M_PI;
}

float Madgwick::getPitch() {
    return asin(2.0f*(q0*q2 - q3*q1)) * 180.0f / M_PI;
}

float Madgwick::getRoll() {
    return atan2(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f / M_PI;
}

// Full Madgwick AHRS implementation (gradient descent)
void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    // Short name local variables
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;

    // Convert gyroscope degrees/sec to radians/sec
    float gxRad = gx * M_PI / 180.0f;
    float gyRad = gy * M_PI / 180.0f;
    float gzRad = gz * M_PI / 180.0f;

    // Normalise accelerometer measurement
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    ax /= norm; ay /= norm; az /= norm;

    // Normalise magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return;
    mx /= norm; my /= norm; mz /= norm;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    float hx = mx * (q0q0 + q1q1 - q2q2 - q3q3) + my * (2.0f * (q1*q2 - q0*q3)) + mz * (2.0f * (q1*q3 + q0*q2));
    float hy = mx * (2.0f * (q1*q2 + q0*q3)) + my * (q0q0 - q1q1 + q2q2 - q3q3) + mz * (2.0f * (q2*q3 - q0*q1));
    float bx = sqrtf(hx * hx + hy * hy);
    float bz = mx * (2.0f * (q1*q3 - q0*q2)) + my * (2.0f * (q2*q3 + q0*q1)) + mz * (q0q0 - q1q1 - q2q2 + q3q3);

    // Gradient descent algorithm corrective step
    float f1 = 2.0f * (q1*q3 - q0*q2) - ax;
    float f2 = 2.0f * (q0*q1 + q2*q3) - ay;
    float f3 = 2.0f * (0.5f - q1q1 - q2q2) - az;
    float f4 = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1*q3 - q0*q2) - mx;
    float f5 = 2.0f * bx * (q1*q2 - q0*q3) + 2.0f * bz * (q0*q1 + q2*q3) - my;
    float f6 = 2.0f * bx * (q0*q2 + q1*q3) + 2.0f * bz * (0.5f - q1q1 - q2q2) - mz;

    s0 = -f2 * q2 + f1 * q3 - f4 * q2 + f5 * q3 - f6 * q1;
    s1 = f2 * q1 + f3 * q2 - f1 * q0 + f4 * q3 - f5 * q0 + f6 * q2;
    s2 = -f3 * q1 + f2 * q0 + f1 * q3 - f4 * q0 + f5 * q3 - f6 * q3;
    s3 = f1 * q1 + f2 * q0 - f3 * q3 + f4 * q1 - f5 * q2 + f6 * q0;

    // Normalise step magnitude
    recipNorm = 1.0f / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    // Compute rate of change of quaternion
    qDot0 = 0.5f * (-q1 * gxRad - q2 * gyRad - q3 * gzRad) - beta * s0;
    qDot1 = 0.5f * ( q0 * gxRad + q2 * gzRad - q3 * gyRad) - beta * s1;
    qDot2 = 0.5f * ( q0 * gyRad - q1 * gzRad + q3 * gxRad) - beta * s2;
    qDot3 = 0.5f * ( q0 * gzRad + q1 * gyRad - q2 * gxRad) - beta * s3;

    // Integrate to yield quaternion
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normalise quaternion
    recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}
