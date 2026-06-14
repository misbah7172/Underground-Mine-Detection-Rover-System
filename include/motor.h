#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
public:
    // pins: two PWM inputs for BTS7960 (A and B). channelA/channelB are LEDC channels.
    Motor(int pinA, int pinB, uint8_t channelA, uint8_t channelB);
    // L298N constructor: IN1, IN2, EN(pin), EN channel
    Motor(int in1Pin, int in2Pin, int enPin, uint8_t enChannel, bool useL298);
    void begin(uint32_t freq, uint8_t resolution);
    // speed -255..255 where sign is direction
    void setSpeed(int16_t speed);
    void stop();

private:
    // BTS7960 style
    int pinA;
    int pinB;
    uint8_t chanA;
    uint8_t chanB;
    // L298N style
    bool isL298 = false;
    int in1Pin;
    int in2Pin;
    int enPin;
    uint8_t enChannel;
    uint8_t maxDuty();
};

#endif
