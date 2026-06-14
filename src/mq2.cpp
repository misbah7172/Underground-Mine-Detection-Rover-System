#include "mq2.h"
#include "config.h"

MQ2Sensor::MQ2Sensor(int p): pin(p) {}

void MQ2Sensor::begin() {
    analogReadResolution(12);
    analogSetPinAttenuation(pin, ADC_11db);
}

int MQ2Sensor::readRaw() {
    return analogRead(pin);
}

float MQ2Sensor::readPpm() {
    int raw = readRaw();
    // normalize 0..4095 to 0..1
    return (float)raw / 4095.0f;
}

bool MQ2Sensor::isSmokeDetected(float threshold) {
    return readPpm() >= threshold;
}
