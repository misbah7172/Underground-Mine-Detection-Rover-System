#include "power.h"
#include "config.h"

PowerManager::PowerManager(int adcPin, float dividerRatio, float fullV, float lowV)
    : pin(adcPin), ratio(dividerRatio), fullVoltage(fullV), lowVoltage(lowV) {}

void PowerManager::begin() {
    // ADC attenuation for full-scale reading above 1V
    analogReadResolution(12);
    analogSetPinAttenuation(pin, ADC_11db);
}

float PowerManager::readVoltage() {
    uint16_t raw = analogRead(pin);
    float adcMax = 4095.0;
    float vref = 3.3; // approximate; recommend calibration
    float vin = (raw / adcMax) * vref * ratio;
    return vin;
}

float PowerManager::readPercentage() {
    float v = readVoltage();
    if (v >= fullVoltage) return 100.0;
    if (v <= lowVoltage) return 0.0;
    return ((v - lowVoltage) / (fullVoltage - lowVoltage)) * 100.0;
}

bool PowerManager::isBatteryLow() {
    return readVoltage() <= lowVoltage;
}
