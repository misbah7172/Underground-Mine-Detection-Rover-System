#include <Arduino.h>
#include "power.h"
#include "config.h"

PowerManager power(BATTERY_ADC_PIN, BATTERY_VOLT_DIVIDER_RATIO, BATTERY_FULL_VOLTAGE, BATTERY_LOW_VOLTAGE);

unsigned long lastMillis = 0;
const unsigned long interval = 2000;

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Phase 2: Power Management and Stability");
    power.begin();
}

void loop() {
    if (millis() - lastMillis > interval) {
        lastMillis = millis();
        float v = power.readVoltage();
        float pct = power.readPercentage();
        Serial.print("Battery Voltage: "); Serial.print(v); Serial.print(" V  ");
        Serial.print("Charge: "); Serial.print(pct); Serial.println(" %");

        if (power.isBatteryLow()) {
            Serial.println("Battery LOW: initiating safe shutdown behaviors");
            // user can implement motor disable, return-to-base, or alert logic here
        }
    }
}
