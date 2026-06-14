#include <Arduino.h>
#include "mq2.h"
#include "config.h"
#include "gps.h"

MQ2Sensor mq2(MQ2_PIN);
GPSModule gps(Serial2);

unsigned long lastCheck = 0;

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Phase 8: MQ-2 Smoke Sensor Integration");
    mq2.begin();
    gps.begin(9600);
}

void loop() {
    if (millis() - lastCheck > 1000) {
        lastCheck = millis();
        gps.feed();
        GPSData d = gps.getData();
        float val = mq2.readPpm();
        Serial.print("MQ2 raw fraction: "); Serial.println(val);
        if (mq2.isSmokeDetected(0.5f)) {
            double lat = d.valid ? d.latitude : 0.0;
            double lon = d.valid ? d.longitude : 0.0;
            Serial.print("SMOKE, "); Serial.print(lat,6); Serial.print(","); Serial.println(lon,6);
        }
    }
}
