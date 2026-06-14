#include <Arduino.h>
#include "loramodule.h"
#include "config.h"

LoRaModule lora(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Phase 4: LoRa Communication Test");
    if (!lora.begin(433E6)) {
        Serial.println("LoRa init failed");
    } else {
        Serial.println("LoRa init OK");
    }
}

unsigned long lastTx = 0;

void loop() {
    if (millis() - lastTx > 3000) {
        lastTx = millis();
        const char* msg = "Hello from rover";
        lora.sendPacket((const uint8_t*)msg, strlen(msg));
        Serial.println("Sent LoRa packet");
    }

    uint8_t buf[256];
    int len = lora.receivePacket(buf, sizeof(buf));
    if (len > 0) {
        Serial.print("Received LoRa: ");
        Serial.write(buf, len);
        Serial.println();
    }
}
