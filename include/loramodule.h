#ifndef LORAMODULE_H
#define LORAMODULE_H

#include <Arduino.h>
#include <SPI.h>

class LoRaModule {
public:
    LoRaModule(int ssPin, int rstPin, int dio0Pin);
    bool begin(long frequency);
    bool sendPacket(const uint8_t* data, size_t len);
    // send and wait for an ACK reply; returns true if ACK received
    bool sendPacketWithAck(const uint8_t* data, size_t len, int retries = 3, unsigned long timeoutMs = 1500);
    // enqueue a reliable message (non-blocking). Returns true if queued.
    bool sendReliable(const char* payload, int retries = 3, unsigned long timeoutMs = 1500);
    // poll the LoRa module to process incoming packets and retry logic. Call regularly from loop().
    void poll();
    // non-blocking receive; returns length or 0
    int receivePacket(uint8_t* buf, size_t bufsize);
    // fetch next received message captured by poll(); returns true if one is available
    bool readMessage(String& out);

private:
    int ss;
    int rst;
    int dio0;
    uint16_t crc16(const uint8_t* data, size_t len);
};

#endif
