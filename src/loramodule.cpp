#include "loramodule.h"
#include "config.h"
#include <LoRa.h>

LoRaModule::LoRaModule(int ssPin, int rstPin, int dio0Pin)
    : ss(ssPin), rst(rstPin), dio0(dio0Pin) {}

bool LoRaModule::begin(long frequency) {
    SPI.begin();
    LoRa.setPins(ss, rst, dio0);
    if (!LoRa.begin(frequency)) {
        return false;
    }
    LoRa.enableCrc();
    return true;
}

uint16_t LoRaModule::crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

bool LoRaModule::sendPacket(const uint8_t* data, size_t len) {
    if (len == 0 || len > 240) return false;
    uint16_t crc = crc16(data, len);
    LoRa.beginPacket();
    LoRa.write(data, len);
    LoRa.write((uint8_t)(crc >> 8));
    LoRa.write((uint8_t)(crc & 0xFF));
    LoRa.endPacket();
    return true;
}

bool LoRaModule::sendPacketWithAck(const uint8_t* data, size_t len, int retries, unsigned long timeoutMs) {
    if (retries < 1) retries = 1;
    for (int attempt = 0; attempt < retries; ++attempt) {
        if (!sendPacket(data, len)) continue;
        unsigned long start = millis();
        while (millis() - start < timeoutMs) {
            int packetSize = LoRa.parsePacket();
            if (packetSize > 0) {
                // read payload
                String resp = "";
                for (int i = 0; i < packetSize; ++i) {
                    int ch = LoRa.read();
                    if (ch < 0) break;
                    resp += (char)ch;
                }
                resp.trim();
                if (resp.equalsIgnoreCase("ACK") || resp.indexOf("\"type\":\"ack\"") >= 0) {
                    return true;
                }
            }
            delay(50);
        }
    }
    return false;
}

// Simple non-blocking reliable send with small pending queue
struct PendingMsg {
    uint32_t msgId;
    String payload;
    int retriesLeft;
    unsigned long lastSentMs;
    unsigned long timeoutMs;
    bool active;
    int backoffExp;
};

static uint32_t nextMsgId = 1;
static const int MAX_PENDING = 4;
static PendingMsg pending[MAX_PENDING];

struct RxMsg {
    String payload;
};

static const int MAX_RX = 4;
static RxMsg rxQueue[MAX_RX];
static int rxHead = 0;
static int rxTail = 0;
static int rxCount = 0;

bool LoRaModule::sendReliable(const char* payload, int retries, unsigned long timeoutMs) {
    // find free slot
    int idx = -1;
    for (int i = 0; i < MAX_PENDING; ++i) if (!pending[i].active) { idx = i; break; }
    if (idx < 0) return false;
    uint32_t id = nextMsgId++;
    // build envelope JSON: {"msg_id":id,"type":"msg","body":<string>}
    String env = String("{\"msg_id\":") + String(id) + String(",\"type\":\"msg\",\"body\":\"") + String(payload) + String("\"}");
    pending[idx].msgId = id;
    pending[idx].payload = env;
    pending[idx].retriesLeft = max(1, retries);
    pending[idx].timeoutMs = timeoutMs;
    pending[idx].lastSentMs = 0;
    pending[idx].active = true;
    pending[idx].backoffExp = 0;
    return true;
}

static void queue_rx(const String& msg) {
    if (rxCount >= MAX_RX) {
        rxHead = (rxHead + 1) % MAX_RX;
        rxCount--;
    }
    rxQueue[rxTail].payload = msg;
    rxTail = (rxTail + 1) % MAX_RX;
    rxCount++;
}

bool LoRaModule::readMessage(String& out) {
    if (rxCount <= 0) return false;
    out = rxQueue[rxHead].payload;
    rxQueue[rxHead].payload = "";
    rxHead = (rxHead + 1) % MAX_RX;
    rxCount--;
    return true;
}

static void send_raw_string(const String &s) {
    LoRa.beginPacket();
    LoRa.print(s);
    LoRa.endPacket();
}

void LoRaModule::poll() {
    // handle incoming packets
    int packetSize = LoRa.parsePacket();
    if (packetSize > 0) {
        String resp = "";
        while (LoRa.available()) {
            int ch = LoRa.read();
            if (ch < 0) break;
            resp += (char)ch;
        }
        resp.trim();
        // if it's an ack JSON {"type":"ack","msg_id":123}
        if (resp.indexOf("\"type\":\"ack\"") >= 0) {
            // try to extract msg_id
            int idx = resp.indexOf("\"msg_id\":");
            if (idx >= 0) {
                int start = idx + 9;
                long id = resp.substring(start).toInt();
                for (int i = 0; i < MAX_PENDING; ++i) {
                    if (pending[i].active && pending[i].msgId == (uint32_t)id) {
                        pending[i].active = false;
                    }
                }
            }
        } else {
            // if it's a message envelope containing msg_id, respond with ACK
            int idx = resp.indexOf("\"msg_id\":");
            if (idx >= 0) {
                int start = idx + 9;
                long id = resp.substring(start).toInt();
                // Reply ACK
                String ack = String("{\"type\":\"ack\",\"msg_id\":") + String(id) + String("}");
                send_raw_string(ack);
            }
            // forward payload to Serial for main to parse
            queue_rx(resp);
            Serial.println(resp);
        }
    }

    // handle pending retransmits
    unsigned long now = millis();
    for (int i = 0; i < MAX_PENDING; ++i) {
        if (!pending[i].active) continue;
        if (pending[i].lastSentMs == 0 || (now - pending[i].lastSentMs) >= (pending[i].timeoutMs * (1UL << pending[i].backoffExp))) {
            if (pending[i].retriesLeft <= 0) {
                // give up
                pending[i].active = false;
                // emit a simple log to Serial so main can raise event if needed
                Serial.println(String("{\"type\":\"event\",\"level\":\"WARN\",\"category\":\"LORA\",\"message\":\"reliable send failed msg_id=") + String(pending[i].msgId) + String("\"}"));
                continue;
            }
            // send
            send_raw_string(pending[i].payload);
            pending[i].lastSentMs = now;
            pending[i].retriesLeft -= 1;
            pending[i].backoffExp = min(pending[i].backoffExp + 1, 6);
        }
    }
}

int LoRaModule::receivePacket(uint8_t* buf, size_t bufsize) {
    int packetSize = LoRa.parsePacket();
    if (packetSize <= 0) return 0;
    if (packetSize < 3) return 0; // at least 2 bytes CRC
    int payloadSize = packetSize - 2;
    if (payloadSize > (int)bufsize) return 0;
    for (int i = 0; i < payloadSize; ++i) {
        buf[i] = LoRa.read();
    }
    uint8_t crcHi = LoRa.read();
    uint8_t crcLo = LoRa.read();
    uint16_t recvCrc = (crcHi << 8) | crcLo;
    uint16_t calc = crc16(buf, payloadSize);
    if (calc != recvCrc) return 0;
    return payloadSize;
}
