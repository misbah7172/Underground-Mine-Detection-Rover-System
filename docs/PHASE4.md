# PHASE 4 — LoRa Communication System

1. Goal of the phase
- Provide robust, CRC-validated LoRa packet transfer between rover and base station using SX1278 module.

2. Wiring diagram explanation
- Use VSPI pins on ESP32: SCK=18, MISO=19, MOSI=23. Connect `LORA_SS_PIN`, `LORA_RST_PIN`, `LORA_DIO0_PIN` to the RA-02 module accordingly.

3. Pin configuration table
| Signal | Purpose | ESP32 Pin |
|---|---:|---:|
| LORA_SS | LoRa SPI CS | 18 |
| LORA_RST | LoRa Reset | 22 |
| LORA_DIO0 | LoRa DIO0 | 21 |

4. Circuit protection advice
- Level-shift MOSI/MISO/SS if module is 5V; RA-02 modules typically 3.3V. Ensure stable 3.3V for module; add 100nF decoupling and ferrite bead on supply.

5. Folder structure
- `include/loramodule.h`, `src/loramodule.cpp`, `phase4/phase4_main.cpp`, `docs/PHASE4.md`

6. Required libraries
- `sandeepmistry/LoRa` (added to `platformio.ini`)

7. Complete ESP32 code
- See `include/loramodule.h` and `src/loramodule.cpp` and `phase4/phase4_main.cpp`.

8. Explanation of code
- `LoRaModule` wraps the LoRa library and adds CRC16 to each packet for integrity. `sendPacket` appends CRC, `receivePacket` validates CRC and returns payload size.

9. Testing procedure
- Pair rover with a laptop/base LoRa transceiver running same frequency and CRC handling. Observe successful send/receive and CRC acceptance. Use different payloads and check error handling.

10. Common failure points
- Incorrect SPI pin wiring, incorrect module voltage, mismatched frequency or bandwidth settings, and insufficient antenna grounding.

11. Debugging guide
- Check serial for init logs. Verify SPI signals with logic analyzer. Use simple hello packets and verify reception on both ends. Validate CRC mismatches.

12. Suggested improvements
- Add ACKs and retransmissions, sequence numbers, and message types. Implement packet throttling and priority for telemetry.
