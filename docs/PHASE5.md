# PHASE 5 — GPS Parsing and Navigation

1. Goal of the phase
- Integrate NEO-6M GPS with TinyGPSPlus to obtain location, satellites, HDOP, and speed for use in navigation and telemetry.

2. Wiring diagram explanation
- Connect GPS TX -> ESP32 `GPS_RX_PIN` (GPIO16), GPS RX -> ESP32 `GPS_TX_PIN` (GPIO17). Provide GPS module 3.3V/5V as required and common ground.

3. Pin configuration table
| Signal | Purpose | ESP32 Pin |
|---|---:|---:|
| GPS RX | GPS TX -> ESP32 RX2 | 16 |
| GPS TX | GPS RX <- ESP32 TX2 | 17 |

4. Circuit protection advice
- Add a small ferrite bead and 100nF decoupling on GPS power. Use level shifting if module requires 5V signalling (rare).

5. Folder structure
- `include/gps.h`, `src/gps.cpp`, `phase5/phase5_main.cpp`, `docs/PHASE5.md`

6. Required libraries
- `mikalhart/TinyGPSPlus` (added to `platformio.ini`)

7. Complete ESP32 code
- See `include/gps.h` and `src/gps.cpp` and `phase5/phase5_main.cpp`.

8. Explanation of code
- `GPSModule` uses `Serial2` to read NMEA sentences, feeds bytes to `TinyGPSPlus`, and exposes a `GPSData` struct with valid flag.

9. Testing procedure
- Power GPS with clear sky view; run `phase5_main` and observe serial prints. Confirm coordinates, satellites, and HDOP reported.

10. Common failure points
- No fix due to poor antenna placement; wrong RX/TX wiring; insufficient power; wrong baud rate.

11. Debugging guide
- Use USB-Serial adapter to monitor raw NMEA, check GPS module baud (usually 9600). Move antenna to open sky.

12. Suggested improvements
- Add PPS timing if module supports it for accurate time; add RTK or external heading sensors for accurate heading.
