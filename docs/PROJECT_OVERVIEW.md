**Project Overview**

This repository implements a modular autonomous 2-wheel rover built around an ESP32. The system is split into firmware running on the ESP32 and a laptop control station (Streamlit dashboard) used for mission planning, live telemetry, and manual override.

**Highlights:**
- **Hardware:** ESP32 (DOIT DEVKIT V1), SX1278 LoRa, NEO-6M GPS, MPU9250 (IMU), HMC5883L magnetometer support, 2×HC-SR04 ultrasonic sensors, MQ-2 smoke sensor, **NE555-based metal detector with search coil**, BTS7960 motor drivers, 12V battery + buck converters.
- **Firmware:** PlatformIO (Arduino framework). Modular drivers: motor, drive, GPS, IMU (MPU9250 + Madgwick AHRS), waypoint navigator, obstacle avoidance, LoRa comms, **frequency-based metal detection (interrupt counting + calibration)**, metal & gas sensors.
- **Control Station:** Streamlit dashboard with interactive Leaflet map (streamlit-folium) for mission selection, centimetre-radius target, serial/LoRa mission upload, live rover track, telemetry, **real-time metal detector frequency and confidence display**.

**Repository layout (important files):**
- **`platformio.ini`** — project configuration and lib dependencies.
- **`src/main.cpp`** — mission controller, telemetry emission, mission packet parsing.
- **`include/`**, **`src/`** — firmware modules (IMU, `MadgwickAHRS`, `imu_fusion`, `gps`, `loramodule`, `navigator`, `obstacle_avoid`, `motor`, `drive`, `ultrasonic`, `mq2`, `metal_detector`).
- **`dashboard/app.py`** — Streamlit UI and map-based mission builder.
- **`dashboard/protocol.py`**, **`dashboard/serial_client.py`** — mission packet format and serial client used by the dashboard.
- **`docs/`** — phase docs, wiring notes and this overview.

**How it works — high level workflow**
1. Operator opens the dashboard on a laptop and selects a geographic target by clicking the map. The operator chooses a mission radius (centimetres) and taps "Send Mission".
2. The dashboard builds a JSON mission packet and sends it to the rover via the serial-to-LoRa gateway or directly over USB serial. Example packet:

```json
{"mission":"start","lat":23.8103,"lon":90.4125,"radius":35.0,"radius_cm":35.0,"radius_m":0.35}
```

3. On the ESP32, `src/main.cpp` listens for incoming JSON mission packets. When a mission start packet is received the rover:
   - sets the target waypoint and mission radius (meters internally),
   - enables the Waypoint Navigator which uses GPS and the AHRS for heading control,
   - emits telemetry periodically (JSON or key=value lines) with position, heading, speed, and sensor alerts.

4. During navigation the ObstacleAvoider monitors the two HC-SR04 sensors. If an obstacle is detected the rover performs a non-blocking avoidance routine (back + turn) and then re-enters waypoint navigation.

5. Telemetry and event logs (metal/smoke detection, obstacle events) are sent back to the dashboard in real time for display on the map and alert panels.

**Firmware architecture & key behaviors**
- **IMU & AHRS:** The MPU9250 driver reads accel/gyro/mag; `MadgwickAHRS` performs sensor fusion. Magnetometer offsets are stored in Preferences via `imu_fusion` so calibrations persist across power cycles.
- **Navigation:** `navigator` computes bearing & distance-to-goal. A PID-style heading controller (inlined in the navigator) steers the rover using differential motor speeds. Distances are calculated in metres; the dashboard sends radius in centimetres but includes `radius_m` for firmware consumption.
- **LoRa & Serial Protocol:** `loramodule` wraps SX1278 with a CRC16 for packet integrity. The firmware accepts mission JSON packets and also forwards telemetry optionally over LoRa.
- **Non-blocking design:** All behaviors use millis()-based scheduling to avoid blocking delays and keep sensor reads, telemetry, and control loops responsive.

**Wiring & pins (summary)**
- Motor driver PWM and direction — defined in `include/config.h`.
- GPS — Serial2 RX/TX pins (see GPS module in `include/gps.h`).
- LoRa — SPI pins configured in `include/loramodule.h`.
- Ultrasonic sensors — trigger/echo pins configured in `include/ultrasonic.h`.
- IMU — I2C (SDA / SCL) to MPU9250. Magnetometer access handled via the MPU's AK8963.
- Sensors (MQ-2, metal detector) — analog/digital pins in `include/config.h`.

Refer to the phase docs for wiring diagrams and pin-by-pin notes: [docs/PHASE1_PARTS.md](docs/PHASE1_PARTS.md) and other PHASE files in `docs/`.

**Build & flash (quick commands)**
Run these from the repository root. Build and upload using PlatformIO:

```bash
# build
platformio run

# build + upload (set your port if needed)
platformio run -t upload --upload-port COM8
```

Dashboard setup and run (Python virtualenv recommended):

```bash
python -m venv .venv
.
.venv\Scripts\activate
pip install -r dashboard/requirements.txt
streamlit run dashboard/app.py
```

**Dashboard features & usage**
- Click the map to set the mission target and drag the radius slider (cm) for mission tolerance.
- `Send Mission` transmits the start packet to the rover via the configured serial port.
- Live telemetry shows rover marker, trail, heading, and sensor alerts. Manual controls allow direct drive commands for field testing.

**Telemetry format**
- The firmware emits JSON lines and key=value lines. The dashboard parses both. Telemetry typically contains `lat`, `lon`, `heading`, `speed`, and sensor flags like `metal=true` or `smoke=...` (see `dashboard/protocol.py`).

**Calibration & tuning checklist (recommended after first bench tests)**
- Calibrate magnetometer offsets (rough hard/soft iron) and save via Preferences — `include/imu_fusion.h` stores offsets.
- Tune the navigator's heading gains (P/I/D) and motor throttle scaling in `include/navigator.h` / `src/navigator.cpp`.
- Adjust ObstacleAvoider distances and backoff durations based on real-world tests.

**Next improvements & TODOs**
- Add SD-card event logging and periodic mission telemetry persistence.
- Implement LoRa ACK/retry for mission-critical telemetry and packet confirmation.
- Add a guided magnetometer calibration routine (soft-iron scaling) accessible from the dashboard.

**Where to look in the code**
- Mission controller and main behavior: [src/main.cpp](src/main.cpp)
- AHRS / sensor fusion: [src/MadgwickAHRS.cpp](src/MadgwickAHRS.cpp) and [include/imu_fusion.h](include/imu_fusion.h)
- Waypoint navigator: [src/navigator.cpp](src/navigator.cpp)
- Obstacle avoidance: [src/obstacle_avoid.cpp](src/obstacle_avoid.cpp)
- LoRa wrapper: [src/loramodule.cpp](src/loramodule.cpp)
- Dashboard UI: [dashboard/app.py](dashboard/app.py)
- Dashboard protocol: [dashboard/protocol.py](dashboard/protocol.py)
- **Metal detector (NE555):** [include/metal_detector_ne555.h](include/metal_detector_ne555.h) and [src/metal_detector_ne555.cpp](src/metal_detector_ne555.cpp) — frequency-based detection with interrupt counting, calibration, and noise filtering. See [docs/METAL_DETECTOR_NE555.md](docs/METAL_DETECTOR_NE555.md) for hardware wiring and tuning guide.

If you want, I can:
- add a one-page wiring diagram (SVG/PNG) to `docs/`;
- add a magnetometer calibration routine accessible from the dashboard;
- implement SD logging and LoRa ACK/retry logic next.

---
Generated: May 14, 2026
