# Mine Detection Rover - Underground Detection System

## Overview
This project is an ESP32-based rover platform for underground mine detection, obstacle handling, telemetry, and future laptop control.

## Components / Bill of Materials
Below is a comprehensive list of components used by this project (suggested quantities shown):

- **Microcontroller:** ESP32 DOIT DevKit V1 (ESP32-WROOM-32) — 1
- **LoRa module:** SX1278 (433 MHz) with antenna and header — 1
- **GPS module:** NEO-6M UART GPS module — 1
- **IMU:** MPU9250 (Accel + Gyro + AK8963 magnetometer) or MPU6050 + HMC5883L combo — 1
- **Motor drivers:** BTS7960 H-bridge driver modules — 2
- **DC motors:** 12V DC geared motors (matched pair) — 2
- **Chassis & drivetrain:** 2WD rover chassis, wheels, mounting hardware — 1 kit
- **Ultrasonic sensors:** HC-SR04 distance sensors — 2
- **Metal detector module:** NE555 astable oscillator module + search coil (4-pin: GND, VCC, Coil1, Coil2) — 1 module + coil
	- 100 Ω series resistor for output to ESP32 input recommended
	- Shielding / ferrite beads recommended to reduce motor noise coupling
- **Gas/smoke sensor:** MQ-2 sensor module (with amplifier) — 1
- **Environmental sensor (optional):** DHT11 or DHT22 — 1
- **MicroSD logging:** microSD card module (SPI) + microSD card (Class 10, 8–32GB) — 1
- **Power supply:** 12V battery pack (Li-ion pack or SLA) with appropriate capacity — 1
- **Power protection:** inline fuse, emergency stop switch — 1 each
- **Power regulation:** Buck converters (12V → 5V and 12V → 3.3V) — as required (2)
- **Battery & current sensing (optional but recommended):** voltage divider or INA219 / ACS712 current sensor — 1
- **Connectors & wiring:** screw terminals, bullet connectors, AWG 14–20 power wires, Dupont jumper wires — assorted
- **Level shifting / logic interfaces:** bidirectional level shifter or proper wiring for 3.3V/5V modules — as needed
- **Passive components:** pull-up/pull-down resistors, decoupling capacitors, 100nF and electrolytic caps — assorted
- **Indicators & controls:** LEDs, momentary buttons, emergency-stop latching switch — assorted
- **Mechanical hardware:** screws, standoffs, spacers, zip ties, mounting brackets — assorted
- **Tools & accessories:** USB A→micro/Type-C cable, soldering iron + solder, multimeter, wire stripper, heat shrink tubing — assorted
- **Optional modules / improvements:** antenna tuning components, metal detector coil preamp, RF shielding, custom PCB, enclosure

Note: many hobby modules (NEO-6M, SX1278, MQ-2, HC-SR04) are 5V-tolerant or require 5V power; ensure signal levels are safe for the ESP32 (3.3V) or use level shifters. See `docs/METAL_DETECTOR_NE555.md` for specific wiring of the NE555 metal detector and coil guidance.

## Wiring Snapshot
- Ultrasonic Trigger: GPIO 5
- Ultrasonic Echo: GPIO 18
- DHT11 Data: GPIO 4
- E-STOP: configured in `config.h`
- Motor PWM and direction pins: configured in `config.h`
- LoRa, GPS, MQ-2, metal detector, and battery monitor pins: configured in `config.h`

## Build And Upload
1. Install the PlatformIO extension in VS Code.
2. Connect the ESP32 through USB.
3. Run `pio run` to build.
4. Run `pio run -t upload` to flash the board.
5. Open Serial Monitor at 115200 baud for telemetry.

## Laptop Dashboard
The control station lives in `dashboard/app.py` and uses Streamlit.

Run it with:

```bash
f:/Electronics_Lab/.venv/Scripts/python.exe -m streamlit run dashboard/app.py
```

The dashboard supports live telemetry, manual drive control, map-based target selection, mission upload, and emergency stop commands.

The selected mission is sent as a JSON packet like:

```json
{"mission":"start","lat":23.8103,"lon":90.4125,"radius":20}
```

The rover firmware still needs the mission receiver / navigation loop to act on that packet if you want fully autonomous travel to the selected location.

## Process
1. Power the rover through the fused 12V line and buck converters.
2. Verify sensor readings on Serial before enabling motor motion.
3. Test manual control and emergency stop.
4. Validate obstacle avoidance and heading correction.
5. Move to waypoint, spiral search, and event logging phases.

## Data Collected
- Distance measurements
- Temperature and humidity
- Battery status
- GPS position and heading
- LoRa telemetry packets
- Metal-detect and smoke events
