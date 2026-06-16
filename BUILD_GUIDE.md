# Mine Detection Rover Build Guide

## 1. Project Summary

This project is an ESP32-based mine-detection rover controlled from a Streamlit laptop dashboard. The current build does not use GPS or LoRa. The rover receives commands over USB serial from the dashboard, follows local paths drawn on the dashboard whiteboard, detects metal with an NE555 ADC-based sensor, and can optionally send metal alerts over WiFi UDP.

## 2. Current Feature Set

- Manual motor control from the dashboard.
- Local whiteboard path planning using circle, square, triangle, lawnmower, or freehand paths.
- Autonomously cover the whole local path following using timed turns and timed forward movement.
- Return-to-center behavior at the end of a board mission.
- Front ultrasonic obstacle check with backup/turn avoidance.
- NE555 ADC metal detector calibration and detection telemetry on GPIO 35. This detector is active-low: metal reads near `0`, and no metal reads near `4095`.
- DHT11 temperature/humidity telemetry on GPIO 15.
- Red dot marking on the dashboard board when metal alerts include local `x_cm` / `y_cm` coordinates.
- Optional WiFi UDP alert transmission from rover to dashboard.

## 3. Removed Modules

GPS and LoRa are removed from the active firmware build.

- Do not wire the NEO-6M GPS module for the current build.
- Do not wire the SX1278 LoRa module for the current build.
- `platformio.ini` excludes `gps.cpp`, `loramodule.cpp`, and `navigator.cpp` from the active firmware build.
- The `LoRa` and `TinyGPSPlus` PlatformIO dependencies were removed from `platformio.ini`.

The old source files are still present in the repository for historical phase tests, but they are not part of the main firmware build.

## 4. Hardware Required

- ESP32 DOIT DevKit V1.
- L298N motor driver.
- Two DC geared motors.
- Rover chassis and wheels.
- 12V motor battery or suitable motor supply.
- Common ground between ESP32 and L298N.
- HC-SR04 front ultrasonic sensor.
- NE555 metal detector circuit/module with search coil.
- Optional MQ-2 gas sensor.
- Optional DHT11 sensor.
- Optional WiFi router/network for UDP metal alerts.

## 5. Motor Wiring

The active L298N input pin mapping is defined in `include/config.h`:

| L298N Pin | ESP32 GPIO |
| --- | --- |
| IN1 | 26 |
| IN2 | 27 |
| IN3 | 14 |
| IN4 | 12 |

The firmware assumes ENA and ENB are jumpered HIGH on the L298N module.

## 6. Sensor Wiring

Current important pins in `include/config.h`:

| Function | ESP32 GPIO |
| --- | --- |
| Front ultrasonic TRIG | 33 |
| Front ultrasonic ECHO | 32 |
| NE555 metal ADC output | 35 |
| MQ-2 analog input | 34 |
| DHT11 data | 15 |

GPIO 35 and GPIO 36 are input-only pins on ESP32. That is fine for ADC/input use.

## 7. Power Wiring

- Power motors from the L298N motor supply input, not from ESP32 USB.
- Use a fused 12V motor supply where possible.
- Use a buck converter for stable logic power if powering modules from the rover battery.
- Always connect ESP32 GND and L298N GND together.
- Keep motor power wires away from the metal detector coil/signal wire to reduce noise.

In the dashboard sidebar:

- Select `COM7` or the ESP32 serial port shown on your PC.
- Select baud rate `115200`.
- Click `Connect`.

## 10. Manual Motor Test

After connecting the dashboard:

1. Open the `Control` tab.
2. Set base speed to a visible value, for example `140` or higher.
3. Click `Forward`.
4. The rover should move briefly.
5. Use `Stop Motors` or `Emergency Stop` if needed.

The dashboard sends manual drive commands like:

```json
{"cmd":"drive","left":255,"right":255,"duration_ms":900}
```

## 11. Whiteboard Path Mission

The `Mission` tab contains the local path whiteboard.

1. Choose a path source: Circle, Square, Triangle, Lawnmower, or Freehand.
2. Adjust `Board scale (cm / pixel)`.
3. Keep `Return to center` enabled if you want the rover to finish at the blue start dot.
4. Click `Send Path Mission`.

The dashboard sends local path coordinates in centimeters, not GPS coordinates:

```json
{"cmd":"path","mission_id":"board-path","points":[{"x_cm":0,"y_cm":50},{"x_cm":50,"y_cm":50},{"x_cm":0,"y_cm":0}]}
```

The rover starts at the blue center point, follows the board path using timed motor movement, returns to center if enabled, and stops.

## 12. Local Path Tuning

The no-GPS path follower is time-based. Tune these values in `include/config.h`:


If the rover drives too far, reduce `LOCAL_PATH_MS_PER_CM`.
If it drives too short, increase `LOCAL_PATH_MS_PER_CM`.
If turns overshoot, reduce `LOCAL_PATH_MS_PER_DEG`.
If turns undershoot, increase `LOCAL_PATH_MS_PER_DEG`.

## 13. Metal Detection Flow

At boot, the rover calibrates the NE555 ADC baseline for about two seconds. Keep metal away from the coil during boot.

During movement:

- The rover samples the metal detector ADC about every `10 ms`.
- The rover sends compact telemetry about every `150 ms` and detailed telemetry about every `1 s`.
- The dashboard telemetry panel refreshes without full-page mission/control reloads.
- GPIO 35 ADC values between `METAL_ADC_DETECT_MIN` and `METAL_ADC_DETECT_MAX` are treated as metal detected.
- Current default range is `0`-`100` ADC because your sensor reads `0` when metal is detected and about `4095` when no metal is present.
- When metal is detected, it sends a serial event to the dashboard.
- If WiFi alerts are configured, it also sends a UDP alert.
- The dashboard marks the board with a red dot using local `x_cm` / `y_cm` coordinates.

By default, metal detection does not stop the mission:

```cpp
#define METAL_DETECTION_STOPS_MISSION 0
```

Set it to `1` if you want the rover to stop when metal is detected.

## 13.1 DHT11 Telemetry

The DHT11 data pin is GPIO 15. The firmware publishes:

```json
{"dht11_ok":true,"dht11_temperature_c":31.0,"dht11_humidity_percent":86.1}
```

If the DHT11 is not connected or cannot be read, `dht11_ok` is `false` and the temperature/humidity values are `null`.

## 13.2 Ultrasonic Obstacle Avoidance and Remembered Path

The HC-SR04 front ultrasonic sensor uses GPIO 33 for TRIG and GPIO 32 for ECHO.

During a board path mission, the rover stores the current mission target point. If an obstacle is detected closer than `OBSTACLE_DISTANCE_CM`, it:

1. Stops.
2. Backs up.
3. Turns right.
4. Drives a short sidestep.
5. Turns back toward the original heading.
6. Recalculates the route to the remembered mission target.
7. Continues the original path.

Telemetry includes `remembered_target_valid`, `mission_target_index`, `obstacle_count`, `last_obstacle_cm`, and `avoiding_obstacle`.

## 14. WiFi UDP Alerts

The dashboard UDP listener defaults to port `4210`.

In `include/config.h`, configure WiFi if needed:

```cpp
#define WIFI_ALERT_SSID "your_wifi_name"
#define WIFI_ALERT_PASSWORD "your_wifi_password"
#define WIFI_ALERT_HOST "your_dashboard_computer_ip"
#define WIFI_ALERT_PORT 4210
```

In the dashboard sidebar:

1. Set UDP port to `4210`.
2. Click `Start UDP`.
3. Expand `Configure rover WiFi UDP alerts`.
4. Enter the WiFi SSID/password and the dashboard computer IP.
5. Click `Send WiFi Config`.
6. Use `Test Metal Alert` to verify the red dot appears.

The runtime WiFi command is:

```json
{"cmd":"wifi_config","ssid":"your_wifi_name","password":"your_wifi_password","host":"your_dashboard_computer_ip","port":4210}
```

The rover sends alerts like:

```json
{"type":"alert","alert_type":"metal_detected","x_cm":12.0,"y_cm":45.0,"adc_drop":520,"confidence":80}
```

The dashboard can auto-update every 1 second for telemetry monitoring. It is off by default to keep control and mission button clicks stable. Enable it only while monitoring, then turn it off before sending path missions.

## 15. Serial Protocol Summary

Dashboard to rover:

```json
{"cmd":"drive","left":255,"right":255,"duration_ms":900}
{"cmd":"stop"}
{"cmd":"estop"}
{"cmd":"status"}
{"cmd":"ping"}
{"cmd":"path","mission_id":"board-path","points":[{"x_cm":0,"y_cm":50}]}
```

Rover to dashboard:

```json
{"type":"telemetry","mode":"IDLE","mission_active":false,"distance_cm":120.0,"metal_detected":false}
{"type":"event","level":"INFO","category":"MOTOR","message":"L298N pins IN1=26 IN2=27 IN3=14 IN4=12"}
{"type":"alert","alert_type":"metal_detected","x_cm":12.0,"y_cm":45.0}
```

## 16. Troubleshooting

### Motors do not move

- Confirm ENA/ENB jumpers are installed.
- Confirm motor battery is connected and charged.
- Confirm common ground between ESP32 and L298N.
- Confirm IN pins match GPIO 26, 27, 14, 12.
- Check dashboard raw log for the motor pin boot event.
- Test manual control before autonomous path mode.

### Upload fails

- Disconnect the dashboard from COM7.
- Close serial monitors.
- Unplug and reconnect the ESP32 USB cable.
- Hold BOOT during upload if the board does not enter flashing mode automatically.

### Board path is inaccurate

- Tune `LOCAL_PATH_MS_PER_CM` and `LOCAL_PATH_MS_PER_DEG`.
- Test on the same floor surface each time.
- Use a fully charged motor battery.
- Reduce speed if the rover slips.

### Red metal dots do not appear

- Confirm metal detection events appear in the dashboard log.
- Confirm alerts include `x_cm` and `y_cm`.
- If using WiFi UDP, confirm dashboard UDP listener is started on port `4210`.

## 17. Recommended Build Order

1. Wire ESP32, L298N, motors, common ground, and motor battery.
2. Upload firmware.
3. Connect dashboard over serial.
4. Test manual motor controls.
5. Wire ultrasonic sensor and verify distance telemetry.
6. Wire NE555 metal detector and verify calibration/ADC telemetry.
7. Test a short straight board path.
8. Tune timing constants.
9. Test square/circle paths.
10. Enable WiFi UDP alerts only after serial control works.
