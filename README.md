# Mine Detection Rover

ESP32-based mine-detection rover controlled from a Streamlit laptop dashboard over USB serial. This scaffold follows `BUILD_GUIDE.md` and defaults to ESP32 port **COM8** at **115200 baud**.

## Project layout

```text
.
├── platformio.ini              # ESP32 PlatformIO build/upload config, COM8 default
├── include/config.h            # Pin map, tuning constants, WiFi UDP settings
├── src/main.cpp                # Active rover firmware
├── src/gps.cpp                 # Historical placeholder, excluded from build
├── src/loramodule.cpp          # Historical placeholder, excluded from build
├── src/navigator.cpp           # Historical placeholder, excluded from build
├── dashboard/app.py            # Streamlit serial/mission dashboard
├── dashboard/requirements.txt  # Python dashboard dependencies
├── scripts/run_dashboard.ps1   # Installs dashboard deps and runs Streamlit
└── scripts/upload_firmware_COM8.ps1
```

## Prerequisites

1. Install VS Code extensions recommended by this workspace:
   - PlatformIO IDE
   - Python
2. Connect the ESP32 by USB and confirm it appears as `COM8`.
3. Close the dashboard or serial monitor before uploading firmware; only one program can own `COM8` at a time.

## Build and upload firmware

From a terminal in this folder:

```powershell
.\.venv\Scripts\python.exe -m pip install --upgrade platformio
.\.venv\Scripts\python.exe -m platformio run
.\.venv\Scripts\python.exe -m platformio run -t upload --upload-port COM8
```

Or run:

```powershell
.\scripts\upload_firmware_COM8.ps1
```

The PlatformIO config already contains:

```ini
upload_port = COM8
monitor_port = COM8
monitor_speed = 115200
```

## Run dashboard

```powershell
python -m pip install -r dashboard/requirements.txt
python -m streamlit run dashboard/app.py
```

If you are using the workspace virtual environment directly:

```powershell
.\.venv\Scripts\python.exe -m pip install -r dashboard/requirements.txt
.\.venv\Scripts\python.exe -m streamlit run dashboard/app.py
```

Or run:

```powershell
.\scripts\run_dashboard.ps1
```

Then open the dashboard, keep the default serial port `COM8`, baud `115200`, and click **Connect**.

## First hardware test

1. Wire ESP32, L298N, motors, motor battery, and common ground.
2. Upload firmware.
3. Start the dashboard and connect to `COM8`.
4. Open **Control**.
5. Set base speed to `140` or higher.
6. Click **Forward**.
7. Use **Stop Motors** or **Emergency Stop** if needed.

## Tuning

Edit `include/config.h` after real floor tests:

- `LOCAL_PATH_MS_PER_CM`: increase if the rover drives too short; decrease if it drives too far.
- `LOCAL_PATH_MS_PER_DEG`: increase if turns undershoot; decrease if turns overshoot.
- `METAL_ADC_DETECT_MIN` / `METAL_ADC_DETECT_MAX`: GPIO35 active-low metal detection range. Current default is `0`-`100` ADC; no metal is usually near `4095`.
- `OBSTACLE_DISTANCE_CM`: ultrasonic avoidance trigger distance.

## Sensor telemetry

Fast telemetry is split into two packet types:

- Compact telemetry every ~150 ms for fast values like `metal_adc`, `metal_detected`, rover pose, and heading.
- Detailed telemetry every ~1 second for slower fields like DHT11, WiFi status, distance, and mission metadata.

The dashboard merges compact and detailed packets, so fast metal ADC changes update quickly without losing slower fields.

The rover now publishes these additional telemetry fields:

- `metal_adc`, `metal_adc_min`, `metal_adc_max`, `metal_adc_detect_min`, `metal_adc_detect_max` — current GPIO35 active-low metal detector values. `metal_adc_min` captures short ADC drops between dashboard refreshes.
- `dht11_ok`, `dht11_temperature_c`, `dht11_humidity_percent` — DHT11 readings from GPIO15.
- `mission_target_index`, `remembered_target_valid`, `obstacle_count`, `last_obstacle_cm`, `avoiding_obstacle` — path memory and obstacle avoidance state.

During a mission, the ultrasonic obstacle routine remembers the current path target, performs a backup/right-sidestep detour, then recalculates toward the remembered target and continues the original mission path.

The dashboard has **Auto update every 1s** available, but it is off by default to keep control and mission buttons stable. Enable it only while monitoring telemetry, then turn it off before sending path missions.

## WiFi UDP alerts

Serial alerts work by default. For WiFi UDP metal alerts, use the dashboard runtime configuration:

1. Start the dashboard and connect to `COM8`.
2. In the sidebar, click **Start UDP** on port `4210`.
3. Expand **Configure rover WiFi UDP alerts**.
4. Enter the WiFi SSID/password and the dashboard computer IP.
5. Click **Send WiFi Config**.
6. Click **Test Metal Alert** to confirm the dashboard receives an alert and adds a red dot.

The dashboard sends this command to the rover:

```json
{"cmd":"wifi_config","ssid":"your_wifi_name","password":"your_wifi_password","host":"192.168.0.199","port":4210}
```

You can also compile WiFi credentials into firmware if desired:

1. Set `WIFI_ALERT_ENABLED` to `1` in `include/config.h`.
2. Set `WIFI_ALERT_SSID`, `WIFI_ALERT_PASSWORD`, and `WIFI_ALERT_HOST`.
3. Upload firmware again.
4. In the dashboard sidebar, click **Start UDP** on port `4210`.

## Notes

- GPS and LoRa are intentionally excluded from the current build.
- L298N ENA/ENB are expected to be jumpered HIGH, so firmware treats speed values as direction/on thresholds instead of PWM speed control.
- The rover starts local path missions at `(0, 0)` with initial heading toward positive Y.
