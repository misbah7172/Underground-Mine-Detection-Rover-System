# Underground Mine Detection Rover Project

## Build Process
1. Install PlatformIO CLI or the VS Code PlatformIO extension.
2. Open the `Electronics_Lab` workspace.
3. Connect the ESP32 through USB.
4. Run `pio run` to compile.
5. Run `pio run -t upload` to flash the board.
6. Open the serial monitor at 115200 baud.

## Wiring

### Ultrasonic Sensor
- ESP32 GPIO 5 -> HC-SR04 Trigger
- ESP32 GPIO 18 -> HC-SR04 Echo
- ESP32 GND -> HC-SR04 GND
- ESP32 5V -> HC-SR04 VCC

### DHT11 Sensor
- ESP32 GPIO 4 -> DHT11 Data
- ESP32 3.3V -> DHT11 VCC
- ESP32 GND -> DHT11 GND
- Optional: 10k pull-up between VCC and Data

### Rover Control System
- Motor driver power -> fused 12V battery rail
- Motor driver GND -> common ground with ESP32
- Motor PWM/direction -> pins defined in `include/config.h`
- E-STOP -> emergency stop input defined in `include/config.h`

## Elements List
- ESP32 DevKit V1
- HC-SR04 ultrasonic sensor
- DHT11 sensor
- BTS7960 motor drivers
- DC geared motors
- LoRa SX1278 module
- NEO-6M GPS module
- MQ-2 gas/smoke sensor
- Mini metal detector module
- 12V battery pack
- Buck converters for 5V and 3.3V rails
- Fuse, wiring, connectors, mounting hardware

## Project Structure
- `src/` - firmware sources
- `include/` - shared headers and configuration
- `phase*/` - stage-by-stage entry points
- `docs/` - build, wiring, and phase notes
- `platformio.ini` - PlatformIO project configuration
