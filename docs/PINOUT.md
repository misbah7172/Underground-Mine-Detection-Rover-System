# Pinout / Wiring Reference

This file lists the pin configuration used by the firmware (see `include/config.h`). Edit `include/config.h` to change pins and recompile.

## Summary (current)

- Microcontroller: ESP32 DOIT DevKit V1 (3.3V logic)

### Motor driver — L298N (configured)
- `MOTOR_L_IN1_PIN` : GPIO 26  (Left motor IN1)
- `MOTOR_L_IN2_PIN` : GPIO 27  (Left motor IN2)
- `MOTOR_L_EN_PIN`  : GPIO 25  (Left motor EN — PWM / LEDC channel 0)

- `MOTOR_R_IN1_PIN` : GPIO 14  (Right motor IN1)
- `MOTOR_R_IN2_PIN` : GPIO 12  (Right motor IN2)
- `MOTOR_R_EN_PIN`  : GPIO 13  (Right motor EN — PWM / LEDC channel 2)

Notes:
- EN pins are driven with ESP32 LEDC PWM. IN1 / IN2 are digital direction pins.
- If you use a different L298N module wiring, update the pins in `include/config.h`.

### Ultrasonic sensor (front only)
- Front TRIG: `ULTRASONIC_FRONT_TRIG_PIN` = GPIO 33
- Front ECHO: `ULTRASONIC_FRONT_ECHO_PIN` = GPIO 32

Wiring notes:
- TRIG → output pin from ESP32 (driven HIGH pulse). ECHO → input pin (measure pulse width). Use `pulseIn()` safe pin choices and avoid pins used by SPI where possible.

### DHT11 (temperature/humidity)
- `DHT11_PIN` = GPIO 4

Wiring notes:
- DHT power: 3.3V recommended. Use pull-up if required (library handles it).

### MQ-2 (gas / smoke)
- `MQ2_PIN` = ADC GPIO 34  (use an ADC-capable pin your board exposes)

Wiring notes:
- MQ-2 heater typically runs at 5V. The analog output is read by ESP32 ADC (0-3.3V). Use proper voltage divider or module that outputs safe ADC levels.

Note: some ESP32 devkits do not expose GPIO36; if your board lacks GPIO36 choose another ADC pin (GPIO32-39 range) that is available on your module (GPIO34 or GPIO35 are common alternatives).

### GPS — NEO‑6M (UART)
- `GPS_RX_PIN` = GPIO 16  (ESP32 RX2) — connect to GPS TX   ##13 
- `GPS_TX_PIN` = GPIO 17  (ESP32 TX2) — connect to GPS RX  ##25

Notes:
- Many NEO‑6M modules include a regulator; prefer 3.3V for the logic interface when possible. GPS course is available in telemetry (used for heading fallback).

### LoRa — SX1278 (SPI)
- `LORA_MOSI_PIN` = GPIO 23
- `LORA_MISO_PIN` = GPIO 19
- `LORA_SCK_PIN`  = GPIO 18 ##
- `LORA_SS_PIN`   = GPIO 5   (NSS / CS)
- `LORA_RST_PIN`  = GPIO 22
- `LORA_DIO0_PIN` = GPIO 21

Notes:
- SX1278 uses SPI (VSPI / HSPI). Recommended wiring:
	- MOSI -> GPIO23
	- MISO -> GPIO19
	- SCK  -> GPIO18
	- NSS  -> GPIO5
	- DIO0 -> GPIO21
	- RESET-> GPIO22
- Rear ultrasonic removed. GPIO39 is now used by `NE555_OUTPUT_PIN` (input-only is OK for NE555 output).

### Metal detector — NE555 module
- `NE555_OUTPUT_PIN` = GPIO 39

Notes:
- NE555 module output is typically a square wave (5V). Protect ESP32 GPIO with a series resistor (100 Ω) and a clamping/level-shifting method (e.g., resistor divider, comparator, or optocoupler) to avoid 5V on ESP32 pins. See `docs/METAL_DETECTOR_NE555.md` for details.

### Other signals
- `E_STOP_PIN` = GPIO 36 (active LOW)

### Actuator LEDs
- `HEATER_LED_PIN` = GPIO 15
- `COOLER_LED_PIN` = GPIO 2

### Telemetry & Misc
- Serial baud: `BAUD_RATE` = 115200 (Serial monitor)

## Important Conflicts & Warnings

- GPIO 27 is used as `MOTOR_L_IN2_PIN` (L298N). `NE555_OUTPUT_PIN` has been moved to GPIO39 to avoid conflict.
- Many ESP32 pins have special boot or strapping behavior — avoid using strapping pins for signals that must be driven at boot. Check ESP32 pinout before final wiring.

## How to change pin
1. Open `include/config.h` in the project root.
2. Edit the `#define` values (e.g., `MOTOR_L_IN1_PIN`, `ULTRASONIC_REAR_ECHO_PIN`, etc.) to match your wiring.
3. Rebuild and flash:

```bash
pio run
pio run -t upload
```

## Recommended safe GPIOs (examples)

If you need free pins, consider using these (avoid strapping pins): GPIO 25, 26, 27 (note: 25/26/27 are OK for outputs but check your module), GPIO 32–39 are input-capable (ADC) but some cannot be used as outputs.

## Final note
Update this file after you finalize wiring for your rover. If you want, I can automatically reassign pins to avoid conflicts and patch `include/config.h` for you.
