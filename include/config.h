#ifndef CONFIG_H
#define CONFIG_H

#define ULTRASONIC_TRIG_PIN 33
#define ULTRASONIC_ECHO_PIN 32

#define DHT11_PIN 4
#define DHT_TYPE DHT11

#define BAUD_RATE 115200

// Manual control power override
#define MANUAL_FORCE_MAX_POWER 1
#define MANUAL_MAX_POWER 255

// Actuator pins (moved off motor pins)
#define HEATER_LED_PIN 15
#define COOLER_LED_PIN 2

// Temperature thresholds (°C)
#define TEMP_HEATER_THRESHOLD 18
#define TEMP_COOLER_THRESHOLD 28

#define MAX_DISTANCE 400
#define MIN_DISTANCE 2

// Use L298N motor driver wiring (1) or BTS7960 (0)
#define MOTOR_DRIVER_L298N 1 // set 1 to use L298N wiring (low-current), 0 to keep BTS7960

// Motor driver (BTS7960) pins - Phase 1
// BTS7960 legacy pins (kept for reference)
#define MOTOR_L_PWM_A 25
#define MOTOR_L_PWM_B 26
#define MOTOR_R_PWM_A 27
#define MOTOR_R_PWM_B 14

// L298N pin mapping (IN1/IN2 + EN(PWM))
#define MOTOR_L_IN1_PIN 26
#define MOTOR_L_IN2_PIN 27
#define MOTOR_L_EN_PIN 25

#define MOTOR_R_IN1_PIN 14
#define MOTOR_R_IN2_PIN 12
#define MOTOR_R_EN_PIN 13

// L298N enable pins: 1 = PWM via ENA/ENB, 0 = ENA/ENB tied HIGH for full power
#define L298N_USE_ENABLE_PINS 0

// Emergency stop (active LOW) - moved off motor pins
#define E_STOP_PIN 36

// PWM settings
#define MOTOR_PWM_FREQ 20000
#define MOTOR_PWM_RESOLUTION 8

// Battery monitoring
#define BATTERY_ADC_PIN 35
// voltage divider: R1 (top) = 100k, R2 (bottom) = 33k -> ratio = (R1+R2)/R2
#define BATTERY_VOLT_DIVIDER_RATIO  ( (100.0 + 33.0) / 33.0 )
#define BATTERY_FULL_VOLTAGE 12.6
#define BATTERY_LOW_VOLTAGE 10.5

// LoRa (SX1278) pins (VSPI)
// SPI wiring (common VSPI pins)
#define LORA_MOSI_PIN 23
#define LORA_MISO_PIN 19
#define LORA_SCK_PIN 18
// LoRa control pins
#define LORA_SS_PIN 5   // NSS / CS
#define LORA_RST_PIN 22 // RESET
#define LORA_DIO0_PIN 21

// GPS (NEO-6M) Serial2 pins
#define GPS_RX_PIN 16 // ESP32 RX2 (connect to TX of GPS)
#define GPS_TX_PIN 17 // ESP32 TX2 (connect to RX of GPS)

// Metal detector module - NE555 frequency-based (old single-pin setup, kept for compatibility)
#define METAL_DETECTOR_PIN_OLD 34 // legacy single-pin detection

// NE555 Metal Detector - Frequency-based system
// Reassigned to free pin after removing rear ultrasonic
#define NE555_OUTPUT_PIN 39 // NE555 square wave output to this GPIO (interrupt-capable)
#define NE555_CALIBRATION_DURATION_MS 3000 // 3 seconds baseline calibration
#define NE555_SAMPLE_WINDOW_MS 100 // 100 ms frequency sample window
#define NE555_DETECTION_THRESHOLD_PCT 15 // ±15% frequency deviation triggers detection
#define NE555_SAMPLE_HISTORY_SIZE 10 // rolling average over 10 samples

// MQ-2 smoke/gas sensor analog pin
#define MQ2_PIN 34

// Ultrasonic sensor - front only
#define ULTRASONIC_FRONT_TRIG_PIN 33
#define ULTRASONIC_FRONT_ECHO_PIN 32

// SD card
// SD card (optional)
#define USE_SD 0
#define SD_CS_PIN 13

// SD log rotation settings
#define SD_LOG_MAX_BYTES (1024UL * 512UL) // 512 KB per file
#define SD_LOG_BACKUPS 3

#define USE_IMU 0

// If IMU is disabled, fallback to GPS-derived heading when speed > this (m/s)
#define HEADING_FALLBACK_SPEED_THRESHOLD_MPS 0.5f

#endif



