#pragma once

#include <Arduino.h>

// Serial dashboard link
#define SERIAL_BAUD 115200

// L298N motor driver input pins. ENA/ENB are expected to be jumpered HIGH.
#define MOTOR_IN1_PIN 26
#define MOTOR_IN2_PIN 27
#define MOTOR_IN3_PIN 14
#define MOTOR_IN4_PIN 12

// Sensors
#define ULTRASONIC_TRIG_PIN 33
#define ULTRASONIC_ECHO_PIN 32
#define METAL_ADC_PIN 35
#define MQ2_ADC_PIN 34
#define DHT11_DATA_PIN 15

// Timed local path follower tuning. Adjust after real floor tests.
#define LOCAL_PATH_MAX_POINTS 60
#define LOCAL_PATH_INITIAL_HEADING_DEG 90.0f
#define LOCAL_PATH_MS_PER_CM 45.0f
#define LOCAL_PATH_MS_PER_DEG 7.0f
#define LOCAL_PATH_FORWARD_SPEED 180
#define LOCAL_PATH_TURN_SPEED 170
#define LOCAL_PATH_STOP_AFTER_SEGMENT_MS 120

// Obstacle avoidance
#define OBSTACLE_CHECK_INTERVAL_MS 220UL
#define OBSTACLE_DISTANCE_CM 22.0f
#define OBSTACLE_BACKUP_MS 450UL
#define OBSTACLE_TURN_MS 550UL
#define OBSTACLE_SIDESTEP_FORWARD_MS 650UL
#define OBSTACLE_MAX_AVOIDANCE_PER_TARGET 5
#define ULTRASONIC_TIMEOUT_US 8000UL

// NE555 metal detector on GPIO 35.
// Active-low detector: metal pulls ADC near 0; no metal is usually near 4095.
#define METAL_CALIBRATION_MS 2000UL
#define METAL_SAMPLE_INTERVAL_MS 10UL
#define METAL_ADC_DETECT_MIN 0
#define METAL_ADC_DETECT_MAX 100
#define METAL_ALERT_COOLDOWN_MS 350UL
#define METAL_DETECTION_STOPS_MISSION 0

// DHT11 environmental sensor
#define DHT11_SAMPLE_INTERVAL_MS 1000UL

// Telemetry
#define TELEMETRY_INTERVAL_MS 150UL
#define TELEMETRY_DETAIL_INTERVAL_MS 1000UL

// Optional WiFi UDP alerts. Leave disabled until serial control works.
#define WIFI_ALERT_SSID "vivoY21"
#define WIFI_ALERT_PASSWORD "154089@UIU"
#define WIFI_ALERT_HOST "172.30.204.254"
#define WIFI_ALERT_ENABLED    1
#define WIFI_ALERT_PORT       4210               // dashboard listens here

// Add this new line:
#define WIFI_CMD_PORT         4211               // ESP32 listens for commands here