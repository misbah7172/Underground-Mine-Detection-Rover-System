#include <Arduino.h>
#include "config.h"
#include "motor.h"
#include "drive.h"
#include "gps.h"
#include "navigator.h"
#if USE_IMU
#include "imu_fusion.h"
#endif
#include "ultrasonic.h"
#include "loramodule.h"
#if USE_SD
#include "sd_logger.h"
#endif
#include "metal_detector_ne555.h"

// PWM channels for ESP32 LEDC
#define CHAN_ML_A 0
#define CHAN_ML_B 1
#define CHAN_MR_A 2
#define CHAN_MR_B 3

// Obstacle handling thresholds (bench-safe defaults)
static const float OBSTACLE_THRESHOLD_CM = 35.0f;
static const unsigned long TELEMETRY_INTERVAL_MS = 500;
static const unsigned long OBSTACLE_CHECK_INTERVAL_MS = 120;

#if MOTOR_DRIVER_L298N
Motor motorL(MOTOR_L_IN1_PIN, MOTOR_L_IN2_PIN, MOTOR_L_EN_PIN, CHAN_ML_A, true);
Motor motorR(MOTOR_R_IN1_PIN, MOTOR_R_IN2_PIN, MOTOR_R_EN_PIN, CHAN_MR_A, true);
#else
Motor motorL(MOTOR_L_PWM_A, MOTOR_L_PWM_B, CHAN_ML_A, CHAN_ML_B);
Motor motorR(MOTOR_R_PWM_A, MOTOR_R_PWM_B, CHAN_MR_A, CHAN_MR_B);
#endif
Drive drive(motorL, motorR);
GPSModule gps(Serial2);
WaypointNavigator navigator(drive);
#if USE_IMU
IMUFusion imu;
#endif
UltrasonicSensor frontUltrasonic(ULTRASONIC_FRONT_TRIG_PIN, ULTRASONIC_FRONT_ECHO_PIN);
LoRaModule lora(LORA_SS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
#if USE_SD
SdLogger sdLogger;
#endif
MetalDetectorNE555 metalDetector;

enum RoverMode {
    MODE_IDLE,
    MODE_AUTO,
    MODE_MANUAL,
    MODE_ESTOP
};

enum AvoidancePhase {
    AVOID_NONE,
    AVOID_BACKUP,
    AVOID_TURN
};

RoverMode currentMode = MODE_IDLE;
AvoidancePhase avoidPhase = AVOID_NONE;

bool imuReady = false;
bool missionActive = false;
double targetLat = 0.0;
double targetLon = 0.0;
float targetRadiusM = 1.0f;

unsigned long lastTelemetryMs = 0;
unsigned long lastObstacleCheckMs = 0;
unsigned long avoidPhaseStartMs = 0;
unsigned long manualUntilMs = 0;

float lastDistanceCm = -1.0f;
int avoidTurnDir = 1;
float lastHeadingDeg = 0.0f;
GPSData lastGps = {0.0, 0.0, 0.0, 0.0, NAN, 99.9, 0, false};
String serialLine;
bool metalDetectedPrevious = false; // track metal state change

static const char* modeToString(RoverMode mode) {
    switch (mode) {
        case MODE_AUTO: return "AUTO";
        case MODE_MANUAL: return "MANUAL";
        case MODE_ESTOP: return "ESTOP";
        default: return "IDLE";
    }
}

bool extractFloatField(const String& json, const char* key, float& outVal) {
    String token = String("\"") + key + "\":";
    int start = json.indexOf(token);
    if (start < 0) return false;
    start += token.length();
    while (start < json.length() && (json[start] == ' ' || json[start] == '"')) start++;
    int end = start;
    while (end < json.length()) {
        char c = json[end];
        if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E') {
            end++;
        } else {
            break;
        }
    }
    if (end <= start) return false;
    outVal = json.substring(start, end).toFloat();
    return true;
}

bool extractIntField(const String& json, const char* key, int& outVal) {
    float v;
    if (!extractFloatField(json, key, v)) return false;
    outVal = (int)v;
    return true;
}

void sendEvent(const char* level, const char* category, const String& message) {
    Serial.print("{\"type\":\"event\",\"level\":\"");
    Serial.print(level);
    Serial.print("\",\"category\":\"");
    Serial.print(category);
    Serial.print("\",\"message\":\"");
    // Basic escaping for quotes
    String safe = message;
    safe.replace("\"", "'");
    Serial.print(safe);
    Serial.print("\"");
    if (lastGps.valid) {
        Serial.print(",\"lat\":");
        Serial.print(lastGps.latitude, 6);
        Serial.print(",\"lon\":");
        Serial.print(lastGps.longitude, 6);
    }
    Serial.println("}");
}

void sendTelemetryNow() {
    String payload = "{";
    payload += String("\"type\":\"telemetry\",\"mode\":\"") + modeToString(currentMode) + "\"";
    payload += String(",\"mission_active\":") + (missionActive ? "true" : "false");
    payload += String(",\"distance_cm\":") + String(lastDistanceCm, 1);
    payload += String(",\"heading_deg\":") + String(lastHeadingDeg, 1);
    if (lastGps.valid) {
        payload += String(",\"gps_lat\":") + String(lastGps.latitude, 6);
        payload += String(",\"gps_lon\":") + String(lastGps.longitude, 6);
        payload += String(",\"speed_kph\":") + String(lastGps.speedKph, 2);
        payload += String(",\"satellites\":") + String(lastGps.satellites);
    }
    if (missionActive) {
        payload += String(",\"target_lat\":") + String(targetLat, 6);
        payload += String(",\"target_lon\":") + String(targetLon, 6);
        payload += String(",\"target_radius_m\":") + String(targetRadiusM, 3);
    }
    // mag calibration progress (only when IMU enabled)
#if USE_IMU
    if (imu.isMagCalibrating()) {
        payload += String(",\"mag_cal_active\":true");
        payload += String(",\"mag_cal_progress\":") + String(imu.getMagCalProgress());
    } else {
        payload += String(",\"mag_cal_active\":false");
    }
#else
    payload += String(",\"mag_cal_active\":false");
#endif
    // metal detector status
    payload += String(",\"metal_detected\":") + (metalDetector.isMetalDetected() ? "true" : "false");
    payload += String(",\"metal_freq_hz\":") + String(metalDetector.getCurrentFrequency(), 1);
    payload += String(",\"metal_freq_dev_pct\":") + String(metalDetector.getFrequencyDeviation(), 1);
    payload += String(",\"metal_confidence\":") + String(metalDetector.getConfidence());
    payload += "}";
    Serial.println(payload);
    // log to SD if available
#if USE_SD
    sdLogger.logLine(payload);
#endif
}

void startAvoidance(float obstacleDistanceCm) {
    avoidPhase = AVOID_BACKUP;
    avoidPhaseStartMs = millis();
    avoidTurnDir = ((millis() / 1000) % 2 == 0) ? 1 : -1;
    sendEvent("WARN", "SENSOR", String("Obstacle detected at ") + String(obstacleDistanceCm, 1) + " cm. Starting avoidance.");
}

void runAvoidance() {
    unsigned long elapsed = millis() - avoidPhaseStartMs;
    if (avoidPhase == AVOID_BACKUP) {
        if (elapsed < 700) {
            drive.setSpeed(-120, -120);
            return;
        }
        avoidPhase = AVOID_TURN;
        avoidPhaseStartMs = millis();
    }

    if (avoidPhase == AVOID_TURN) {
        unsigned long turnElapsed = millis() - avoidPhaseStartMs;
        if (turnElapsed < 1000) {
            if (avoidTurnDir > 0) drive.setSpeed(120, -120);
            else drive.setSpeed(-120, 120);
            return;
        }
        avoidPhase = AVOID_NONE;
        sendEvent("INFO", "NAV", "Avoidance complete, resuming navigation");
    }
}

void handleDriveCommand(const String& line) {
    int left = 0;
    int right = 0;
    int durationMs = 800;
    extractIntField(line, "left", left);
    extractIntField(line, "right", right);
    extractIntField(line, "duration_ms", durationMs);
#if MANUAL_FORCE_MAX_POWER
    if (left != 0) left = left > 0 ? MANUAL_MAX_POWER : -MANUAL_MAX_POWER;
    if (right != 0) right = right > 0 ? MANUAL_MAX_POWER : -MANUAL_MAX_POWER;
#endif
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);
    drive.setSpeed(left, right);
    currentMode = MODE_MANUAL;
    manualUntilMs = millis() + (unsigned long)max(durationMs, 200);
}

void handleMissionStart(const String& line) {
    float lat = 0.0f, lon = 0.0f;
    float radiusCm = 100.0f;
    float radiusM = -1.0f;

    bool hasLat = extractFloatField(line, "lat", lat);
    bool hasLon = extractFloatField(line, "lon", lon);
    bool hasRadiusCm = extractFloatField(line, "radius_cm", radiusCm);
    bool hasRadiusM = extractFloatField(line, "radius_m", radiusM);

    if (!hasRadiusCm) {
        float radiusRaw = 0.0f;
        if (extractFloatField(line, "radius", radiusRaw)) {
            // Dashboard sends radius in cm for bench testing.
            radiusCm = radiusRaw;
            hasRadiusCm = true;
        }
    }

    if (!hasLat || !hasLon) {
        sendEvent("ERROR", "MISSION", "Mission start rejected: lat/lon missing");
        return;
    }

    targetLat = lat;
    targetLon = lon;
    targetRadiusM = hasRadiusM ? max(radiusM, 0.1f) : max(radiusCm / 100.0f, 0.1f);

    navigator.setGoal(targetLat, targetLon, targetRadiusM);
    missionActive = true;
    currentMode = MODE_AUTO;
    avoidPhase = AVOID_NONE;

    sendEvent("INFO", "MISSION", String("Mission accepted: target=") + String(targetLat, 6) + "," + String(targetLon, 6));
    // try to forward mission over LoRa and require ACK
    if (lora.begin(433E6)) {
        bool queued = lora.sendReliable(line.c_str(), 4, 1500);
        if (!queued) sendEvent("WARN", "LORA", "Mission forward failed to queue (pending full)");
        else sendEvent("INFO", "LORA", "Mission queued for reliable LoRa forward (msg_id assigned)");
    } else {
        sendEvent("WARN", "LORA", "LoRa not initialized; mission not forwarded") ;
    }
}

void handleCommandLine(const String& line) {
    if (line.length() == 0) return;

    if (line.indexOf("\"mission\":\"start\"") >= 0) {
        handleMissionStart(line);
        return;
    }

    if (line.indexOf("\"cmd\":\"drive\"") >= 0) {
        if (currentMode != MODE_ESTOP) handleDriveCommand(line);
        return;
    }

    if (line.indexOf("\"cmd\":\"stop\"") >= 0) {
        drive.stop();
        currentMode = missionActive ? MODE_AUTO : MODE_IDLE;
        return;
    }

    if (line.indexOf("\"cmd\":\"estop\"") >= 0) {
        drive.stop();
        currentMode = MODE_ESTOP;
        missionActive = false;
        avoidPhase = AVOID_NONE;
        sendEvent("WARN", "SAFETY", "Emergency stop latched");
        return;
    }

    if (line.indexOf("\"cmd\":\"clear_estop\"") >= 0) {
        if (currentMode == MODE_ESTOP) {
            currentMode = MODE_IDLE;
            sendEvent("INFO", "SAFETY", "Emergency stop cleared");
        }
        return;
    }

    if (line.indexOf("\"cmd\":\"ping\"") >= 0) {
        Serial.println("{\"type\":\"ack\",\"message\":\"pong\"}");
        return;
    }

    if (line.indexOf("\"cmd\":\"status\"") >= 0) {
        sendTelemetryNow();
        return;
    }

    // Magnetometer calibration commands (non-blocking)
    if (line.indexOf("\"cmd\":\"calibrate_mag\"") >= 0) {
        int dur = 15;
        extractIntField(line, "duration_s", dur);
        sendEvent("INFO", "CAL", String("Starting magnetometer calibration for ") + String(dur) + " s");
    #if USE_IMU
        imu.startMagCalibration((unsigned int)max(1, dur));
    #else
        sendEvent("WARN", "CAL", "IMU disabled: magnetometer calibration not available");
    #endif
        return;
    }

    if (line.indexOf("\"cmd\":\"calibrate_mag_stop\"") >= 0) {
    #if USE_IMU
        imu.stopMagCalibration();
        sendEvent("INFO", "CAL", "Magnetometer calibration stopped by command");
    #else
        sendEvent("WARN", "CAL", "IMU disabled: no calibration to stop");
    #endif
        return;
    }
}

void readSerialCommands() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            String line = serialLine;
            serialLine = "";
            line.trim();
            if (line.length() > 0) handleCommandLine(line);
        } else if (c != '\r') {
            serialLine += c;
            if (serialLine.length() > 400) serialLine = "";
        }
    }
}

void setup() {
    Serial.begin(BAUD_RATE);
    delay(200);
    Serial.println("Rover mission controller booting...");

    drive.begin();
    gps.begin(9600);
    navigator.begin();
    frontUltrasonic.init();

#if USE_IMU
    imuReady = imu.begin();
    if (imuReady) sendEvent("INFO", "IMU", "IMU fusion initialized");
    else sendEvent("WARN", "IMU", "IMU init failed, heading fallback enabled");
#else
    imuReady = false;
    sendEvent("INFO", "IMU", "IMU disabled at compile time; heading fallback enabled");
#endif

    // initialize SD logger
#if USE_SD
    if (sdLogger.begin()) sendEvent("INFO", "SD", "SD logger ready");
    else sendEvent("WARN", "SD", "SD logger not available");
#else
    sendEvent("INFO", "SD", "SD logger disabled in config");
#endif

    // initialize LoRa (optional)
    if (lora.begin(433E6)) sendEvent("INFO", "LORA", "LoRa initialized");
    else sendEvent("WARN", "LORA", "LoRa init failed or not present");

    // initialize metal detector NE555
    metalDetector.begin();
    sendEvent("INFO", "METAL", "Starting metal detector calibration (3 sec, keep away from metal)...");
    if (metalDetector.calibrate()) {
        sendEvent("INFO", "METAL", String("Metal detector calibrated. Baseline: ") + String(metalDetector.getBaselineFrequency(), 1) + " Hz");
    } else {
        sendEvent("WARN", "METAL", "Metal detector calibration failed");
    }

    sendEvent("INFO", "SYSTEM", "Mission controller ready");
}

void loop() {
    static unsigned long lastMicros = micros();
    unsigned long nowMicros = micros();
    float dt = (nowMicros - lastMicros) / 1000000.0f;
    if (dt <= 0.0f) dt = 0.02f;
    lastMicros = nowMicros;

    readSerialCommands();

    String loraLine;
    while (lora.readMessage(loraLine)) {
        loraLine.trim();
        if (loraLine.length() > 0) handleCommandLine(loraLine);
    }

    gps.feed();
    lastGps = gps.getData();

#if USE_IMU
    if (imuReady) {
        imu.update();
        lastHeadingDeg = imu.getHeading();
    } else {
        // Heading fallback: use GPS course when moving above threshold
        float fallbackKph = HEADING_FALLBACK_SPEED_THRESHOLD_MPS * 3.6f;
        if (lastGps.valid && lastGps.speedKph > fallbackKph && !isnan(lastGps.courseDeg)) {
            lastHeadingDeg = lastGps.courseDeg;
        }
    }
#else
    // Heading fallback: use GPS course when moving above threshold
    float fallbackKph = HEADING_FALLBACK_SPEED_THRESHOLD_MPS * 3.6f;
    if (lastGps.valid && lastGps.speedKph > fallbackKph && !isnan(lastGps.courseDeg)) {
        lastHeadingDeg = lastGps.courseDeg;
    }
#endif

    // update metal detector frequency measurement
    metalDetector.update();
    
    // handle metal detection state change
    bool metalNow = metalDetector.isMetalDetected();
    if (metalNow && !metalDetectedPrevious) {
        // Metal just detected!
        sendEvent("WARN", "METAL", String("*** METAL DETECTED *** Freq dev: ") + String(metalDetector.getFrequencyDeviation(), 1) + "% Confidence: " + String(metalDetector.getConfidence()) + "%");
        drive.stop(); // emergency stop
        currentMode = MODE_IDLE;
        missionActive = false;
        // alert via LoRa if available and GPS lock present
        if (lastGps.valid) {
            String alert = String("{\"type\":\"alert\",\"alert_type\":\"metal_detected\",\"lat\":") + String(lastGps.latitude, 6)
                         + String(",\"lon\":") + String(lastGps.longitude, 6)
                         + String(",\"freq_hz\":") + String(metalDetector.getCurrentFrequency(), 1)
                         + String(",\"confidence\":") + String(metalDetector.getConfidence())
                         + String("}");
            bool queued = lora.sendReliable(alert.c_str(), 4, 1500);
            if (queued) sendEvent("INFO", "LORA", "Metal alert queued for LoRa transmission");
        }
    }
    metalDetectedPrevious = metalNow;

    if (millis() - lastObstacleCheckMs >= OBSTACLE_CHECK_INTERVAL_MS) {
        lastObstacleCheckMs = millis();
        lastDistanceCm = frontUltrasonic.getDistance();
    }

    if (currentMode == MODE_ESTOP) {
        drive.stop();
    } else if (currentMode == MODE_MANUAL) {
        if (millis() > manualUntilMs) {
            drive.stop();
            currentMode = missionActive ? MODE_AUTO : MODE_IDLE;
        }
    } else if (missionActive && currentMode == MODE_AUTO) {
        if (avoidPhase != AVOID_NONE) {
            runAvoidance();
        } else if (lastDistanceCm > 0 && lastDistanceCm < OBSTACLE_THRESHOLD_CM) {
            startAvoidance(lastDistanceCm);
            runAvoidance();
        } else if (lastGps.valid) {
            bool arrived = navigator.update(lastGps, lastHeadingDeg, dt);
            if (arrived) {
                missionActive = false;
                currentMode = MODE_IDLE;
                drive.stop();
                sendEvent("INFO", "MISSION", "Target reached");
            }
        } else {
            // Hold if GPS fix is unavailable while in auto mode
            drive.stop();
        }
    } else {
        drive.stop();
    }

    // poll LoRa for incoming messages and reliable-send retries
    lora.poll();

    if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = millis();
        sendTelemetryNow();
    }

    delay(10);
}
