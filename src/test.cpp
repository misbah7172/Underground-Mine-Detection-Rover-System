#include <Arduino.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

#include "config.h"

struct PointCm {
  float x;
  float y;
};

enum RobotMode {
  MODE_IDLE,
  MODE_MANUAL,
  MODE_MISSION,
  MODE_ESTOP
};

PointCm missionPoints[LOCAL_PATH_MAX_POINTS];
int missionPointCount = 0;
int missionTargetIndex = -1;
PointCm rememberedTarget = {0.0f, 0.0f};
bool rememberedTargetValid = false;
int obstacleAvoidanceCount = 0;
float lastObstacleDistanceCm = -1.0f;
bool avoidingObstacle = false;

RobotMode robotMode = MODE_IDLE;
bool missionActive = false;
bool missionAbortRequested = false;
bool estopLatched = false;

float poseXcm = 0.0f;
float poseYcm = 0.0f;
float headingDeg = LOCAL_PATH_INITIAL_HEADING_DEG;

String serialBuffer;
unsigned long manualStopAtMs = 0;
unsigned long lastTelemetryAtMs = 0;
unsigned long lastDetailedTelemetryAtMs = 0;
unsigned long lastMetalSampleAtMs = 0;
unsigned long lastMetalAlertAtMs = 0;
unsigned long metalPauseUntilMs = 0;   // non-zero while rover is paused for metal detection
unsigned long lastDhtSampleAtMs = 0;

long metalBaselineAdc = 0;
int metalAdc = 0;
int metalAdcMin = 4095;
int metalAdcMax = 0;
int metalAdcDrop = 0;
bool metalDetected = false;
bool previousMetalDetected = false;

DHT dht(DHT11_DATA_PIN, DHT11);
float dhtTemperatureC = NAN;
float dhtHumidity = NAN;
bool dhtOk = false;

WiFiUDP cmdUdp;                        // receives commands from dashboard
bool cmdUdpReady = false;

WiFiUDP udp;
bool wifiAlertReady = false;
String wifiAlertHost = WIFI_ALERT_HOST;
uint16_t wifiAlertPort = WIFI_ALERT_PORT;
String wifiAlertSsid = "";

const char* modeName() {
  switch (robotMode) {
    case MODE_IDLE: return "IDLE";
    case MODE_MANUAL: return "MANUAL";
    case MODE_MISSION: return "MISSION";
    case MODE_ESTOP: return "ESTOP";
    default: return "UNKNOWN";
  }
}

float wrap360(float degrees) {
  while (degrees < 0.0f) degrees += 360.0f;
  while (degrees >= 360.0f) degrees -= 360.0f;
  return degrees;
}

float normalizeDeltaDeg(float degrees) {
  while (degrees > 180.0f) degrees -= 360.0f;
  while (degrees < -180.0f) degrees += 360.0f;
  return degrees;
}

void writeJsonLine(JsonDocument& doc) {
  serializeJson(doc, Serial);
  Serial.println();
}

void sendEvent(const char* level, const char* category, const String& message) {
  StaticJsonDocument<256> doc;
  doc["type"] = "event";
  doc["level"] = level;
  doc["category"] = category;
  doc["message"] = message;
  writeJsonLine(doc);
}

void stopMotors() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  digitalWrite(MOTOR_IN3_PIN, LOW);
  digitalWrite(MOTOR_IN4_PIN, LOW);
}

void setMotorChannel(int speed, int pinA, int pinB) {
  if (speed > 10) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (speed < -10) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
}

void driveRaw(int leftSpeed, int rightSpeed) {
  if (estopLatched) {
    stopMotors();
    return;
  }

  // ENA/ENB are jumpered HIGH, so speed magnitude is treated as an on/off threshold.
  setMotorChannel(leftSpeed, MOTOR_IN1_PIN, MOTOR_IN2_PIN);
  setMotorChannel(rightSpeed, MOTOR_IN3_PIN, MOTOR_IN4_PIN);
}

void updatePoseByDistance(float distanceCm) {
  float headingRad = headingDeg * PI / 180.0f;
  poseXcm += cosf(headingRad) * distanceCm;
  poseYcm += sinf(headingRad) * distanceCm;
}

float readDistanceCm() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  unsigned long durationUs = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (durationUs == 0) {
    return -1.0f;
  }
  return durationUs / 58.0f;
}

void sendAlert(float xCm, float yCm) {
  StaticJsonDocument<256> doc;
  doc["type"] = "alert";
  doc["alert_type"] = "metal_detected";
  doc["x_cm"] = xCm;
  doc["y_cm"] = yCm;
  doc["adc"] = metalAdc;
  doc["adc_baseline"] = metalBaselineAdc;
  doc["adc_drop"] = metalAdcDrop;
  doc["adc_detect_min"] = METAL_ADC_DETECT_MIN;
  doc["adc_detect_max"] = METAL_ADC_DETECT_MAX;
  // Active-low detector: ADC near 0 is strongest metal signal; no metal is ~4095.
  int detectSpan = max(1, METAL_ADC_DETECT_MAX - METAL_ADC_DETECT_MIN);
  int confidenceSignal = constrain(METAL_ADC_DETECT_MAX - metalAdc, 0, detectSpan);
  doc["confidence"] = constrain(map(confidenceSignal, 0, detectSpan, 55, 99), 55, 99);

  char payload[256];
  size_t len = serializeJson(doc, payload, sizeof(payload));
  Serial.println(payload);

  if (wifiAlertReady) {
    udp.beginPacket(wifiAlertHost.c_str(), wifiAlertPort);
    udp.write(reinterpret_cast<const uint8_t*>(payload), len);
    udp.endPacket();
  }
}

void sendTelemetry(bool detailed = false) {
  StaticJsonDocument<1024> doc;
  doc["type"] = "telemetry";
  doc["mode"] = modeName();
  doc["mission_active"] = missionActive;
  doc["metal_detected"] = metalDetected;
  doc["metal_adc"] = metalAdc;
  doc["metal_adc_min"] = metalAdcMin;
  doc["metal_adc_max"] = metalAdcMax;
  doc["metal_adc_detect_min"] = METAL_ADC_DETECT_MIN;
  doc["metal_adc_detect_max"] = METAL_ADC_DETECT_MAX;
  doc["x_cm"] = poseXcm;
  doc["y_cm"] = poseYcm;
  doc["heading_deg"] = headingDeg;
  doc["telemetry_detail"] = detailed;

  if (detailed) {
    doc["distance_cm"] = readDistanceCm();
    doc["metal_baseline"] = metalBaselineAdc;
    doc["metal_adc_drop"] = metalAdcDrop;
    doc["mission_target_index"] = missionTargetIndex;
    doc["mission_points"] = missionPointCount;
    doc["remembered_target_valid"] = rememberedTargetValid;
    if (rememberedTargetValid) {
      doc["remembered_target_x_cm"] = rememberedTarget.x;
      doc["remembered_target_y_cm"] = rememberedTarget.y;
    }
    doc["avoiding_obstacle"] = avoidingObstacle;
    doc["obstacle_count"] = obstacleAvoidanceCount;
    doc["last_obstacle_cm"] = lastObstacleDistanceCm;
    doc["dht11_ok"] = dhtOk;
    if (dhtOk) {
      doc["dht11_temperature_c"] = dhtTemperatureC;
      doc["dht11_humidity_percent"] = dhtHumidity;
    } else {
      doc["dht11_temperature_c"] = nullptr;
      doc["dht11_humidity_percent"] = nullptr;
    }
    doc["wifi_alert_ready"] = wifiAlertReady;
    doc["wifi_alert_host"] = wifiAlertHost;
    doc["wifi_alert_port"] = wifiAlertPort;
    if (WiFi.status() == WL_CONNECTED) {
      doc["wifi_ip"] = WiFi.localIP().toString();
      doc["wifi_ssid"] = WiFi.SSID();
    } else {
      doc["wifi_ip"] = nullptr;
      doc["wifi_ssid"] = wifiAlertSsid;
    }
  }
  writeJsonLine(doc);
  metalAdcMin = metalAdc;
  metalAdcMax = metalAdc;
}

void sampleDht11() {
  unsigned long now = millis();
  if (now - lastDhtSampleAtMs < DHT11_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastDhtSampleAtMs = now;

  float humidity = dht.readHumidity();
  float temperatureC = dht.readTemperature();
  if (isnan(humidity) || isnan(temperatureC)) {
    dhtOk = false;
    return;
  }

  dhtHumidity = humidity;
  dhtTemperatureC = temperatureC;
  dhtOk = true;
}

void sampleMetalDetector(bool allowMissionStop) {
  unsigned long now = millis();
  if (now - lastMetalSampleAtMs < METAL_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastMetalSampleAtMs = now;

  metalAdc = analogRead(METAL_ADC_PIN);
  if (metalAdc < metalAdcMin) metalAdcMin = metalAdc;
  if (metalAdc > metalAdcMax) metalAdcMax = metalAdc;
  metalAdcDrop = static_cast<int>(metalBaselineAdc - metalAdc);
  previousMetalDetected = metalDetected;
  metalDetected = metalAdc >= METAL_ADC_DETECT_MIN && metalAdc <= METAL_ADC_DETECT_MAX;

  if (metalDetected && now - lastMetalAlertAtMs >= METAL_ALERT_COOLDOWN_MS) {
    lastMetalAlertAtMs = now;
    sendAlert(poseXcm, poseYcm);
    lastTelemetryAtMs = now;
    sendTelemetry(false);

    // Pause the rover for 3 seconds on metal detection, then resume the mission.
    if (allowMissionStop) {
      stopMotors();
      metalPauseUntilMs = now + 3000UL;
      sendEvent("WARN", "METAL", "Metal detected on GPIO35; rover paused for 3 s then will resume mission");
    }
#if METAL_DETECTION_STOPS_MISSION
    // Legacy flag: abort mission entirely instead of pausing.
    if (allowMissionStop && !metalPauseUntilMs) {
      missionAbortRequested = true;
      stopMotors();
      sendEvent("WARN", "METAL", "Metal detected; mission stopped because METAL_DETECTION_STOPS_MISSION=1");
    }
#else
    (void)allowMissionStop;
#endif
  }

  if (metalDetected != previousMetalDetected) {
    StaticJsonDocument<160> doc;
    doc["type"] = "event";
    doc["level"] = metalDetected ? "WARN" : "INFO";
    doc["category"] = "METAL";
    doc["message"] = metalDetected ? "Metal detected: GPIO35 ADC is active-low" : "Metal cleared";
    doc["adc"] = metalAdc;
    writeJsonLine(doc);
    lastTelemetryAtMs = now;
    sendTelemetry(false);
  }
}

void pollSerial(bool duringMotion);
void pollCmdUdp(bool duringMotion);

void maintainBackground(bool allowMissionStop) {
  pollSerial(allowMissionStop);
  pollCmdUdp(allowMissionStop);    // <-- add this line
  sampleMetalDetector(allowMissionStop);
  sampleDht11();

  unsigned long now = millis();
  if (now - lastTelemetryAtMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryAtMs = now;
    bool detailedTelemetry = now - lastDetailedTelemetryAtMs >= TELEMETRY_DETAIL_INTERVAL_MS;
    if (detailedTelemetry) {
      lastDetailedTelemetryAtMs = now;
    }
    sendTelemetry(detailedTelemetry);
  }
}

// Block execution (motors already stopped by sampleMetalDetector) until the
// 3-second metal pause expires, keeping serial/telemetry alive during the wait.
void waitForMetalPause() {
  while (metalPauseUntilMs > 0 && millis() < metalPauseUntilMs && !missionAbortRequested && !estopLatched) {
    maintainBackground(false);  // don't trigger another pause while waiting
    delay(5);
  }
  if (metalPauseUntilMs > 0 && millis() >= metalPauseUntilMs) {
    metalPauseUntilMs = 0;
    sendEvent("INFO", "METAL", "Metal pause complete; resuming mission");
  }
}

void timedMotorAction(int leftSpeed, int rightSpeed, unsigned long durationMs, bool allowMissionStop) {
  waitForMetalPause();  // honour any active metal-detection pause before moving
  if (missionAbortRequested || estopLatched) return;

  unsigned long start = millis();
  driveRaw(leftSpeed, rightSpeed);
  while (!missionAbortRequested && !estopLatched && millis() - start < durationMs) {
    maintainBackground(allowMissionStop);
    if (metalPauseUntilMs > 0) {          // metal detected mid-move: freeze then resume
      stopMotors();
      waitForMetalPause();
      if (missionAbortRequested || estopLatched) break;
      driveRaw(leftSpeed, rightSpeed);    // restart motors after pause
    }
    delay(5);
  }
  stopMotors();
  delay(LOCAL_PATH_STOP_AFTER_SEGMENT_MS);
}

void avoidObstacleManeuver(PointCm target) {
  avoidingObstacle = true;
  rememberedTarget = target;
  rememberedTargetValid = true;
  obstacleAvoidanceCount++;

  sendEvent("WARN", "OBSTACLE", String("Obstacle detected at ") + lastObstacleDistanceCm + "cm; detouring then resuming remembered target x=" + target.x + " y=" + target.y);
  stopMotors();

  float originalHeading = headingDeg;

  timedMotorAction(-LOCAL_PATH_FORWARD_SPEED, -LOCAL_PATH_FORWARD_SPEED, OBSTACLE_BACKUP_MS, true);
  updatePoseByDistance(-static_cast<float>(OBSTACLE_BACKUP_MS) / LOCAL_PATH_MS_PER_CM);

  float turnDegrees = OBSTACLE_TURN_MS / LOCAL_PATH_MS_PER_DEG;
  timedMotorAction(LOCAL_PATH_TURN_SPEED, -LOCAL_PATH_TURN_SPEED, OBSTACLE_TURN_MS, true);
  headingDeg = wrap360(headingDeg - turnDegrees);

  timedMotorAction(LOCAL_PATH_FORWARD_SPEED, LOCAL_PATH_FORWARD_SPEED, OBSTACLE_SIDESTEP_FORWARD_MS, true);
  updatePoseByDistance(static_cast<float>(OBSTACLE_SIDESTEP_FORWARD_MS) / LOCAL_PATH_MS_PER_CM);

  timedMotorAction(-LOCAL_PATH_TURN_SPEED, LOCAL_PATH_TURN_SPEED, OBSTACLE_TURN_MS, true);
  headingDeg = originalHeading;

  avoidingObstacle = false;
  sendEvent("INFO", "PATH", String("Obstacle avoided. Remembered path target still x=") + target.x + " y=" + target.y + "; recalculating route.");
}

void turnToHeading(float targetHeadingDeg) {
  targetHeadingDeg = wrap360(targetHeadingDeg);
  float delta = normalizeDeltaDeg(targetHeadingDeg - headingDeg);
  if (fabs(delta) < 3.0f) {
    headingDeg = targetHeadingDeg;
    return;
  }

  unsigned long durationMs = static_cast<unsigned long>(fabs(delta) * LOCAL_PATH_MS_PER_DEG);
  if (delta > 0.0f) {
    // Counter-clockwise / left turn.
    timedMotorAction(-LOCAL_PATH_TURN_SPEED, LOCAL_PATH_TURN_SPEED, durationMs, true);
  } else {
    // Clockwise / right turn.
    timedMotorAction(LOCAL_PATH_TURN_SPEED, -LOCAL_PATH_TURN_SPEED, durationMs, true);
  }

  if (!missionAbortRequested && !estopLatched) {
    headingDeg = targetHeadingDeg;
  }
}

bool driveForwardTo(PointCm target, float distanceCm) {
  PointCm startPose = {poseXcm, poseYcm};
  unsigned long durationMs = static_cast<unsigned long>(distanceCm * LOCAL_PATH_MS_PER_CM);
  if (durationMs < 1) {
    poseXcm = target.x;
    poseYcm = target.y;
    return true;
  }

  waitForMetalPause();  // honour any active metal-detection pause before moving
  if (missionAbortRequested || estopLatched) return false;

  unsigned long start = millis();
  unsigned long lastObstacleCheck = 0;
  driveRaw(LOCAL_PATH_FORWARD_SPEED, LOCAL_PATH_FORWARD_SPEED);

  while (!missionAbortRequested && !estopLatched) {
    unsigned long now = millis();
    unsigned long elapsed = now - start;
    float progress = min(1.0f, static_cast<float>(elapsed) / static_cast<float>(durationMs));
    poseXcm = startPose.x + (target.x - startPose.x) * progress;
    poseYcm = startPose.y + (target.y - startPose.y) * progress;

    if (elapsed >= durationMs) {
      break;
    }

    maintainBackground(true);

    if (metalPauseUntilMs > 0) {         // metal detected mid-forward-drive: freeze then resume
      stopMotors();
      waitForMetalPause();
      if (missionAbortRequested || estopLatched) break;
      start = millis() - elapsed;        // shift start so remaining time is preserved
      driveRaw(LOCAL_PATH_FORWARD_SPEED, LOCAL_PATH_FORWARD_SPEED);
    }

    if (now - lastObstacleCheck >= OBSTACLE_CHECK_INTERVAL_MS) {
      lastObstacleCheck = now;
      float distance = readDistanceCm();
      if (distance > 0.0f && distance <= OBSTACLE_DISTANCE_CM) {
        lastObstacleDistanceCm = distance;
        avoidObstacleManeuver(target);
        return false;
      }
    }

    delay(5);
  }

  stopMotors();
  delay(LOCAL_PATH_STOP_AFTER_SEGMENT_MS);

  if (!missionAbortRequested && !estopLatched) {
    poseXcm = target.x;
    poseYcm = target.y;
  }
  return !missionAbortRequested && !estopLatched;
}

void driveToPoint(PointCm target) {
  rememberedTarget = target;
  rememberedTargetValid = true;
  int avoidanceAttemptsForTarget = 0;

  while (!missionAbortRequested && !estopLatched) {
    float dx = target.x - poseXcm;
    float dy = target.y - poseYcm;
    float distanceCm = sqrtf(dx * dx + dy * dy);
    if (distanceCm < 0.5f) {
      poseXcm = target.x;
      poseYcm = target.y;
      return;
    }

    float targetHeading = wrap360(atan2f(dy, dx) * 180.0f / PI);
    turnToHeading(targetHeading);
    if (missionAbortRequested || estopLatched) {
      return;
    }

    bool reached = driveForwardTo(target, distanceCm);
    if (reached) {
      return;
    }

    avoidanceAttemptsForTarget++;
    if (avoidanceAttemptsForTarget >= OBSTACLE_MAX_AVOIDANCE_PER_TARGET) {
      missionAbortRequested = true;
      stopMotors();
      sendEvent("ERROR", "OBSTACLE", String("Too many obstacles while trying to reach remembered target index ") + missionTargetIndex + "; mission aborted for safety.");
      return;
    }

    sendEvent("INFO", "PATH", String("Resuming remembered path to target index ") + missionTargetIndex + " attempt " + (avoidanceAttemptsForTarget + 1));
  }
}

void executeMission() {
  missionAbortRequested = false;
  missionActive = true;
  robotMode = MODE_MISSION;
  poseXcm = 0.0f;
  poseYcm = 0.0f;
  headingDeg = LOCAL_PATH_INITIAL_HEADING_DEG;
  missionTargetIndex = -1;
  rememberedTargetValid = false;
  obstacleAvoidanceCount = 0;
  lastObstacleDistanceCm = -1.0f;
  avoidingObstacle = false;

  sendEvent("INFO", "MISSION", String("Starting local path mission with ") + missionPointCount + " point(s)");

  for (int i = 0; i < missionPointCount; i++) {
    if (missionAbortRequested || estopLatched) {
      break;
    }
    missionTargetIndex = i;
    rememberedTarget = missionPoints[i];
    rememberedTargetValid = true;
    driveToPoint(missionPoints[i]);
  }

  stopMotors();
  missionActive = false;
  if (!estopLatched) {
    robotMode = MODE_IDLE;
  }

  if (missionAbortRequested || estopLatched) {
    sendEvent("WARN", "MISSION", "Mission aborted");
  } else {
    sendEvent("INFO", "MISSION", "Mission complete");
  }
  missionTargetIndex = -1;
  sendTelemetry(true);
}

void handleCommand(JsonDocument& doc, bool duringMotion) {
  const char* cmd = doc["cmd"] | "";

  if (strcmp(cmd, "ping") == 0) {
    StaticJsonDocument<128> reply;
    reply["type"] = "event";
    reply["level"] = "INFO";
    reply["category"] = "LINK";
    reply["message"] = "pong";
    writeJsonLine(reply);
    return;
  }

  if (strcmp(cmd, "status") == 0) {
    sendTelemetry(true);
    return;
  }

  if (strcmp(cmd, "test_metal_alert") == 0) {
    metalAdc = analogRead(METAL_ADC_PIN);
    metalAdcDrop = static_cast<int>(metalBaselineAdc - metalAdc);
    sendEvent("INFO", "METAL", "Sending test metal alert at current rover position");
    sendAlert(poseXcm, poseYcm);
    return;
  }

  if (strcmp(cmd, "estop") == 0) {
    estopLatched = true;
    missionAbortRequested = true;
    robotMode = MODE_ESTOP;
    stopMotors();
    sendEvent("ERROR", "SAFETY", "Emergency stop latched. Send reset_estop before moving again.");
    return;
  }

  if (strcmp(cmd, "reset_estop") == 0) {
    estopLatched = false;
    missionAbortRequested = false;
    robotMode = MODE_IDLE;
    stopMotors();
    sendEvent("INFO", "SAFETY", "Emergency stop reset");
    return;
  }

  if (strcmp(cmd, "stop") == 0) {
    missionAbortRequested = true;
    manualStopAtMs = 0;
    stopMotors();
    if (!estopLatched) {
      robotMode = MODE_IDLE;
    }
    sendEvent("WARN", "MOTOR", "Stop command received");
    return;
  }

  if (estopLatched) {
    sendEvent("WARN", "SAFETY", "Ignoring motion command while emergency stop is latched");
    return;
  }

  if (duringMotion || missionActive) {
    sendEvent("WARN", "BUSY", String("Ignoring command while mission is active: ") + cmd);
    return;
  }

  if (strcmp(cmd, "wifi_config") == 0) {
    const char* ssid = doc["ssid"] | "";
    const char* password = doc["password"] | "";
    const char* host = doc["host"] | "";
    uint16_t port = doc["port"] | WIFI_ALERT_PORT;

    if (strlen(ssid) == 0 || strlen(host) == 0 || port == 0) {
      wifiAlertReady = false;
      sendEvent("ERROR", "WIFI", "wifi_config requires ssid, host, and valid port");
      return;
    }

    wifiAlertSsid = ssid;
    wifiAlertHost = host;
    wifiAlertPort = port;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(false, false);
    delay(100);
    WiFi.begin(ssid, password);

    sendEvent("INFO", "WIFI", String("Connecting to SSID '") + ssid + "' for UDP alerts to " + wifiAlertHost + ":" + wifiAlertPort);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 12000UL) {
      delay(250);
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifiAlertReady = true;
      sendEvent("INFO", "WIFI", String("WiFi UDP alerts ready. rover_ip=") + WiFi.localIP().toString() + " host=" + wifiAlertHost + ":" + wifiAlertPort);
    } else {
      wifiAlertReady = false;
      sendEvent("ERROR", "WIFI", "WiFi connection failed; metal alerts will continue over serial only");
    }
    sendTelemetry(true);
    return;
  }

  if (strcmp(cmd, "drive") == 0) {
    int left = doc["left"] | 0;
    int right = doc["right"] | 0;
    unsigned long durationMs = doc["duration_ms"] | 0UL;
    driveRaw(left, right);
    robotMode = MODE_MANUAL;
    manualStopAtMs = durationMs > 0 ? millis() + durationMs : 0;
    sendEvent("INFO", "MOTOR", String("Drive left=") + left + " right=" + right + " duration_ms=" + durationMs);
    return;
  }

  if (strcmp(cmd, "path") == 0) {
    JsonArray points = doc["points"].as<JsonArray>();
    missionPointCount = 0;
    for (JsonVariant point : points) {
      if (missionPointCount >= LOCAL_PATH_MAX_POINTS) {
        break;
      }
      missionPoints[missionPointCount].x = point["x_cm"] | 0.0f;
      missionPoints[missionPointCount].y = point["y_cm"] | 0.0f;
      missionPointCount++;
    }

    if (missionPointCount == 0) {
      sendEvent("ERROR", "MISSION", "Path command did not include any points");
      return;
    }

    executeMission();
    return;
  }

  sendEvent("WARN", "SERIAL", String("Unknown command: ") + cmd);
}

void processSerialLine(String line, bool duringMotion) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  // Path missions can carry many points. Keep this large parse buffer on the
  // heap instead of the loopTask stack, because a path command immediately
  // executes the mission before this function returns.
  DynamicJsonDocument doc(4096);
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    sendEvent("ERROR", "SERIAL", String("Invalid JSON: ") + err.c_str());
    return;
  }

  handleCommand(doc, duringMotion);
}

void pollSerial(bool duringMotion) {
  while (Serial.available() > 0) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      processSerialLine(serialBuffer, duringMotion);
      serialBuffer = "";
    } else if (c != '\r') {
      serialBuffer += c;
      if (serialBuffer.length() > 3500) {
        serialBuffer = "";
        sendEvent("ERROR", "SERIAL", "Input line too long; buffer cleared");
      }
    }
  }
}

void calibrateMetalDetector() {
  sendEvent("INFO", "METAL", String("Calibrating NE555 baseline. Active-low metal detection: GPIO35 ADC ") + METAL_ADC_DETECT_MIN + "-" + METAL_ADC_DETECT_MAX + " means metal; no metal is usually near 4095.");
  unsigned long start = millis();
  long sum = 0;
  long count = 0;
  while (millis() - start < METAL_CALIBRATION_MS) {
    sum += analogRead(METAL_ADC_PIN);
    count++;
    delay(10);
  }
  metalBaselineAdc = count > 0 ? sum / count : analogRead(METAL_ADC_PIN);
  metalAdc = metalBaselineAdc;
  metalAdcMin = metalAdc;
  metalAdcMax = metalAdc;
  metalAdcDrop = 0;
  metalDetected = false;
  previousMetalDetected = false;
  sendEvent("INFO", "METAL", String("Calibration complete. baseline_adc=") + metalBaselineAdc + "; detect_range=" + METAL_ADC_DETECT_MIN + "-" + METAL_ADC_DETECT_MAX);
}

void setupWifiAlerts() {
#if WIFI_ALERT_ENABLED
  wifiAlertSsid = WIFI_ALERT_SSID;
  wifiAlertHost = WIFI_ALERT_HOST;
  wifiAlertPort = WIFI_ALERT_PORT;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_ALERT_SSID, WIFI_ALERT_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 6000UL) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiAlertReady = true;
    sendEvent("INFO", "WIFI", String("WiFi UDP alerts ready. ip=") + WiFi.localIP().toString());
  } else {
    wifiAlertReady = false;
    sendEvent("WARN", "WIFI", "WiFi alert connection failed; continuing with serial alerts only");
  }
#else
  wifiAlertReady = false;
  sendEvent("INFO", "WIFI", "WiFi UDP alerts waiting for dashboard wifi_config command");
#endif
}

void setupCmdUdp() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (cmdUdp.begin(WIFI_CMD_PORT)) {
    cmdUdpReady = true;
    sendEvent("INFO", "WIFI", String("UDP command listener ready on port ") + WIFI_CMD_PORT);
  } else {
    sendEvent("ERROR", "WIFI", "Failed to start UDP command listener");
  }
}

void pollCmdUdp(bool duringMotion) {
  if (!cmdUdpReady) return;
  int packetSize = cmdUdp.parsePacket();
  if (packetSize <= 0) return;

  char buf[3500];
  int len = cmdUdp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = '\0';

  // Store sender so we can reply if needed
  // cmdUdp.remoteIP(), cmdUdp.remotePort()

  String line = String(buf);
  line.trim();
  if (line.length() > 0) {
    processSerialLine(line, duringMotion);  // reuse exact same JSON handler
  }
}

void setup() {
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_IN3_PIN, OUTPUT);
  pinMode(MOTOR_IN4_PIN, OUTPUT);
  stopMotors();

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  dht.begin();

  analogReadResolution(12);
  analogSetPinAttenuation(METAL_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(MQ2_ADC_PIN, ADC_11db);

  Serial.begin(SERIAL_BAUD);
  delay(500);

  sendEvent("INFO", "BOOT", "Mine Detection Rover firmware started");
  sendEvent("INFO", "MOTOR", String("L298N pins IN1=") + MOTOR_IN1_PIN + " IN2=" + MOTOR_IN2_PIN + " IN3=" + MOTOR_IN3_PIN + " IN4=" + MOTOR_IN4_PIN);
  sendEvent("INFO", "DHT11", String("DHT11 data pin GPIO") + DHT11_DATA_PIN);

  calibrateMetalDetector();
  setupWifiAlerts();
  sendTelemetry(true);
}

void loop() {
  pollSerial(false);
  pollCmdUdp(false); 
  sampleMetalDetector(false);
  sampleDht11();

  if (robotMode == MODE_MANUAL && manualStopAtMs > 0 && millis() >= manualStopAtMs) {
    stopMotors();
    manualStopAtMs = 0;
    robotMode = MODE_IDLE;
    sendEvent("INFO", "MOTOR", "Timed manual drive complete");
  }

  unsigned long now = millis();
  if (now - lastTelemetryAtMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryAtMs = now;
    bool detailedTelemetry = now - lastDetailedTelemetryAtMs >= TELEMETRY_DETAIL_INTERVAL_MS;
    if (detailedTelemetry) {
      lastDetailedTelemetryAtMs = now;
    }
    sendTelemetry(detailedTelemetry);
  }

  delay(5);
}