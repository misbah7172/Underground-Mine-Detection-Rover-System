# NE555-Based Metal Detector System

## Overview

The metal detector system uses a **NE555 astable oscillator** connected to a search coil. The NE555 produces a square wave output whose **frequency changes in the presence of metal**. The ESP32 measures this frequency change using interrupt-based pulse counting and determines metal presence based on deviation from a baseline.

## How It Works

1. **Baseline Calibration (Startup)**
   - At startup, the rover takes frequency readings for ~3 seconds with **no metal present**
   - This establishes a baseline frequency (e.g., ~65 kHz for typical NE555 circuits)
   - Baseline is stored in RAM (survives mission duration)

2. **Real-Time Frequency Measurement**
   - The ESP32 counts rising edges from the NE555 output every 100 ms
   - Frequency is calculated as: `Freq (Hz) = (pulse_count × 1000) / 100 ms`
   - Last 10 frequency samples are kept in a rolling window

3. **Noise Filtering & Detection**
   - A moving average is computed over the 10 most recent samples
   - Metal is detected when frequency deviation exceeds **±15%** (configurable)
   - Hysteresis (300 ms delay) prevents false positives from single transients

4. **Alert & Response**
   - When metal is **first detected**: rover stops motors immediately
   - GPS coordinates + detection confidence are sent over LoRa (reliable queue)
   - Event is logged to SD card and emitted to Serial
   - Dashboard shows real-time frequency, deviation %, and confidence level

## Hardware Wiring

### NE555 Oscillator + Coil

The NE555 module typically has **4 exposed pins** on a breakout board:

| Pin | Function | Connection |
|-----|----------|-----------|
| GND | Ground | ESP32 GND |
| VCC | Power (usually 5V or 3.3V) | Power supply |
| Coil1 | Search coil terminal 1 | NE555 timing network (see oscillator circuit) |
| Coil2 | Search coil terminal 2 | NE555 timing network (see oscillator circuit) |

The coil and NE555 form an LC tank circuit. The NE555 oscillates at a frequency determined by:
- Capacitance of the coil parasitic capacitance + tuning cap
- Proximity of metal to the coil (lower frequency when metal is near)

### ESP32 Connection

| ESP32 GPIO | Function | Note |
|-----------|----------|------|
| GPIO 39 | NE555 OUT (square wave) | Rising-edge interrupt pin, input-only GPIO |
| GND | Common ground | Shared reference |

**GPIO 39** matches the current firmware in `include/config.h`. It is input-only, which is fine for a sensor output.

### Full Schematic

```
NE555 Output (pin 3)  ---> [level shift / divider] ---> ESP32 GPIO 39
```

*If your NE555 board outputs 5V logic, do not connect it directly to the ESP32. Use a divider or level shifter so the ESP32 sees 3.3V max.*

## Configuration

Edit `include/config.h`:

```cpp
// NE555 Metal Detector - Frequency-based system
#define NE555_OUTPUT_PIN 39                 // GPIO pin connected to NE555 OUT
#define NE555_CALIBRATION_DURATION_MS 3000  // 3 second baseline capture
#define NE555_SAMPLE_WINDOW_MS 100          // 100 ms per frequency sample
#define NE555_DETECTION_THRESHOLD_PCT 15    // ±15% deviation triggers detection
#define NE555_SAMPLE_HISTORY_SIZE 10        // Rolling avg over 10 samples
```

## Firmware API

### Class: `MetalDetectorNE555`

Located in `include/metal_detector_ne555.h` and `src/metal_detector_ne555.cpp`

```cpp
MetalDetectorNE555 metalDetector;

// Initialize ISR and GPIO
metalDetector.begin();

// Calibrate baseline (blocking, ~3 sec). Call during setup or calibration phase.
bool ok = metalDetector.calibrate();

// Update frequency measurements (non-blocking). Call from main loop.
metalDetector.update();

// Check if metal is currently detected
bool detected = metalDetector.isMetalDetected();

// Get detection confidence (0-100%)
int conf = metalDetector.getConfidence();

// Get current frequency in Hz
float freq = metalDetector.getCurrentFrequency();

// Get baseline frequency in Hz
float baseline = metalDetector.getBaselineFrequency();

// Get frequency deviation from baseline (%)
float devPct = metalDetector.getFrequencyDeviation();
```

### Main Loop Integration

```cpp
void setup() {
    // ... initialize other systems ...
    metalDetector.begin();
    sendEvent("INFO", "METAL", "Starting calibration...");
    metalDetector.calibrate();
    sendEvent("INFO", "METAL", "Metal detector ready");
}

void loop() {
    // ... read sensors ...
    metalDetector.update();
    
    // Handle metal detection
    if (metalDetector.isMetalDetected()) {
        drive.stop(); // STOP MOTORS
        // Send LoRa alert with GPS
        if (lastGps.valid) {
            String alert = String("{\"type\":\"alert\",\"lat\":") + String(lastGps.latitude, 6)
                         + String(",\"lon\":") + String(lastGps.longitude, 6)
                         + String(",\"confidence\":") + String(metalDetector.getConfidence())
                         + String("}");
            lora.sendReliable(alert.c_str(), 4, 1500);
        }
    }
    
    // ... rest of control logic ...
}
```

## Telemetry & Dashboard

The rover emits metal detector data in real-time telemetry:

```json
{
  "type": "telemetry",
  "metal_detected": false,
  "metal_freq_hz": 65241.5,
  "metal_freq_dev_pct": 2.3,
  "metal_confidence": 0
}
```

**Dashboard Display** (Control tab):
- **Metal Freq**: Current oscillation frequency in Hz
- **Freq Deviation**: Percentage shift from baseline (🟢 < 5%, 🟡 5-15%, 🔴 > 15%)
- **Metal Confidence**: Detection confidence (0-100%)
- **Metal Status**: 🚨 DETECTED or ✓ Clear

## Calibration Procedure

1. **Ensure coil is clear of metal**
   - Place rover on bench away from metal objects
   - Keep away from ferrous tools, steel parts, etc.

2. **Wire the module**
   - `VCC` on the NE555 board to a stable 5V supply if the module expects 5V
   - `GND` to ESP32 GND and battery supply ground
   - `OUT` to ESP32 GPIO 39 through a 3.3V-safe level shift

3. **Power on and start mission**
   - Firmware automatically runs 3-second calibration during `setup()`
   - Watch Serial monitor for confirmation: "Metal detector calibrated. Baseline: 65241.5 Hz"
   - Dashboard shows baseline frequency when connected

4. **Test detection**
   - After calibration, bring a metal object near the coil
   - Frequency should drop (typically 20-50% for ferrous metals)
   - Dashboard should show red alert and "DETECTED" status
   - Rover will stop motors and send LoRa alert

## Tuning & Troubleshooting

### Baseline Frequency Too Low
- **Symptom**: Baseline shows 1000-5000 Hz (unusually low)
- **Cause**: Poor oscillator circuit, timing capacitor too large, coil damaged
- **Fix**: Check NE555 timing capacitors and connections

### False Positives (frequent random detections)
- **Symptom**: Metal alert fires without nearby metal
- **Cause**: Motor/radio noise coupling into coil or NE555
- **Fix**:
  - Shield coil with mu-metal or aluminum foil
  - Add 0.1 µF capacitor near NE555 VCC/GND
  - Move NE555 board away from motors/radio
  - Increase `NE555_DETECTION_THRESHOLD_PCT` to 20-25%

### No Detection
- **Symptom**: Metal nearby but no alert
- **Cause**: Threshold too high, weak coil signal, or calibration issue
- **Fix**:
  - Re-run calibration on bench away from all metal
  - Decrease threshold to 10% (more sensitive, higher false positive risk)
  - Check GPIO 27 receives clean square wave (oscilloscope test)

### Frequency Drift Over Time
- **Symptom**: Baseline drifts, causing sensitivity changes
- **Cause**: Temperature drift in oscillator, aging of components
- **Fix**: Re-calibrate rover at start of each mission session

## Performance Specs

| Metric | Value |
|--------|-------|
| Baseline Frequency | ~50–100 kHz (depends on circuit) |
| Detection Range | 5–20 cm (depends on coil size & metal type) |
| Sample Rate | 100 ms (10 samples/sec) |
| Response Time | ~300 ms (after 3 samples + hysteresis) |
| Sensitivity | Configurable ±10–25% frequency deviation |
| Confidence Range | 0–100% |

## Advanced Usage

### Recalibrate During Mission

Send command via Serial/LoRa:
```json
{"cmd":"metal_calibrate"}
```

*(Implement in firmware if needed)*

### Adjust Threshold Dynamically

Edit at runtime (firmware enhancement):
```cpp
// Command parser addition
if (line.indexOf("\"cmd\":\"metal_threshold\"") >= 0) {
    int newThreshold = 20;
    extractIntField(line, "threshold", newThreshold);
    // apply to detection logic
}
```

### Log Frequency Data for Analysis

Frequency is already logged to `telemetry.log` on SD card. Extract and plot in Python:

```python
import json
import matplotlib.pyplot as plt

freqs = []
with open('telemetry.log') as f:
    for line in f:
        data = json.loads(line)
        if 'metal_freq_hz' in data:
            freqs.append(data['metal_freq_hz'])

plt.plot(freqs)
plt.xlabel('Sample #')
plt.ylabel('Frequency (Hz)')
plt.title('Metal Detector Frequency Over Time')
plt.show()
```

## Integration Notes

- **Motor Interference**: Motors can generate noise that couples into the coil. Shield with ferrite, mu-metal, or run metal detector only during motion planning, not active drive.
- **Concurrent Telemetry**: Metal detector telemetry is non-blocking and updates every 100 ms. No delay() calls in the frequency measurement path.
- **LoRa Alert**: Uses the durable reliable queue; alerts will retry up to 4 times with exponential backoff if ACK is not received.
- **Power Consumption**: ISR is lightweight (single increment). Update loop is ~1-2 ms per 100 ms window.

## Next Steps

- Field test with real coil and target metals (iron, steel, aluminum)
- Measure baseline and tuning for your specific coil
- Adjust threshold and sample history size for environment
- Integrate with mission planning (avoid metal-rich areas, spiral search priority)
