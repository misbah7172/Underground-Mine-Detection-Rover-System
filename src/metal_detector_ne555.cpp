#include "metal_detector_ne555.h"
#include "config.h"

// Global pointer for ISR access
static MetalDetectorNE555* gMetalDetector = nullptr;

// Lightweight ISR: just count pulses
void IRAM_ATTR metalDetectorISR() {
    if (gMetalDetector) {
        gMetalDetector->_incrementPulse();
    }
}

MetalDetectorNE555::MetalDetectorNE555()
    : initialized(false), calibrated(false), pulseCount(0), lastSampleMs(0),
      baselineFreq(0.0f), currentFreq(0.0f), freqDeviation(0.0f),
      historyIndex(0), sampleCount(0), detectionActive(false),
      detectionStartMs(0), detectionHysteresisMs(300) {
    memset(freqHistory, 0, sizeof(freqHistory));
}

void MetalDetectorNE555::begin() {
    if (initialized) return;
    gMetalDetector = this;
    pinMode(NE555_OUTPUT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(NE555_OUTPUT_PIN), metalDetectorISR, RISING);
    lastSampleMs = millis();
    initialized = true;
}

bool MetalDetectorNE555::calibrate() {
    if (!initialized) return false;
    
    // Collect baseline frequency over NE555_CALIBRATION_DURATION_MS
    unsigned long calStart = millis();
    unsigned long calEnd = calStart + NE555_CALIBRATION_DURATION_MS;
    
    float freqSum = 0.0f;
    int freqCount = 0;
    
    resetPulseCount();
    unsigned long lastCheckMs = calStart;
    
    while (millis() < calEnd) {
        unsigned long now = millis();
        
        // Sample every 100 ms during calibration
        if (now - lastCheckMs >= NE555_SAMPLE_WINDOW_MS) {
            unsigned long elapsed = now - lastCheckMs;
            float freq = (pulseCount * 1000.0f) / elapsed;
            freqSum += freq;
            freqCount++;
            resetPulseCount();
            lastCheckMs = now;
        }
        
        delay(10); // small delay to avoid busy-wait
    }
    
    if (freqCount > 0) {
        baselineFreq = freqSum / freqCount;
        currentFreq = baselineFreq;
        calibrated = true;
        return true;
    }
    
    return false;
}

void MetalDetectorNE555::update() {
    if (!initialized || !calibrated) return;
    
    unsigned long now = millis();
    
    // Sample frequency every NE555_SAMPLE_WINDOW_MS
    if (now - lastSampleMs >= NE555_SAMPLE_WINDOW_MS) {
        unsigned long elapsed = now - lastSampleMs;
        
        // Calculate frequency (Hz)
        currentFreq = (pulseCount * 1000.0f) / elapsed;
        resetPulseCount();
        lastSampleMs = now;
        
        // Add to rolling history
        freqHistory[historyIndex] = currentFreq;
        historyIndex = (historyIndex + 1) % NE555_SAMPLE_HISTORY_SIZE;
        if (sampleCount < NE555_SAMPLE_HISTORY_SIZE) {
            sampleCount++;
        }
        
        // Compute average and deviation
        float avgFreq = getAverageFrequency();
        if (baselineFreq > 0.0f) {
            freqDeviation = ((avgFreq - baselineFreq) / baselineFreq) * 100.0f;
        }
        
        // Metal detection logic with hysteresis
        bool shouldDetect = (fabs(freqDeviation) > NE555_DETECTION_THRESHOLD_PCT) && (sampleCount >= 3);
        
        if (shouldDetect && !detectionActive) {
            // Rising edge: metal detected
            detectionActive = true;
            detectionStartMs = now;
        } else if (!shouldDetect && detectionActive && (now - detectionStartMs) > detectionHysteresisMs) {
            // Falling edge: metal no longer detected (after hysteresis)
            detectionActive = false;
        }
    }
}

bool MetalDetectorNE555::isMetalDetected() {
    return detectionActive;
}

int MetalDetectorNE555::getConfidence() {
    if (!calibrated || sampleCount == 0) return 0;
    
    float absDeviation = fabs(freqDeviation);
    
    // Confidence increases as deviation increases beyond threshold
    int conf = (int)((absDeviation - NE555_DETECTION_THRESHOLD_PCT) / (50.0f - NE555_DETECTION_THRESHOLD_PCT) * 100.0f);
    if (conf < 0) conf = 0;
    if (conf > 100) conf = 100;
    
    // Reduce confidence during early samples (less reliable)
    if (sampleCount < 5) {
        conf = (conf * sampleCount) / 5;
    }
    
    return conf;
}

float MetalDetectorNE555::getCurrentFrequency() {
    return currentFreq;
}

float MetalDetectorNE555::getBaselineFrequency() {
    return baselineFreq;
}

float MetalDetectorNE555::getFrequencyDeviation() {
    return freqDeviation;
}

unsigned long MetalDetectorNE555::getPulseCount() {
    return pulseCount;
}

float MetalDetectorNE555::getAverageFrequency() {
    if (sampleCount == 0) return 0.0f;
    
    float sum = 0.0f;
    for (int i = 0; i < sampleCount; ++i) {
        sum += freqHistory[i];
    }
    return sum / sampleCount;
}

void MetalDetectorNE555::resetPulseCount() {
    pulseCount = 0;
}

// Make ISR callback accessible (friend or public for ISR)
void MetalDetectorNE555::_incrementPulse() {
    pulseCount++;
}
