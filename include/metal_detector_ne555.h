#ifndef METAL_DETECTOR_NE555_H
#define METAL_DETECTOR_NE555_H

#include <Arduino.h>
#include "config.h"

class MetalDetectorNE555 {
public:
    MetalDetectorNE555();
    
    // Initialize ISR and variables
    void begin();
    
    // Calibration: measure baseline frequency when no metal present (blocking, ~3 sec)
    bool calibrate();
    
    // Non-blocking frequency update; call from main loop
    void update();
    
    // Returns true if metal detected based on frequency deviation
    bool isMetalDetected();
    
    // Detection confidence (0-100%)
    int getConfidence();
    
    // Current frequency in Hz
    float getCurrentFrequency();
    
    // Baseline frequency in Hz
    float getBaselineFrequency();
    
    // Frequency deviation from baseline (%)
    float getFrequencyDeviation();
    
    // Get last raw pulse count
    unsigned long getPulseCount();
    
    // ISR callback (increment pulse count) - called from ISR
    void _incrementPulse();
    
private:
    bool initialized;
    bool calibrated;
    
    // ISR state
    volatile unsigned long pulseCount;
    unsigned long lastSampleMs;
    
    // Frequency tracking
    float baselineFreq;
    float currentFreq;
    float freqDeviation;
    
    // Rolling sample history for noise filtering
    float freqHistory[NE555_SAMPLE_HISTORY_SIZE];
    int historyIndex;
    int sampleCount;
    
    // Detection state with hysteresis
    bool detectionActive;
    unsigned long detectionStartMs;
    unsigned long detectionHysteresisMs;
    
    // Calculate moving average of frequency samples
    float getAverageFrequency();
    
    // Reset pulse counter
    void resetPulseCount();
};

#endif
