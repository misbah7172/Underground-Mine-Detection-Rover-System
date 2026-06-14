# PHASE 7 — Metal Detector Integration

1. Goal of the phase
- Integrate a mini induction metal detector to detect metal objects and report events with timestamp and GPS when available.

2. Wiring diagram explanation
- Metal detector digital output -> `METAL_DETECTOR_PIN` (GPIO34). If analog output is available, connect to ADC-capable pin and read sensitivity values.

3. Pin configuration table
| Signal | Purpose | ESP32 Pin |
|---|---:|---:|
| Metal Detector | Digital/analog detection input | 34 |

4. Circuit protection advice
- Isolate detector electronics from motor power; provide decoupling and shielding to reduce false triggers from EMI.

5. Folder structure
- `include/metal_detector.h`, `src/metal_detector.cpp`, `phase7/phase7_main.cpp`, `docs/PHASE7.md`

6. Required libraries
- None (uses standard Arduino functions)

7. Complete ESP32 code
- See metal detector files and `phase7_main.cpp`.

8. Explanation of code
- `MetalDetector` reads a digital pin; `phase7_main` reports digital triggers and analog raw values periodically.

9. Testing procedure
- Bring metal near the detector coil and observe serial prints. Test at different motor states to evaluate interference.

10. Common failure points
- False positives due to motor EMI; poor grounding; coil sensitivity settings too high.

11. Debugging guide
- Test detector with motors powered off, then on. Add shielding and place detector away from motors. Implement debounce/filtering in software.

12. Suggested improvements
- Add software debounce, envelope detection, and event logging with GPS coordinates. Use a separate ground plane or shielded cable for detector.
