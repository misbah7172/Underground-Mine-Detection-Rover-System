# PHASE 2 — Power Management and Stability

1. Goal of the phase
- Implement reliable battery voltage monitoring and safe behaviors when battery is low. Provide tools for ADC-based measurement and software alerts.

2. Wiring diagram explanation
- Use a voltage divider (R1=100k, R2=33k) between battery positive and ground; ADC reads divided voltage; ensure ADC pin shares common ground with battery.

3. Pin configuration table
| Signal | Purpose | ESP32 Pin |
|---|---:|---:|
| BATTERY_ADC | Battery sensing ADC input | 35 |

4. Circuit protection advice
- Use fuse between battery and system. Use proper divider resistor power ratings. Add input transient suppression (TVS) and input filtering (RC) before ADC if motor noise present.

5. Folder structure
- `include/power.h`, `src/power.cpp`, `phase2/phase2_main.cpp`, `docs/PHASE2.md`

6. Required libraries
- None (uses Arduino core functions)

7. Complete ESP32 code
- See `include/power.h` and `src/power.cpp` and `phase2/phase2_main.cpp`.

8. Explanation of code
- `PowerManager` reads ADC with 12-bit resolution and ADC_11db attenuation to allow higher input voltages; computes actual battery voltage using divider ratio; reports percentage and low battery threshold.

9. Testing procedure
- Power the vehicle and read serial output; verify voltage matches multimeter; test low-battery behavior by reducing battery voltage or simulating via bench supply.

10. Common failure points
- ADC reading inaccurate due to Vref drift; noisy readings from motors; wrong divider values; missing common ground.

11. Debugging guide
- Calibrate by measuring known voltages and adjust `BATTERY_VOLT_DIVIDER_RATIO` or use a measured Vref. Add averaging filter in software if noisy.

12. Suggested improvements
- Add smoothing, exponential filter, current sensing (shunt + INA219), and an interrupt-based battery watchdog to force safe stop.
