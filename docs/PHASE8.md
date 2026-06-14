# PHASE 8 — Smoke Sensor (MQ-2)

1. Goal
- Integrate MQ-2 sensor for smoke/gas detection and report events with GPS timestamps.

2. Wiring
- MQ-2 analog output -> `MQ2_PIN` (GPIO36). Provide stable 5V/3.3V supply and common ground.

3. Files
- `include/mq2.h`, `src/mq2.cpp`, `phase8/phase8_main.cpp`.

4. Testing
- Run `phase8_main` and verify raw fraction values; calibrate threshold empirically.
