# PHASE 3 — Obstacle Avoidance

1. Goal of the phase
- Implement reliable, non-blocking obstacle detection and avoidance using two ultrasonic sensors with minimal oscillation and recovery behavior.

2. Wiring diagram explanation
- Place two HC-SR04 sensors at the front-left and front-right. Each sensor requires TRIG and ECHO pins. Ensure separate pins for left and right sensors.

3. Pin configuration table
| Signal | Purpose | ESP32 Pin |
|---|---:|---:|
| US_LEFT_TRIG | Left ultrasonic trigger | see wiring |
| US_LEFT_ECHO | Left ultrasonic echo | see wiring |
| US_RIGHT_TRIG | Right ultrasonic trigger | see wiring |
| US_RIGHT_ECHO | Right ultrasonic echo | see wiring |

4. Circuit protection advice
- Ultrasonic sensors are low power; ensure wiring away from motor power and use decoupling capacitors on 5V/3.3V lines.

5. Folder structure
- `include/obstacle_avoid.h`, `src/obstacle_avoid.cpp`, `phase3/phase3_main.cpp`, `docs/PHASE3.md`

6. Required libraries
- None beyond existing `ultrasonic` driver

7. Complete ESP32 code
- See obstacle avoider files.

8. Explanation of code
- `ObstacleAvoider` polls both sensors periodically (non-blocking) and returns simple avoidance commands. It avoids tight oscillation by using a check interval and simple prioritized maneuvers.

9. Testing procedure
- Bench test with motors unloaded; place obstacles at various distances and verify correct avoidance maneuvers. Log distances over serial.

10. Common failure points
- Using same pins for both sensors; echo interference between sensors; oscillation due to immediate opposite commands.

11. Debugging guide
- Stagger sensor reads, add mutex/timer to avoid crosstalk; print distances and decisions to serial for tuning.

12. Suggested improvements
- Add stateful avoidance behaviors, use IMU for orientation stability, and add path re-planning instead of reactive turning.
