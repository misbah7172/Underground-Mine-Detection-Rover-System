# PHASE 1 — Basic Rover Movement

1. Goal of the phase
- Provide reliable, non-blocking differential-drive motor control for the ESP32 using BTS7960 drivers; include emergency stop and serial control for bench testing.

2. Wiring diagram explanation
- Connect each BTS7960 motor driver module PWMA and PWMB inputs to the ESP32 PWM-capable pins defined in `config.h`. Power the motors from the 12V battery; provide common ground to the ESP32. E-STOP wired to a GPIO with pull-up; active LOW.

3. Pin configuration table
| Signal | Purpose | ESP32 Pin |
|---|---:|---:|
| MOTOR_L_PWM_A | Left motor PWM A (forward) | 25 |
| MOTOR_L_PWM_B | Left motor PWM B (reverse) | 26 |
| MOTOR_R_PWM_A | Right motor PWM A (forward) | 27 |
| MOTOR_R_PWM_B | Right motor PWM B (reverse) | 14 |
| E_STOP | Emergency stop (active LOW) | 12 |

4. Circuit protection advice
- Use an automotive-grade slow-blow fuse between battery and motor driver (5–10 A depending on motors). Add bulk electrolytic capacitor (220µF–1000µF) near motor driver power input and ceramic 0.1µF caps across motor terminals to reduce EMI. Ensure common ground and add TVS diodes for transient suppression. Add flyback diodes on companion circuits where needed (BTS7960 includes protection but add system-level protection).

5. Folder structure
- include/ (headers: motor.h, drive.h, config.h)
- src/ (motor.cpp, drive.cpp)
- phase1/ (phase1_main.cpp example)
- docs/ (PHASE1.md and wiring/parts docs)

6. Required libraries
- Arduino (built-in)
- No external libraries required for Phase 1

7. Complete ESP32 code
- See `phase1/phase1_main.cpp`, `include/motor.h`, `src/motor.cpp`, `include/drive.h`, `src/drive.cpp`

8. Explanation of code
- `Motor` class wraps two LEDC PWM channels to control a BTS7960 bridge. A positive speed drives A PWM, negative drives B PWM. `Drive` composes two `Motor` instances and exposes `setSpeed(left,right)` with a command timeout. `phase1_main.cpp` provides a serial command interface ('w','a','s','d','x') and E-STOP safety.

9. Testing procedure
- Bench test without wheels: connect motors loose, power motor supply, send serial commands at 115200 baud: `w` forward, `s` reverse, `a` left, `d` right, `x` stop. Verify E-STOP by pulling pin LOW.

10. Common failure points
- No common ground between battery and ESP32; incorrect PWM channel assignments; insufficient power/fuse ratings; wiring motor power to ESP32 5V pin causing damage.

11. Debugging guide
- Check Serial logs for status messages. Verify E-STOP pin value with multimeter. Use an oscilloscope to inspect PWM signals on the BTS7960 inputs. Check motor driver fault LED and battery voltage.

12. Suggested improvements
- Implement closed-loop current sensing, add soft-start ramping, and safe-state watchdog timer. Add a motor temperature sensor, and implement speed ramping profiles in software.
