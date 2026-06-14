# PHASE 1 — Wiring and Elements List

## Elements list
- ESP32 DevKit V1
- 2x BTS7960 motor driver modules
- 2x DC geared motors
- 12V Li-ion battery pack
- E-STOP pushbutton (NC recommended)
- Fuse holder and inline fuse
- Buck converter modules for logic rails
- Wiring, connectors, terminal blocks
- Capacitors: 220µF electrolytic, 0.1µF ceramic

## Wiring summary
- Motor driver power inputs -> 12V battery (fused)
- Motor driver GND -> ESP32 GND
- Motor driver RPWM/LPWM -> ESP32 PWM pins (see `config.h`)
- E-STOP -> ESP32 `E_STOP_PIN` configured with pull-up
- All grounds must be common
- Place decoupling capacitors near motor driver power inputs

## Build process
1. Assemble the motor power path first.
2. Wire the E-STOP before enabling motor tests.
3. Verify motor direction with low-speed test commands.
4. Confirm the rover stops on power or command timeout.

## Safety checklist
- Fuse between battery and drivers
- Verify motor current draw does not exceed driver or fuse ratings
- Use proper gauge wires for motor current
- Test emergency stop before every field run
