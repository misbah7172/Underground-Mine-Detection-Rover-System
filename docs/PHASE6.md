# PHASE 6 — Waypoint Autonomous Movement

1. Goal of the phase
- Implement waypoint navigation: accept a target coordinate and drive the rover toward it, stopping when within a radius.

2. Wiring diagram explanation
- Uses GPS for localization and existing motor/drive system for motion; no additional wiring beyond Phase 1 and Phase 5.

3. Pin configuration table
- See `docs/PHASE1.md` (motors) and `docs/PHASE5.md` (GPS)

4. Circuit protection advice
- Ensure safe-stop logic is active while performing navigation tests. Use tethered tests before untethered operation.

5. Folder structure
- `include/navigator.h`, `src/navigator.cpp`, `phase6/phase6_main.cpp`, `docs/PHASE6.md`

6. Required libraries
- `TinyGPSPlus` for GPS parsing

7. Complete ESP32 code
- See navigator files and `phase6_main.cpp`. The current implementation is a placeholder that computes distance and bearing; integrate IMU/compass for heading control and PID steering.

8. Explanation of code
- `WaypointNavigator` computes haversine distance and bearing; currently commands forward motion and is intended to be completed by integrating a heading sensor and closed-loop steering.

9. Testing procedure
- Set a nearby waypoint and run on a short field. Monitor serial for distance updates and verify rover approaches the goal. Start at low speeds.

10. Common failure points
- Relying only on GPS course yields poor heading control at low speeds; GPS noise can cause oscillation; compass interference from motors.

11. Debugging guide
- Log current GPS coordinates and computed bearing/distance. Use stationary tests to validate bearing calculation. Add compass/IMU to close the loop.

12. Suggested improvements
- Add magnetometer for heading, implement PID for steering, include waypoint queue and recovery behaviors.
