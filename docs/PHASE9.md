# PHASE 9 — Spiral Search Algorithm

1. Goal
- Implement an expanding spiral search over a GPS area using generated waypoints and the navigator.

2. Approach
- Generate an Archimedean spiral in meter coordinates, convert to lat/lon offsets, and visit waypoints sequentially using waypoint navigator.

3. Files
- `include/spiral_search.h`, `src/spiral_search.cpp`, `phase9/phase9_main.cpp`.

4. Limitations
- GPS noise and heading errors can degrade spiral accuracy; use short step sizes and overlap to mitigate.
