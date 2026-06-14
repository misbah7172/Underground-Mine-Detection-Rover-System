# PHASE 11 — Laptop Control Station Software

## Goal
Create a Streamlit dashboard for manual control, telemetry viewing, and map-based mission upload from a laptop.

## Elements List
- Laptop running Python
- ESP32 rover firmware with telemetry support
- LoRa SX1278 radio link or USB-to-serial telemetry bridge
- Map/telemetry display area
- OpenStreetMap / Leaflet map widget
- Search radius control
- Manual drive controls
- Mission upload controls
- Log viewer for battery, GPS, metal, and smoke events

## Wiring And Connection Plan
- Laptop -> USB serial or LoRa gateway interface
- Rover telemetry -> ESP32 LoRa module or serial debug port
- Manual commands -> laptop UI to rover command channel
- Telemetry return path -> rover to dashboard via the same link

## Process
1. Start the rover firmware and confirm telemetry packets are visible.
2. Open the dashboard on the laptop.
3. Connect to the rover over the selected link.
4. Click a target location on the map.
5. Adjust the search radius.
6. Display live sensor values, battery state, GPS position, and rover trail.
7. Send the mission packet wirelessly.
8. Add manual control buttons for forward, reverse, left, right, and stop.

## Dashboard Features
- Live telemetry panel with battery, GPS, heading, obstacle, and sensor readings
- Interactive Leaflet map with click-to-select target and radius circle
- Rover trail/path overlay
- Manual rover control buttons
- Mission queue and status view
- Event log table
- Emergency stop button
- Connection status indicator
- Auto-refresh telemetry polling
- JSON command protocol for rover control

## Mission Packet
The dashboard sends a single-target packet in this form:

```json
{"mission":"start","lat":23.8103,"lon":90.4125,"radius":20}
```

The rover firmware must parse this packet and hand it to the autonomous navigation stack to move toward the selected location.

## Notes
- The dashboard is implemented in `dashboard/app.py`.
- The dashboard expects rover telemetry as JSON or key-value lines over serial.
- Manual commands are sent as JSON lines such as `{"cmd":"drive",...}`.
- The map UI uses Leaflet through `streamlit-folium`.