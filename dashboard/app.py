from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Tuple
import sys

import folium
import pandas as pd
import streamlit as st
from streamlit_autorefresh import st_autorefresh
from streamlit_folium import st_folium

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from protocol import (  # noqa: E402
    TelemetrySnapshot,
    EventRecord,
    calibrate_magnetometer_command,
    calibrate_magnetometer_stop_command,
    drive_command,
    estop_command,
    mission_command,
    mission_start_packet,
    parse_line,
    parse_waypoints,
    ping_command,
    status_command,
    stop_command,
)
from serial_client import RoverSerialClient, available_ports  # noqa: E402


MAX_LOG_ROWS = 300
MAX_EVENT_ROWS = 200
MAX_HISTORY = 200


def init_state() -> None:
    st.session_state.setdefault("client", RoverSerialClient())
    st.session_state.setdefault("telemetry", TelemetrySnapshot(source="dashboard"))
    st.session_state.setdefault("telemetry_history", [])
    st.session_state.setdefault("raw_log", [])
    st.session_state.setdefault("events", [])
    st.session_state.setdefault("selected_port", "")
    st.session_state.setdefault("baudrate", 115200)
    st.session_state.setdefault("auto_refresh_s", 2)
    st.session_state.setdefault("connection_message", "Disconnected")
    st.session_state.setdefault("mission_text", "12.9716,77.5946,3\n12.9722,77.5951,3")
    st.session_state.setdefault("mission_name", "mission-1")
    st.session_state.setdefault("manual_speed", 140)
    st.session_state.setdefault("manual_turn", 60)
    st.session_state.setdefault("mag_cal_seconds", 15)
    st.session_state.setdefault("target_lat", None)
    st.session_state.setdefault("target_lon", None)
    st.session_state.setdefault("target_radius_cm", 200.0)
    st.session_state.setdefault("rover_trail", [])
    st.session_state.setdefault("map_zoom", 16)
    st.session_state.setdefault("mission_status", "Awaiting target selection")
    st.session_state.setdefault("alert_filter", "All")


def log_raw(line: str) -> None:
    st.session_state.raw_log.append(line)
    st.session_state.raw_log = st.session_state.raw_log[-MAX_LOG_ROWS:]


def add_event(event: EventRecord) -> None:
    st.session_state.events.append(event.as_dict())
    st.session_state.events = st.session_state.events[-MAX_EVENT_ROWS:]


def record_telemetry(snapshot: TelemetrySnapshot) -> None:
    st.session_state.telemetry = snapshot
    st.session_state.telemetry_history.append(snapshot.as_dict())
    st.session_state.telemetry_history = st.session_state.telemetry_history[-MAX_HISTORY:]
    if snapshot.gps_lat is not None and snapshot.gps_lon is not None:
        append_rover_trail(snapshot.gps_lat, snapshot.gps_lon)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    from math import asin, cos, radians, sin, sqrt

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2.0) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2.0) ** 2
    return 6371000.0 * 2.0 * asin(sqrt(a))


def append_rover_trail(latitude: float, longitude: float) -> None:
    trail: List[Tuple[float, float]] = st.session_state.rover_trail
    point = (float(latitude), float(longitude))
    if not trail:
        trail.append(point)
        return
    last_lat, last_lon = trail[-1]
    if haversine_m(last_lat, last_lon, point[0], point[1]) >= 0.75:
        trail.append(point)
        st.session_state.rover_trail = trail[-500:]


def set_target(latitude: float, longitude: float) -> None:
    st.session_state.target_lat = round(float(latitude), 6)
    st.session_state.target_lon = round(float(longitude), 6)
    st.session_state.mission_status = "Target updated from map"


def target_is_set() -> bool:
    return st.session_state.target_lat is not None and st.session_state.target_lon is not None


def get_map_center() -> Tuple[float, float]:
    telemetry: TelemetrySnapshot = st.session_state.telemetry
    if target_is_set():
        return float(st.session_state.target_lat), float(st.session_state.target_lon)
    if telemetry.gps_lat is not None and telemetry.gps_lon is not None:
        return float(telemetry.gps_lat), float(telemetry.gps_lon)
    if st.session_state.rover_trail:
        lat, lon = st.session_state.rover_trail[-1]
        return float(lat), float(lon)
    return 23.8103, 90.4125


def build_mission_map() -> folium.Map:
    center_lat, center_lon = get_map_center()
    zoom_start = int(st.session_state.map_zoom)
    fmap = folium.Map(location=[center_lat, center_lon], zoom_start=zoom_start, control_scale=True, tiles=None)
    folium.TileLayer("OpenStreetMap", name="OpenStreetMap", control=True).add_to(fmap)
    folium.TileLayer("CartoDB positron", name="Light", control=True).add_to(fmap)

    telemetry: TelemetrySnapshot = st.session_state.telemetry
    trail = st.session_state.rover_trail
    if len(trail) >= 2:
        folium.PolyLine(trail, color="#2563eb", weight=4, opacity=0.85, tooltip="Rover trail").add_to(fmap)

    if telemetry.gps_lat is not None and telemetry.gps_lon is not None:
        folium.Marker(
            [telemetry.gps_lat, telemetry.gps_lon],
            tooltip="Rover position",
            popup=f"Rover: {telemetry.gps_lat:.6f}, {telemetry.gps_lon:.6f}",
            icon=folium.Icon(color="blue", icon="car", prefix="fa"),
        ).add_to(fmap)

    if target_is_set():
        target_lat = float(st.session_state.target_lat)
        target_lon = float(st.session_state.target_lon)
        radius_cm = float(st.session_state.target_radius_cm)
        radius_m = radius_cm / 100.0
        folium.Marker(
            [target_lat, target_lon],
            tooltip="Mission target",
            popup=f"Target: {target_lat:.6f}, {target_lon:.6f}",
            icon=folium.Icon(color="green", icon="flag", prefix="fa"),
        ).add_to(fmap)
        folium.Circle(
            [target_lat, target_lon],
            radius=radius_m,
            color="#16a34a",
            weight=2,
            fill=True,
            fill_color="#16a34a",
            fill_opacity=0.15,
            tooltip=f"Search radius: {radius_cm:.0f} cm ({radius_m:.2f} m)",
        ).add_to(fmap)

    if target_is_set() and telemetry.gps_lat is not None and telemetry.gps_lon is not None:
        folium.PolyLine(
            [
                [telemetry.gps_lat, telemetry.gps_lon],
                [float(st.session_state.target_lat), float(st.session_state.target_lon)],
            ],
            color="#f59e0b",
            weight=2,
            opacity=0.75,
            dash_array="8",
            tooltip="Route to target",
        ).add_to(fmap)

    folium.LayerControl(collapsed=True).add_to(fmap)
    return fmap


def poll_rover() -> None:
    client: RoverSerialClient = st.session_state.client
    if not client.connected:
        return
    try:
        for line in client.read_lines():
            log_raw(line)
            parsed = parse_line(line, source="serial")
            if parsed["kind"] == "telemetry":
                record_telemetry(parsed["telemetry"])
            elif parsed["kind"] in {"event", "log"}:
                add_event(parsed["event"])
    except Exception as exc:  # noqa: BLE001
        st.session_state.connection_message = f"Connection lost: {exc}"
        client.disconnect()


def send_command(payload: str, note: str) -> None:
    client: RoverSerialClient = st.session_state.client
    if not client.connected:
        st.warning("Connect to the rover first.")
        return
    try:
        client.send_line(payload)
        st.session_state.connection_message = note
    except Exception as exc:  # noqa: BLE001
        st.error(f"Command failed: {exc}")


def render_metric_card(label: str, value: str, help_text: str = "") -> None:
    st.metric(label, value, help=help_text or None)


def fmt_float(value, suffix: str = "", digits: int = 2) -> str:
    if value is None:
        return "—"
    return f"{value:.{digits}f}{suffix}"


def control_button_row() -> None:
    speed = int(st.session_state.manual_speed)
    turn = int(st.session_state.manual_turn)
    cols = st.columns(3)
    with cols[0]:
        if st.button("↖ Forward Left", use_container_width=True):
            send_command(drive_command(speed - turn, speed), "Forward-left sent")
    with cols[1]:
        if st.button("↑ Forward", use_container_width=True):
            send_command(drive_command(speed, speed), "Forward sent")
    with cols[2]:
        if st.button("↗ Forward Right", use_container_width=True):
            send_command(drive_command(speed, speed - turn), "Forward-right sent")

    cols = st.columns(3)
    with cols[0]:
        if st.button("← Turn Left", use_container_width=True):
            send_command(drive_command(-turn, turn), "Turn-left sent")
    with cols[1]:
        if st.button("■ Stop", type="primary", use_container_width=True):
            send_command(stop_command(), "Stop sent")
    with cols[2]:
        if st.button("→ Turn Right", use_container_width=True):
            send_command(drive_command(turn, -turn), "Turn-right sent")

    cols = st.columns(3)
    with cols[0]:
        if st.button("↙ Back Left", use_container_width=True):
            send_command(drive_command(-speed + turn, -speed), "Back-left sent")
    with cols[1]:
        if st.button("↓ Backward", use_container_width=True):
            send_command(drive_command(-speed, -speed), "Backward sent")
    with cols[2]:
        if st.button("↘ Back Right", use_container_width=True):
            send_command(drive_command(-speed, -speed + turn), "Back-right sent")


def render_dashboard() -> None:
    telemetry: TelemetrySnapshot = st.session_state.telemetry

    st.subheader("Live Telemetry")
    metric_cols = st.columns(4)
    with metric_cols[0]:
        render_metric_card("Battery", f"{fmt_float(telemetry.battery_v, ' V')}  |  {fmt_float(telemetry.battery_pct, '%')}", "Power status")
    with metric_cols[1]:
        render_metric_card("Heading", f"{fmt_float(telemetry.heading_deg, '°')}", "Compass / AHRS")
    with metric_cols[2]:
        render_metric_card("Distance", f"{fmt_float(telemetry.distance_cm, ' cm')}", "Obstacle range")
    with metric_cols[3]:
        render_metric_card("Mode / State", f"{telemetry.mode or '—'} / {telemetry.state or '—'}", "Mission mode")

    metric_cols = st.columns(4)
    with metric_cols[0]:
        render_metric_card("Metal Freq", f"{fmt_float(telemetry.metal_freq_hz, ' Hz')}", "NE555 frequency")
    with metric_cols[1]:
        freq_dev = telemetry.metal_freq_dev_pct or 0
        dev_color = "🔴" if abs(freq_dev) > 15 else "🟡" if abs(freq_dev) > 5 else "🟢"
        render_metric_card("Freq Deviation", f"{dev_color} {fmt_float(freq_dev, '%')}", "Frequency shift from baseline")
    with metric_cols[2]:
        render_metric_card("Metal Confidence", f"{fmt_float(telemetry.metal_confidence, '%')}", "Detection confidence")
    with metric_cols[3]:
        metal_status = "🚨 DETECTED" if telemetry.metal_detected else "✓ Clear"
        render_metric_card("Metal Status", metal_status, "Metal detection status")

    metric_cols = st.columns(4)
    with metric_cols[0]:
        render_metric_card("Temperature", f"{fmt_float(telemetry.temperature_c, ' °C')}", "Ambient temperature")
    with metric_cols[1]:
        render_metric_card("Humidity", f"{fmt_float(telemetry.humidity_pct, '%')}", "Ambient humidity")
    with metric_cols[2]:
        render_metric_card("Gas / Smoke", f"{fmt_float(telemetry.gas_raw)}", "MQ-2 raw reading")
    with metric_cols[3]:
        render_metric_card("Metal", f"{fmt_float(telemetry.metal_raw)}", f"Detected: {telemetry.metal_detected}")

    st.divider()

    alert_rows = [
        event for event in reversed(st.session_state.events)
        if str(event.get("level", "INFO")).upper() in {"WARN", "WARNING", "ERROR", "CRITICAL"}
        or str(event.get("category", "")).upper() in {"SENSOR", "ALERT", "EVENT"}
    ][:3]
    if alert_rows:
        st.subheader("Sensor Alerts")
        for event in alert_rows:
            level = str(event.get("level", "INFO")).upper()
            category = str(event.get("category", "LOG")).upper()
            message = event.get("message", "")
            if level in {"ERROR", "CRITICAL"}:
                st.error(f"[{level}] {category}: {message}")
            elif level in {"WARN", "WARNING"}:
                st.warning(f"[{level}] {category}: {message}")
            else:
                st.info(f"[{level}] {category}: {message}")

    map_cols = st.columns([2, 1])
    with map_cols[0]:
        st.subheader("Position")
        if telemetry.gps_lat is not None and telemetry.gps_lon is not None:
            position_df = pd.DataFrame([{ "lat": telemetry.gps_lat, "lon": telemetry.gps_lon }])
            st.map(position_df, latitude="lat", longitude="lon", zoom=16)
        else:
            st.info("Waiting for GPS coordinates from the rover.")
    with map_cols[1]:
        st.subheader("Current Snapshot")
        rows = telemetry.to_display_rows()
        if rows:
            st.dataframe(pd.DataFrame(rows, columns=["Field", "Value"]), use_container_width=True, hide_index=True)
        else:
            st.write("No telemetry yet.")


def render_logs() -> None:
    st.subheader("Event Log")
    if st.session_state.events:
        st.dataframe(pd.DataFrame(st.session_state.events), use_container_width=True, hide_index=True)
    else:
        st.info("No events received yet.")

    st.subheader("Raw Serial Log")
    raw_text = "\n".join(st.session_state.raw_log[-100:])
    st.text_area("Recent lines", raw_text, height=220)


def render_mission() -> None:
    st.subheader("Mission Planner")
    st.caption("Click on the map to choose the target, then set the search radius and send the mission packet wirelessly.")

    left_col, right_col = st.columns([1.55, 1.0])
    with left_col:
        map_state = st_folium(
            build_mission_map(),
            height=620,
            use_container_width=True,
            key="mission_map",
        )
        clicked = map_state.get("last_clicked") if map_state else None
        if clicked and "lat" in clicked and "lng" in clicked:
            set_target(clicked["lat"], clicked["lng"])

    with right_col:
        target_lat = st.session_state.target_lat
        target_lon = st.session_state.target_lon
        rover_lat = st.session_state.telemetry.gps_lat
        rover_lon = st.session_state.telemetry.gps_lon

        radius_value = st.slider(
            "Search radius (cm)",
            min_value=10,
            max_value=2000,
            value=int(st.session_state.target_radius_cm),
            step=5,
        )
        st.session_state.target_radius_cm = float(radius_value)

        target_cols = st.columns(3)
        with target_cols[0]:
            st.metric("Latitude", "—" if target_lat is None else f"{target_lat:.6f}")
        with target_cols[1]:
            st.metric("Longitude", "—" if target_lon is None else f"{target_lon:.6f}")
        with target_cols[2]:
            st.metric("Radius", f"{st.session_state.target_radius_cm:.0f} cm")

        st.caption(f"Radius in meters: {st.session_state.target_radius_cm / 100.0:.2f} m")

        rover_cols = st.columns(2)
        with rover_cols[0]:
            st.metric("Rover Latitude", "—" if rover_lat is None else f"{rover_lat:.6f}")
        with rover_cols[1]:
            st.metric("Rover Longitude", "—" if rover_lon is None else f"{rover_lon:.6f}")

        if target_lat is not None and target_lon is not None:
            target_map = pd.DataFrame([{ "lat": target_lat, "lon": target_lon }])
            st.map(target_map, latitude="lat", longitude="lon", zoom=18)

        if rover_lat is not None and rover_lon is not None and target_lat is not None and target_lon is not None:
            distance = haversine_m(float(rover_lat), float(rover_lon), float(target_lat), float(target_lon))
            st.info(f"Estimated rover-to-target distance: {distance:.1f} m")

        mission_status = st.session_state.mission_status
        st.caption(f"Mission status: {mission_status}")

        action_cols = st.columns(2)
        with action_cols[0]:
            send_enabled = target_lat is not None and target_lon is not None
            if st.button("Send Mission", type="primary", use_container_width=True, disabled=not send_enabled):
                if send_enabled:
                    packet = mission_start_packet(float(target_lat), float(target_lon), float(st.session_state.target_radius_cm))
                    send_command(packet, "Mission packet sent")
                    st.session_state.mission_status = "Mission sent to rover base station"
                else:
                    st.error("Select a mission target first.")
        with action_cols[1]:
            if st.button("Center on Rover", use_container_width=True):
                if st.session_state.telemetry.gps_lat is not None and st.session_state.telemetry.gps_lon is not None:
                    st.session_state.target_lat = float(st.session_state.telemetry.gps_lat)
                    st.session_state.target_lon = float(st.session_state.telemetry.gps_lon)
                    st.session_state.mission_status = "Target moved to rover position"
                    st.rerun()
                else:
                    st.warning("Rover position is not available yet.")

        st.divider()
        with st.expander("Advanced mission options", expanded=False):
            mission_text = st.text_area("Optional waypoint list", st.session_state.mission_text, height=140)
            st.session_state.mission_text = mission_text
            waypoints = parse_waypoints(mission_text)
            if waypoints:
                st.dataframe(pd.DataFrame([waypoint.as_dict() for waypoint in waypoints]), use_container_width=True, hide_index=True)
            mission_name = st.text_input("Mission ID", st.session_state.mission_name)
            st.session_state.mission_name = mission_name
            adv_cols = st.columns(3)
            with adv_cols[0]:
                if st.button("Upload Waypoint Mission", use_container_width=True):
                    if waypoints:
                        send_command(mission_command(waypoints, mission_name), f"Waypoint mission '{mission_name}' uploaded")
                    else:
                        st.error("Add at least one valid waypoint first.")
            with adv_cols[1]:
                if st.button("Request Status", use_container_width=True):
                    send_command(status_command(), "Status requested")
            with adv_cols[2]:
                if st.button("Ping Rover", use_container_width=True):
                    send_command(ping_command(), "Ping sent")


def render_control_panel() -> None:
    st.subheader("Manual Control")
    c1, c2, c3 = st.columns(3)
    with c1:
        st.session_state.manual_speed = st.slider("Base speed", 0, 255, int(st.session_state.manual_speed), 5)
    with c2:
        st.session_state.manual_turn = st.slider("Turn bias", 0, 255, int(st.session_state.manual_turn), 5)
    with c3:
        st.session_state.mag_cal_seconds = st.slider("Mag calibration seconds", 5, 30, int(st.session_state.mag_cal_seconds), 1)

    control_button_row()

    safety_cols = st.columns(3)
    with safety_cols[0]:
        if st.button("Emergency Stop", type="primary", use_container_width=True):
            send_command(estop_command(), "Emergency stop sent")
    with safety_cols[1]:
        if st.session_state.telemetry.mag_cal_active:
            st.progress(min(max(int(st.session_state.telemetry.mag_cal_progress or 0), 0), 100))
            if st.button("Stop Magnetometer Calibration", use_container_width=True):
                send_command(calibrate_magnetometer_stop_command(), "Stop calibration sent")
        else:
            if st.button("Calibrate Magnetometer", use_container_width=True):
                send_command(calibrate_magnetometer_command(int(st.session_state.mag_cal_seconds)), "Magnetometer calibration requested")
    with safety_cols[2]:
        if st.button("Stop Motors", use_container_width=True):
            send_command(stop_command(), "Stop motors sent")


def render_sidebar() -> None:
    st.sidebar.title("Rover Control Station")
    ports = available_ports()
    if not ports:
        ports = [""]
    if st.session_state.selected_port not in ports:
        st.session_state.selected_port = ports[0]

    st.session_state.selected_port = st.sidebar.selectbox("Serial port", ports, index=ports.index(st.session_state.selected_port))
    st.session_state.baudrate = st.sidebar.selectbox("Baud rate", [9600, 115200, 230400, 460800], index=[9600, 115200, 230400, 460800].index(st.session_state.baudrate) if st.session_state.baudrate in [9600, 115200, 230400, 460800] else 1)
    st.session_state.auto_refresh_s = st.sidebar.slider("Auto refresh (seconds)", 1, 10, int(st.session_state.auto_refresh_s))

    connection = st.session_state.client.connection
    st.sidebar.write(f"Status: {'Connected' if st.session_state.client.connected else 'Disconnected'}")
    if connection:
        st.sidebar.caption(f"{connection.port} @ {connection.baudrate}")
    st.sidebar.caption(st.session_state.connection_message)

    button_cols = st.sidebar.columns(2)
    with button_cols[0]:
        if st.button("Connect", use_container_width=True):
            if st.session_state.selected_port:
                try:
                    st.session_state.client.connect(st.session_state.selected_port, int(st.session_state.baudrate))
                    st.session_state.connection_message = f"Connected to {st.session_state.selected_port}"
                    st.rerun()
                except Exception as exc:  # noqa: BLE001
                    st.session_state.connection_message = f"Connect failed: {exc}"
                    st.error(st.session_state.connection_message)
            else:
                st.warning("Select a serial port first.")
    with button_cols[1]:
        if st.button("Disconnect", use_container_width=True):
            st.session_state.client.disconnect()
            st.session_state.connection_message = "Disconnected"
            st.rerun()

    if st.session_state.client.connected and st.session_state.auto_refresh_s > 0:
        st_autorefresh(interval=st.session_state.auto_refresh_s * 1000, key="rover_autorefresh")


def main() -> None:
    st.set_page_config(page_title="Rover Control Station", layout="wide")
    init_state()
    render_sidebar()
    poll_rover()

    st.title("Rover Control Station")
    st.caption("Streamlit dashboard for rover telemetry, manual control, and mission upload.")

    tab_dashboard, tab_control, tab_mission, tab_logs, tab_help = st.tabs(["Dashboard", "Control", "Mission", "Logs", "Protocol"])

    with tab_dashboard:
        render_dashboard()

    with tab_control:
        render_control_panel()

    with tab_mission:
        render_mission()

    with tab_logs:
        render_logs()

    with tab_help:
        st.subheader("Serial Protocol")
        st.markdown(
            """
            **Commands sent by the dashboard**
            - `{"cmd":"drive","left":120,"right":120}`
            - `{"cmd":"stop"}`
            - `{"cmd":"estop"}`
            - `{"cmd":"clear_estop"}`
            - `{"cmd":"status"}`
            - `{"cmd":"ping"}`
            - `{"cmd":"calibrate_mag","duration_s":15}`
            - `{"mission":"start","lat":23.8103,"lon":90.4125,"radius":35,"radius_cm":35,"radius_m":0.35}`
            - `{"cmd":"mission","mission_id":"mission-1","waypoints":[... ]}`

            **Telemetry accepted by the dashboard**
            - JSON telemetry lines with keys such as `battery_v`, `battery_pct`, `gps_lat`, `gps_lon`, `heading_deg`, `distance_cm`, `temperature_c`, `humidity_pct`, `gas_raw`, and `metal_raw`
            - Key-value lines like `battery_v=12.2|battery_pct=84|gps_lat=12.97|gps_lon=77.59`
            - Event lines with `type=event|level=INFO|message=...`
            """
        )


if __name__ == "__main__":
    main()