import json
import math
import socket
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import plotly.graph_objects as go
import streamlit as st

try:
    import serial
except ImportError:  # pragma: no cover - shown in the UI
    serial = None

try:
    from streamlit_drawable_canvas import st_canvas
except Exception:  # pragma: no cover - optional UI component
    st_canvas = None

try:
    from streamlit_autorefresh import st_autorefresh
except Exception:  # pragma: no cover - optional UI component
    st_autorefresh = None

Point = Dict[str, float]

DEFAULT_PORT = "COM7"
DEFAULT_BAUD = 115200
DEFAULT_UDP_PORT = 4210
MAX_LOG_LINES = 350
MAX_MARKS = 250


st.set_page_config(page_title="Mine Detection Rover", page_icon="🤖", layout="wide")


def init_state() -> None:
    defaults = {
        "udp_sock": None,
        "udp_port": DEFAULT_UDP_PORT,
        "raw_log": [],
        "tx_log": [],
        "latest_telemetry": {},
        "metal_marks": [],
        "current_path": [],
        "last_status": "Disconnected",
        "serial_port": DEFAULT_PORT,
        "serial_baud": DEFAULT_BAUD,
        "auto_refresh_enabled": False,
        "suspend_autorefresh_until": 0.0,
        "last_mission_send_status": "",
        "last_mission_sent_at": "",
        "last_mission_point_count": 0,
        "last_metal_mark_ts": 0.0,
    }
    for key, value in defaults.items():
        st.session_state.setdefault(key, value)


init_state()


class SerialManager:
    """One shared serial owner for all Streamlit reruns/sessions in this process."""

    def __init__(self) -> None:
        self.conn: Any = None
        self.buffer = ""
        self.status = "Disconnected"
        self.lock = threading.RLock()

    def is_connected(self) -> bool:
        return self.conn is not None and getattr(self.conn, "is_open", False)

    def _close_locked(self) -> None:
        if self.conn is not None:
            try:
                self.conn.close()
            except Exception:
                pass
        self.conn = None
        self.buffer = ""

    def connect(self, port: str, baud: int) -> str:
        if serial is None:
            self.status = "Install pyserial first: python -m pip install -r dashboard/requirements.txt"
            return self.status

        with self.lock:
            if self.is_connected() and getattr(self.conn, "port", None) == port and getattr(self.conn, "baudrate", None) == baud:
                self.status = f"Connected to {port} @ {baud}"
                return self.status

            self._close_locked()
            conn = None
            try:
                conn = serial.Serial(port=port, baudrate=baud, timeout=0, write_timeout=1)
                self.conn = conn
                self.buffer = ""
                self.status = f"Connected to {port} @ {baud}"
            except Exception as exc:
                if conn is not None:
                    try:
                        conn.close()
                    except Exception:
                        pass
                self.conn = None
                self.status = f"Serial connection failed: {exc}"
            return self.status

    def disconnect(self) -> str:
        with self.lock:
            self._close_locked()
            self.status = "Disconnected"
            return self.status

    def send_line(self, line: str) -> None:
        with self.lock:
            if not self.is_connected():
                raise RuntimeError("serial port is not connected")
            self.conn.write((line + "\n").encode("utf-8"))

    def read_lines(self) -> List[str]:
        lines: List[str] = []
        with self.lock:
            if not self.is_connected():
                return lines
            try:
                waiting = self.conn.in_waiting
                if waiting <= 0:
                    return lines
                data = self.conn.read(waiting).decode("utf-8", errors="replace")
            except Exception as exc:
                self.status = f"Serial read failed: {exc}"
                return [json.dumps({"type": "event", "level": "ERROR", "category": "SERIAL", "message": self.status})]

            self.buffer += data
            while "\n" in self.buffer:
                line, self.buffer = self.buffer.split("\n", 1)
                if line.strip():
                    lines.append(line)
        return lines
    
class UdpSender:
    """Sends JSON commands to the ESP32 over UDP."""

    def __init__(self) -> None:
        self.esp_ip: str = ""
        self.esp_port: int = 4211          # must match WIFI_CMD_PORT on ESP32
        self.ready: bool = False

    def configure(self, esp_ip: str, esp_port: int) -> str:
        if not esp_ip:
            self.ready = False
            return "No ESP32 IP provided"
        self.esp_ip = esp_ip
        self.esp_port = esp_port
        self.ready = True
        return f"UDP sender ready → {esp_ip}:{esp_port}"

    def send_line(self, line: str) -> None:
        if not self.ready:
            raise RuntimeError("UDP sender not configured")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto((line + "\n").encode("utf-8"), (self.esp_ip, self.esp_port))
        finally:
            sock.close()


@st.cache_resource
def get_udp_sender() -> UdpSender:
    return UdpSender()


@st.cache_resource
def get_serial_manager() -> SerialManager:
    return SerialManager()


def append_log(line: str, source: str = "serial") -> None:
    st.session_state.raw_log.append(
        {
            "time": time.strftime("%H:%M:%S"),
            "source": source,
            "line": line,
        }
    )
    if len(st.session_state.raw_log) > MAX_LOG_LINES:
        st.session_state.raw_log = st.session_state.raw_log[-MAX_LOG_LINES:]


def add_metal_mark(payload: Dict[str, Any], source: str, reason: str = "alert") -> bool:
    """Add/dedupe a red whiteboard dot from alert or telemetry coordinates."""
    x_cm = payload.get("x_cm")
    y_cm = payload.get("y_cm")
    if not isinstance(x_cm, (int, float)) or not isinstance(y_cm, (int, float)):
        return False

    now = time.time()
    last_ts = float(st.session_state.get("last_metal_mark_ts", 0.0) or 0.0)
    if st.session_state.metal_marks:
        last = st.session_state.metal_marks[-1]
        dx = float(x_cm) - float(last.get("x_cm", 0.0))
        dy = float(y_cm) - float(last.get("y_cm", 0.0))
        same_place = (dx * dx + dy * dy) ** 0.5 < 2.0
        if same_place and now - last_ts < 0.75:
            return False

    st.session_state.metal_marks.append(
        {
            "x_cm": float(x_cm),
            "y_cm": float(y_cm),
            "adc": payload.get("adc", payload.get("metal_adc")),
            "adc_min": payload.get("metal_adc_min"),
            "adc_drop": payload.get("adc_drop", payload.get("metal_adc_drop")),
            "confidence": payload.get("confidence"),
            "source": source,
            "reason": reason,
            "time": time.strftime("%H:%M:%S"),
            "ts": now,
        }
    )
    st.session_state.last_metal_mark_ts = now
    if len(st.session_state.metal_marks) > MAX_MARKS:
        st.session_state.metal_marks = st.session_state.metal_marks[-MAX_MARKS:]
    return True


def process_incoming_line(line: str, source: str = "serial") -> None:
    line = line.strip()
    if not line:
        return

    append_log(line, source)
    try:
        payload = json.loads(line)
    except json.JSONDecodeError:
        return

    message_type = payload.get("type")
    if message_type == "telemetry":
        merged = dict(st.session_state.latest_telemetry)
        merged.update(payload)
        merged["dashboard_received_at"] = time.strftime("%H:%M:%S")
        st.session_state.latest_telemetry = merged

        detect_min = merged.get("metal_adc_detect_min", 0)
        detect_max = merged.get("metal_adc_detect_max", 100)
        adc_now = merged.get("metal_adc")
        adc_min = merged.get("metal_adc_min", adc_now)
        adc_is_low = isinstance(adc_now, (int, float)) and detect_min <= adc_now <= detect_max
        adc_min_is_low = isinstance(adc_min, (int, float)) and detect_min <= adc_min <= detect_max
        if merged.get("metal_detected") or adc_is_low or adc_min_is_low:
            reason = "telemetry" if merged.get("metal_detected") or adc_is_low else "telemetry_min"
            add_metal_mark(merged, source, reason)
    elif message_type == "alert" and payload.get("alert_type") == "metal_detected":
        add_metal_mark(payload, source, "alert")


def is_serial_connected() -> bool:
    return get_serial_manager().is_connected()


def connect_serial(port: str, baud: int) -> None:
    status = get_serial_manager().connect(port, baud)
    st.session_state.last_status = status
    append_log(status, "system")


def disconnect_serial() -> None:
    status = get_serial_manager().disconnect()
    st.session_state.last_status = status
    append_log("Serial disconnected", "system")


def ensure_serial_connected() -> bool:
    if is_serial_connected():
        return True

    port = st.session_state.get("serial_port") or DEFAULT_PORT
    baud = int(st.session_state.get("serial_baud") or DEFAULT_BAUD)
    connect_serial(port, baud)
    return is_serial_connected()


def send_command(payload: Dict[str, Any]) -> bool:
    line = json.dumps(payload, separators=(",", ":"))
    sender = get_udp_sender()
    try:
        sender.send_line(line)
        st.session_state.tx_log.append({"time": time.strftime("%H:%M:%S"), "line": line})
        st.session_state.tx_log = st.session_state.tx_log[-MAX_LOG_LINES:]
        return True
    except Exception as exc:
        st.error(f"Failed to send UDP command: {exc}")
        append_log(f"UDP TX failed: {exc}", "system")
        return False


def get_default_ground_station_ip() -> str:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect(("8.8.8.8", 80))
        ip = sock.getsockname()[0]
        sock.close()
        return ip
    except Exception:
        return "192.168.1.100"


def metric_value(value: Any, digits: int = 1) -> Any:
    if value is None:
        return "—"
    if isinstance(value, float):
        return round(value, digits)
    return value


def send_path_mission(points: List[Point]) -> None:
    """Callback-safe mission sender used by the Mission tab button."""
    st.session_state.suspend_autorefresh_until = time.time() + 5.0

    if not points:
        st.session_state.last_mission_send_status = "No mission points prepared."
        return

    payload = {"cmd": "path", "mission_id": "board-path", "points": points}
    st.session_state.current_path = list(points)
    st.session_state.last_mission_point_count = len(points)
    st.session_state.last_mission_sent_at = time.strftime("%H:%M:%S")

    if send_command(payload):
        st.session_state.last_mission_send_status = f"Sent path mission with {len(points)} point(s)."
    else:
        st.session_state.last_mission_send_status = "Mission was not sent. Configure the UDP sender or connect serial first."


def drain_serial() -> None:
    for line in get_serial_manager().read_lines():
        process_incoming_line(line, "serial")


def start_udp_listener(port: int) -> None:
    stop_udp_listener()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    sock.setblocking(False)
    st.session_state.udp_sock = sock
    st.session_state.udp_port = port
    append_log(f"UDP listener started on port {port}", "system")


def stop_udp_listener() -> None:
    sock = st.session_state.udp_sock
    if sock is not None:
        try:
            sock.close()
        except Exception:
            pass
    st.session_state.udp_sock = None


def drain_udp() -> None:
    sock = st.session_state.udp_sock
    if sock is None:
        return

    while True:
        try:
            data, addr = sock.recvfrom(4096)
        except BlockingIOError:
            break
        except Exception as exc:
            append_log(f"UDP receive failed: {exc}", "system")
            break
        process_incoming_line(data.decode("utf-8", errors="replace"), f"udp:{addr[0]}")


def point_dict(x: float, y: float) -> Point:
    return {"x_cm": round(float(x), 2), "y_cm": round(float(y), 2)}


def generate_circle(radius_cm: float, segments: int) -> List[Point]:
    segments = max(8, segments)
    return [
        point_dict(radius_cm * math.sin(2.0 * math.pi * i / segments), radius_cm * math.cos(2.0 * math.pi * i / segments))
        for i in range(segments + 1)
    ]


def generate_square(side_cm: float) -> List[Point]:
    half = side_cm / 2.0
    return [
        point_dict(-half, half),
        point_dict(half, half),
        point_dict(half, -half),
        point_dict(-half, -half),
        point_dict(-half, half),
    ]


def generate_triangle(side_cm: float) -> List[Point]:
    height = side_cm * math.sqrt(3.0) / 2.0
    return [
        point_dict(0.0, height / 2.0),
        point_dict(side_cm / 2.0, -height / 2.0),
        point_dict(-side_cm / 2.0, -height / 2.0),
        point_dict(0.0, height / 2.0),
    ]


def generate_lawnmower(width_cm: float, height_cm: float, spacing_cm: float) -> List[Point]:
    spacing_cm = max(5.0, spacing_cm)
    left = -width_cm / 2.0
    right = width_cm / 2.0
    top = height_cm / 2.0
    rows = max(1, int(math.floor(height_cm / spacing_cm)))
    points: List[Point] = []
    for row in range(rows + 1):
        y = top - min(row * spacing_cm, height_cm)
        if row % 2 == 0:
            points.extend([point_dict(left, y), point_dict(right, y)])
        else:
            points.extend([point_dict(right, y), point_dict(left, y)])
    return points


def parse_freehand_text(text: str) -> List[Point]:
    points: List[Point] = []
    for raw_line in text.replace(";", "\n").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        parts = line.replace(",", " ").split()
        if len(parts) < 2:
            continue
        try:
            points.append(point_dict(float(parts[0]), float(parts[1])))
        except ValueError:
            continue
    return points


def extract_canvas_points(canvas_json: Optional[Dict[str, Any]], scale_cm_per_px: float, board_px: int) -> List[Point]:
    if not canvas_json:
        return []

    center = board_px / 2.0
    raw_points: List[Tuple[float, float]] = []
    for obj in canvas_json.get("objects", []):
        if obj.get("type") != "path":
            continue
        left = float(obj.get("left", 0.0))
        top = float(obj.get("top", 0.0))
        for command in obj.get("path", []):
            if not isinstance(command, list) or len(command) < 3:
                continue
            numbers = [value for value in command[1:] if isinstance(value, (int, float))]
            if len(numbers) >= 2:
                x_px = left + float(numbers[-2])
                y_px = top + float(numbers[-1])
                raw_points.append((x_px, y_px))

    # Down-sample so serial JSON remains small enough for the ESP32 parser.
    if len(raw_points) > 45:
        step = max(1, len(raw_points) // 45)
        raw_points = raw_points[::step]

    points = []
    previous: Optional[Point] = None
    for x_px, y_px in raw_points:
        point = point_dict((x_px - center) * scale_cm_per_px, (center - y_px) * scale_cm_per_px)
        if previous is None or abs(point["x_cm"] - previous["x_cm"]) > 1.0 or abs(point["y_cm"] - previous["y_cm"]) > 1.0:
            points.append(point)
            previous = point
    return points


def make_board_figure(points: List[Point], marks: List[Dict[str, Any]], board_cm: float) -> go.Figure:
    fig = go.Figure()

    if points:
        fig.add_trace(
            go.Scatter(
                x=[p["x_cm"] for p in points],
                y=[p["y_cm"] for p in points],
                mode="lines+markers",
                name="Mission path",
                line={"color": "#1f77b4", "width": 3},
                marker={"size": 6},
            )
        )

    fig.add_trace(
        go.Scatter(
            x=[0],
            y=[0],
            mode="markers+text",
            text=["START"],
            textposition="bottom center",
            name="Blue center start",
            marker={"size": 14, "color": "blue"},
        )
    )

    if marks:
        fig.add_trace(
            go.Scatter(
                x=[m["x_cm"] for m in marks],
                y=[m["y_cm"] for m in marks],
                mode="markers",
                name="Metal alerts",
                marker={"size": 14, "color": "red", "symbol": "circle"},
                text=[f"{m.get('time', '')} adc={m.get('adc', '')} confidence={m.get('confidence', '')}" for m in marks],
            )
        )

    half = max(board_cm / 2.0, 50.0)
    fig.update_layout(
        height=560,
        xaxis_title="x_cm",
        yaxis_title="y_cm",
        xaxis={"range": [-half, half], "zeroline": True},
        yaxis={"range": [-half, half], "scaleanchor": "x", "scaleratio": 1, "zeroline": True},
        margin={"l": 10, "r": 10, "t": 35, "b": 10},
        legend={"orientation": "h"},
    )
    return fig


def render_mission_board(preview_points: List[Point], board_cm: float) -> None:
    # Keep mission red dots live while a path is running without reloading mission controls.
    drain_serial()
    drain_udp()
    st.plotly_chart(make_board_figure(preview_points, st.session_state.metal_marks, board_cm), width="stretch")


if hasattr(st, "fragment"):
    render_mission_board = st.fragment(run_every="250ms")(render_mission_board)


# Read any data already waiting before rendering.
drain_serial()
drain_udp()

autorefresh_allowed = time.time() >= float(st.session_state.get("suspend_autorefresh_until", 0.0))
if st.session_state.auto_refresh_enabled and autorefresh_allowed and st_autorefresh is not None:
    st_autorefresh(interval=1000, limit=None, key="rover_live_refresh_1s")

st.title("🤖 Mine Detection Rover Dashboard")
st.caption("ESP32 USB serial control, local whiteboard path missions, metal detection alerts, and optional UDP alerts.")

with st.sidebar:
    st.header("Connection")
    port = st.text_input("ESP32 serial port", key="serial_port")
    baud = st.selectbox("Baud rate", [115200, 57600, 38400, 9600], index=0, key="serial_baud")

    col_a, col_b = st.columns(2)
    with col_a:
        if st.button("Connect", width="stretch"):
            connect_serial(port, int(baud))
    with col_b:
        if st.button("Disconnect", width="stretch"):
            disconnect_serial()

    serial_status = get_serial_manager().status or st.session_state.last_status
    st.write(f"**Status:** {serial_status}")

    st.session_state.auto_refresh_enabled = st.checkbox(
        "Auto update every 1s",
        value=st.session_state.auto_refresh_enabled,
        help="Use this only while monitoring telemetry. Turn it off before sending path missions to avoid page reruns during button clicks.",
    )
    if not st.session_state.auto_refresh_enabled:
        st.caption("Auto update is off. Use Refresh telemetry/logs or enable it only while monitoring.")
    if st.session_state.auto_refresh_enabled and st_autorefresh is None:
        st.warning("Install streamlit-autorefresh for live 1s updates: python -m pip install -r dashboard/requirements.txt")

    st.divider()
    st.header("UDP metal alerts")
    udp_port = st.number_input("UDP port", min_value=1, max_value=65535, value=st.session_state.udp_port, step=1)
    col_udp_a, col_udp_b = st.columns(2)
    with col_udp_a:
        if st.button("Start UDP", width="stretch"):
            try:
                start_udp_listener(int(udp_port))
            except Exception as exc:
                st.error(f"UDP start failed: {exc}")
    with col_udp_b:
        if st.button("Stop UDP", width="stretch"):
            stop_udp_listener()
    st.write("UDP: " + (f"listening on {st.session_state.udp_port}" if st.session_state.udp_sock else "stopped"))

    with st.expander("WiFi configuration", expanded=True):
        st.caption("Both ESP32 and this PC must be on the same WiFi network.")

        st.subheader("Ground station (this PC)")
        udp_port = st.number_input("Dashboard UDP listen port", min_value=1,
                                    max_value=65535, value=st.session_state.udp_port, step=1)
        col_udp_a, col_udp_b = st.columns(2)
        with col_udp_a:
            if st.button("Start UDP listener", width="stretch"):
                try:
                    start_udp_listener(int(udp_port))
                except Exception as exc:
                    st.error(f"UDP start failed: {exc}")
        with col_udp_b:
            if st.button("Stop UDP listener", width="stretch"):
                stop_udp_listener()
        st.write("Listener: " + (f"✅ port {st.session_state.udp_port}"
                                  if st.session_state.udp_sock else "⛔ stopped"))

        st.divider()
        st.subheader("ESP32 rover")
        esp_ip = st.text_input("ESP32 IP address",
                                value=st.session_state.get("esp_ip", ""),
                                key="esp_ip",
                                help="Check serial monitor at boot — ESP32 prints its IP.")
        esp_cmd_port = st.number_input("ESP32 command port", min_value=1,
                                        max_value=65535, value=4211, step=1)
        gs_ip = st.text_input("This PC's LAN IP (for metal alerts)",
                               value=get_default_ground_station_ip(),
                               key="wifi_host")
        wifi_ssid = st.text_input("WiFi SSID", value="", key="wifi_ssid")
        wifi_pass = st.text_input("WiFi password", value="",
                                   type="password", key="wifi_password")

        col_w1, col_w2 = st.columns(2)
        with col_w1:
            if st.button("Connect UDP sender", width="stretch"):
                if not esp_ip.strip():
                    st.warning("Enter the ESP32 IP address first.")
                else:
                    status = get_udp_sender().configure(esp_ip.strip(), int(esp_cmd_port))
                    append_log(status, "system")
                    st.success(status)
        with col_w2:
            if st.button("Test Metal Alert", width="stretch"):
                send_command({"cmd": "test_metal_alert"})

        st.divider()
        if st.button("Refresh telemetry/logs", width="stretch"):
            drain_serial()
            drain_udp()

control_tab, mission_tab, telemetry_tab = st.tabs(["Control", "Mission", "Telemetry & Logs"])

with control_tab:
    st.subheader("Manual motor control")
    st.info("With ENA/ENB jumpers installed on the L298N, speed is used as a direction/on threshold by the firmware.")

    base_speed = st.slider("Base speed command", min_value=0, max_value=255, value=180, step=5)
    turn_speed = st.slider("Turn speed command", min_value=0, max_value=255, value=170, step=5)
    duration_ms = st.number_input("Drive pulse duration (ms)", min_value=100, max_value=10000, value=900, step=100)

    c1, c2, c3 = st.columns(3)
    with c2:
        if st.button("⬆ Forward", width="stretch"):
            send_command({"cmd": "drive", "left": base_speed, "right": base_speed, "duration_ms": int(duration_ms)})
    c4, c5, c6 = st.columns(3)
    with c4:
        if st.button("⬅ Left", width="stretch"):
            send_command({"cmd": "drive", "left": -turn_speed, "right": turn_speed, "duration_ms": int(duration_ms)})
    with c5:
        if st.button("■ Stop Motors", width="stretch"):
            send_command({"cmd": "stop"})
    with c6:
        if st.button("Right ➡", width="stretch"):
            send_command({"cmd": "drive", "left": turn_speed, "right": -turn_speed, "duration_ms": int(duration_ms)})
    c7, c8, c9 = st.columns(3)
    with c8:
        if st.button("⬇ Back", width="stretch"):
            send_command({"cmd": "drive", "left": -base_speed, "right": -base_speed, "duration_ms": int(duration_ms)})

    st.divider()
    safety_a, safety_b, safety_c = st.columns(3)
    with safety_a:
        if st.button("🚨 Emergency Stop", type="primary", width="stretch"):
            send_command({"cmd": "estop"})
    with safety_b:
        if st.button("Reset E-stop", width="stretch"):
            send_command({"cmd": "reset_estop"})
    with safety_c:
        if st.button("Ping rover", width="stretch"):
            send_command({"cmd": "ping"})

with mission_tab:
    st.subheader("Local whiteboard path mission")

    left_col, right_col = st.columns([0.9, 1.1])
    with left_col:
        path_source = st.selectbox("Path source", ["Circle", "Square", "Triangle", "Lawnmower", "Freehand"])
        board_px = st.slider("Whiteboard size (pixels)", min_value=300, max_value=700, value=420, step=20)
        scale_cm_per_px = st.number_input("Board scale (cm / pixel)", min_value=0.05, max_value=5.0, value=0.5, step=0.05)
        return_to_center = st.checkbox("Return to center", value=True)

        generated_points: List[Point] = []
        if path_source == "Circle":
            radius_cm = st.number_input("Circle radius (cm)", min_value=10.0, max_value=250.0, value=50.0, step=5.0)
            segments = st.slider("Circle segments", min_value=8, max_value=48, value=20, step=1)
            generated_points = generate_circle(radius_cm, segments)
        elif path_source == "Square":
            side_cm = st.number_input("Square side (cm)", min_value=10.0, max_value=300.0, value=90.0, step=5.0)
            generated_points = generate_square(side_cm)
        elif path_source == "Triangle":
            side_cm = st.number_input("Triangle side (cm)", min_value=10.0, max_value=300.0, value=100.0, step=5.0)
            generated_points = generate_triangle(side_cm)
        elif path_source == "Lawnmower":
            width_cm = st.number_input("Coverage width (cm)", min_value=20.0, max_value=400.0, value=120.0, step=5.0)
            height_cm = st.number_input("Coverage height (cm)", min_value=20.0, max_value=400.0, value=90.0, step=5.0)
            spacing_cm = st.number_input("Row spacing (cm)", min_value=5.0, max_value=80.0, value=25.0, step=5.0)
            generated_points = generate_lawnmower(width_cm, height_cm, spacing_cm)
        else:
            if st_canvas is not None:
                st.write("Draw a freehand path. The blue start dot is the rover center (0,0).")
                canvas_result = st_canvas(
                    fill_color="rgba(0, 0, 0, 0)",
                    stroke_width=3,
                    stroke_color="#1f77b4",
                    background_color="#ffffff",
                    height=board_px,
                    width=board_px,
                    drawing_mode="freedraw",
                    key="mission_canvas",
                )
                generated_points = extract_canvas_points(canvas_result.json_data, scale_cm_per_px, board_px)
            else:
                st.warning("Install streamlit-drawable-canvas for drawing. Falling back to text input.")
                text_points = st.text_area(
                    "Freehand points as x_cm,y_cm lines",
                    value="0,50\n50,50\n0,0",
                    height=150,
                )
                generated_points = parse_freehand_text(text_points)

        mission_points = list(generated_points)
        if return_to_center and mission_points:
            last = mission_points[-1]
            if abs(last["x_cm"]) > 0.1 or abs(last["y_cm"]) > 0.1:
                mission_points.append(point_dict(0.0, 0.0))

        st.write(f"Prepared **{len(mission_points)}** point(s).")
        udp_ready = get_udp_sender().ready
        serial_ready = is_serial_connected()
        if not serial_ready and not udp_ready:
            st.warning("Rover is not connected. Connect via serial (sidebar) or configure the UDP sender in WiFi config.")
        elif udp_ready and not serial_ready:
            st.success("UDP sender ready — path mission will be sent over WiFi.")
        with st.expander("Mission JSON preview"):
            st.code(json.dumps({"cmd": "path", "mission_id": "board-path", "points": mission_points}, indent=2), language="json")

        send_col, clear_col = st.columns(2)
        with send_col:
            st.button(
                "Send Path Mission",
                type="primary",
                width="stretch",
                disabled=not mission_points,
                on_click=send_path_mission,
                args=(list(mission_points),),
            )
        with clear_col:
            if st.button("Clear Metal Marks", width="stretch"):
                st.session_state.metal_marks = []

        if st.session_state.last_mission_send_status:
            st.info(
                f"{st.session_state.last_mission_send_status} "
                f"Last send time: {st.session_state.last_mission_sent_at or '—'}. "
                "Check Telemetry & Logs → Sent commands for the JSON path command."
            )

    with right_col:
        board_cm = board_px * scale_cm_per_px
        preview_points = mission_points or st.session_state.current_path
        render_mission_board(preview_points, board_cm)

def render_telemetry_panel() -> None:
    # Fast telemetry-only refresh: this fragment reruns without reloading the whole page,
    # so Mission/Control button clicks are not interrupted.
    drain_serial()
    drain_udp()

    st.subheader("Telemetry")
    tel = st.session_state.latest_telemetry
    if tel:
        metric_cols = st.columns(8)
        metric_cols[0].metric("Mode", tel.get("mode", "?"))
        metric_cols[1].metric("Distance cm", metric_value(tel.get("distance_cm"), 1))
        metric_cols[2].metric("Metal", "YES" if tel.get("metal_detected") else "no")
        adc_now = tel.get("metal_adc", "?")
        adc_min = tel.get("metal_adc_min", adc_now)
        metric_cols[3].metric("ADC now/min", f"{adc_now} / {adc_min}")
        metric_cols[4].metric("DHT temp C", metric_value(tel.get("dht11_temperature_c"), 1))
        metric_cols[5].metric("DHT humidity %", metric_value(tel.get("dht11_humidity_percent"), 1))
        metric_cols[6].metric("Obstacles", tel.get("obstacle_count", "?"))
        metric_cols[7].metric("Heading", metric_value(tel.get("heading_deg"), 1))

        path_cols = st.columns(4)
        path_cols[0].metric("Target index", tel.get("mission_target_index", "?"))
        path_cols[1].metric("Remember path", "yes" if tel.get("remembered_target_valid") else "no")
        path_cols[2].metric("Avoiding obstacle", "YES" if tel.get("avoiding_obstacle") else "no")
        path_cols[3].metric("Last obstacle cm", metric_value(tel.get("last_obstacle_cm"), 1))

        detect_min = tel.get("metal_adc_detect_min", 0)
        detect_max = tel.get("metal_adc_detect_max", 100)
        st.caption(
            f"Fast telemetry: ESP32 sends every ~200 ms. Active-low metal detection: "
            f"**metal = GPIO35 ADC {detect_min}-{detect_max}**, no metal is usually near **4095**. "
            "Red dots are added from metal alerts and metal-detected telemetry."
        )
        st.json(tel)
    else:
        st.info("No telemetry received yet. Connect serial and click Ping or wait for the rover heartbeat.")

    st.divider()
    st.subheader("Metal marks")
    if st.session_state.metal_marks:
        latest_mark = st.session_state.metal_marks[-1]
        st.success(
            f"Latest red dot: x={latest_mark['x_cm']:.1f} cm, y={latest_mark['y_cm']:.1f} cm, "
            f"ADC={latest_mark.get('adc', '?')}, source={latest_mark.get('source', '?')}"
        )
        st.dataframe(st.session_state.metal_marks, width="stretch")
    else:
        st.write("No metal marks yet.")

    st.divider()
    log_col, tx_col = st.columns(2)
    with log_col:
        st.subheader("Incoming log")
        for item in reversed(st.session_state.raw_log[-120:]):
            st.text(f"[{item['time']}] {item['source']}: {item['line']}")
    with tx_col:
        st.subheader("Sent commands")
        for item in reversed(st.session_state.tx_log[-120:]):
            st.text(f"[{item['time']}] {item['line']}")


if hasattr(st, "fragment"):
    render_telemetry_panel = st.fragment(run_every="250ms")(render_telemetry_panel)


with telemetry_tab:
    render_telemetry_panel()