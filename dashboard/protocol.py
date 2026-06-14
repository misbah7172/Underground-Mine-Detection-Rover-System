from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, Iterable, List, Optional, Tuple
import json


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _coerce_value(value: str) -> Any:
    text = value.strip()
    if not text:
        return ""
    lowered = text.lower()
    if lowered in {"true", "yes", "on"}:
        return True
    if lowered in {"false", "no", "off"}:
        return False
    try:
        if text.startswith("0") and text not in {"0", "0.0"} and not text.startswith("0."):
            return text
        number = float(text)
        if number.is_integer():
            return int(number)
        return number
    except ValueError:
        return text


def _get_first(mapping: Dict[str, Any], keys: Iterable[str]) -> Any:
    for key in keys:
        if key in mapping and mapping[key] not in (None, ""):
            return mapping[key]
    return None


def _as_float(mapping: Dict[str, Any], keys: Iterable[str]) -> Optional[float]:
    value = _get_first(mapping, keys)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _as_int(mapping: Dict[str, Any], keys: Iterable[str]) -> Optional[int]:
    value = _get_first(mapping, keys)
    if value is None:
        return None
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return None


def _as_bool(mapping: Dict[str, Any], keys: Iterable[str]) -> Optional[bool]:
    value = _get_first(mapping, keys)
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "on", "detected", "present"}:
        return True
    if text in {"0", "false", "no", "off", "absent", "clear"}:
        return False
    return None


@dataclass
class TelemetrySnapshot:
    timestamp: str = field(default_factory=_now_iso)
    source: str = "serial"
    mode: Optional[str] = None
    state: Optional[str] = None
    battery_v: Optional[float] = None
    battery_pct: Optional[float] = None
    current_ma: Optional[float] = None
    distance_cm: Optional[float] = None
    temperature_c: Optional[float] = None
    humidity_pct: Optional[float] = None
    gas_raw: Optional[float] = None
    metal_raw: Optional[float] = None
    metal_detected: Optional[bool] = None
    mag_cal_active: Optional[bool] = None
    mag_cal_progress: Optional[int] = None
    metal_freq_hz: Optional[float] = None
    metal_freq_dev_pct: Optional[float] = None
    metal_confidence: Optional[int] = None
    gps_lat: Optional[float] = None
    gps_lon: Optional[float] = None
    gps_alt_m: Optional[float] = None
    heading_deg: Optional[float] = None
    speed_left: Optional[float] = None
    speed_right: Optional[float] = None
    obstacle_left_cm: Optional[float] = None
    obstacle_right_cm: Optional[float] = None
    rssi_dbm: Optional[float] = None
    mission_id: Optional[str] = None
    waypoint_index: Optional[int] = None
    message: Optional[str] = None

    def as_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "source": self.source,
            "mode": self.mode,
            "state": self.state,
            "battery_v": self.battery_v,
            "battery_pct": self.battery_pct,
            "current_ma": self.current_ma,
            "distance_cm": self.distance_cm,
            "temperature_c": self.temperature_c,
            "humidity_pct": self.humidity_pct,
            "gas_raw": self.gas_raw,
            "metal_raw": self.metal_raw,
            "metal_detected": self.metal_detected,
            "gps_lat": self.gps_lat,
            "gps_lon": self.gps_lon,
            "gps_alt_m": self.gps_alt_m,
            "heading_deg": self.heading_deg,
            "speed_left": self.speed_left,
            "speed_right": self.speed_right,
            "obstacle_left_cm": self.obstacle_left_cm,
            "obstacle_right_cm": self.obstacle_right_cm,
            "rssi_dbm": self.rssi_dbm,
            "mission_id": self.mission_id,
            "waypoint_index": self.waypoint_index,
            "message": self.message,
            "mag_cal_active": self.mag_cal_active,
            "mag_cal_progress": self.mag_cal_progress,
            "metal_freq_hz": self.metal_freq_hz,
            "metal_freq_dev_pct": self.metal_freq_dev_pct,
            "metal_confidence": self.metal_confidence,
        }

    def to_display_rows(self) -> List[Tuple[str, Any]]:
        rows = []
        for label, value in self.as_dict().items():
            if value not in (None, ""):
                rows.append((label, value))
        return rows


@dataclass
class EventRecord:
    timestamp: str = field(default_factory=_now_iso)
    level: str = "INFO"
    category: str = "LOG"
    message: str = ""
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    extra: Dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        data = {
            "timestamp": self.timestamp,
            "level": self.level,
            "category": self.category,
            "message": self.message,
            "latitude": self.latitude,
            "longitude": self.longitude,
        }
        data.update(self.extra)
        return data


@dataclass
class MissionWaypoint:
    latitude: float
    longitude: float
    radius_m: float = 2.0

    def as_dict(self) -> Dict[str, float]:
        return {
            "lat": float(self.latitude),
            "lon": float(self.longitude),
            "radius_m": float(self.radius_m),
        }


def parse_kv_line(line: str) -> Dict[str, Any]:
    normalized = line.strip().replace(";", "|").replace(",", "|")
    pieces = [piece.strip() for piece in normalized.split("|") if piece.strip()]
    mapping: Dict[str, Any] = {}
    for piece in pieces:
        if "=" in piece:
            key, value = piece.split("=", 1)
        elif ":" in piece:
            key, value = piece.split(":", 1)
        else:
            continue
        mapping[key.strip().lower()] = _coerce_value(value)
    return mapping


def snapshot_from_mapping(mapping: Dict[str, Any], source: str = "serial") -> TelemetrySnapshot:
    lower = {str(key).lower(): value for key, value in mapping.items()}
    return TelemetrySnapshot(
        timestamp=str(_get_first(lower, ["timestamp", "ts", "time"]) or _now_iso()),
        source=source,
        mode=_get_first(lower, ["mode", "control_mode"]),
        state=_get_first(lower, ["state", "status", "mission_state"]),
        battery_v=_as_float(lower, ["battery_v", "battery", "batt_v", "voltage"]),
        battery_pct=_as_float(lower, ["battery_pct", "battery_percent", "charge", "soc"]),
        current_ma=_as_float(lower, ["current_ma", "current", "motor_current_ma"]),
        distance_cm=_as_float(lower, ["distance_cm", "distance", "range_cm", "obstacle_cm"]),
        temperature_c=_as_float(lower, ["temperature_c", "temp_c", "temperature"]),
        humidity_pct=_as_float(lower, ["humidity_pct", "humidity", "hum"]),
        gas_raw=_as_float(lower, ["gas_raw", "mq2_raw", "smoke_raw", "gas"]),
        metal_raw=_as_float(lower, ["metal_raw", "metal", "metal_adc"]),
        metal_detected=_as_bool(lower, ["metal_detected", "metal_present", "mine_detected"]),
        gps_lat=_as_float(lower, ["gps_lat", "lat", "latitude"]),
        gps_lon=_as_float(lower, ["gps_lon", "lon", "longitude"]),
        gps_alt_m=_as_float(lower, ["gps_alt_m", "alt", "altitude"]),
        heading_deg=_as_float(lower, ["heading_deg", "heading", "yaw"]),
        speed_left=_as_float(lower, ["speed_left", "left", "motor_left"]),
        speed_right=_as_float(lower, ["speed_right", "right", "motor_right"]),
        obstacle_left_cm=_as_float(lower, ["obstacle_left_cm", "left_cm", "ultra_left"]),
        obstacle_right_cm=_as_float(lower, ["obstacle_right_cm", "right_cm", "ultra_right"]),
        rssi_dbm=_as_float(lower, ["rssi_dbm", "rssi"]),
        mission_id=_get_first(lower, ["mission_id", "mission", "job"]),
        waypoint_index=_as_int(lower, ["waypoint_index", "wp", "index"]),
        message=_get_first(lower, ["message", "msg", "text"]),
        mag_cal_active=_as_bool(lower, ["mag_cal_active", "magcal_active", "mag_cal"]),
        mag_cal_progress=_as_int(lower, ["mag_cal_progress", "magcal_progress", "mag_progress"]),
        metal_freq_hz=_as_float(lower, ["metal_freq_hz", "metal_frequency", "freq_hz"]),
        metal_freq_dev_pct=_as_float(lower, ["metal_freq_dev_pct", "freq_deviation", "freq_dev"]),
        metal_confidence=_as_int(lower, ["metal_confidence", "confidence", "metal_conf"]),
    )


def parse_event_from_mapping(mapping: Dict[str, Any]) -> EventRecord:
    lower = {str(key).lower(): value for key, value in mapping.items()}
    return EventRecord(
        timestamp=str(_get_first(lower, ["timestamp", "ts", "time"]) or _now_iso()),
        level=str(_get_first(lower, ["level", "severity"]) or "INFO"),
        category=str(_get_first(lower, ["category", "type", "event"]) or "LOG"),
        message=str(_get_first(lower, ["message", "msg", "text"]) or ""),
        latitude=_as_float(lower, ["lat", "latitude", "gps_lat"]),
        longitude=_as_float(lower, ["lon", "longitude", "gps_lon"]),
        extra={
            key: value
            for key, value in lower.items()
            if key not in {"timestamp", "ts", "time", "level", "severity", "category", "type", "event", "message", "msg", "text", "lat", "latitude", "gps_lat", "lon", "longitude", "gps_lon"}
        },
    )


def parse_line(line: str, source: str = "serial") -> Dict[str, Any]:
    text = line.strip()
    if not text:
        return {"kind": "empty"}

    if text.startswith("{") and text.endswith("}"):
        try:
            payload = json.loads(text)
            if isinstance(payload, dict):
                lowered = {str(key).lower(): value for key, value in payload.items()}
                kind = str(_get_first(lowered, ["type", "kind", "message_type"]) or "telemetry").lower()
                if kind in {"telemetry", "status", "state"}:
                    return {"kind": "telemetry", "telemetry": snapshot_from_mapping(payload, source=source), "raw": text}
                if kind in {"event", "log", "alarm"}:
                    return {"kind": "event", "event": parse_event_from_mapping(payload), "raw": text}
                if kind in {"mission", "mission_ack", "ack"}:
                    return {"kind": "event", "event": parse_event_from_mapping(payload), "raw": text}
        except json.JSONDecodeError:
            pass

    mapping = parse_kv_line(text)
    if mapping:
        lowered = {str(key).lower(): value for key, value in mapping.items()}
        kind = str(_get_first(lowered, ["type", "kind"]) or "").lower()
        if kind in {"telemetry", "status"} or any(key in lowered for key in {"battery_v", "gps_lat", "temperature_c", "heading_deg"}):
            return {"kind": "telemetry", "telemetry": snapshot_from_mapping(mapping, source=source), "raw": text}
        if kind in {"event", "log", "alarm"} or any(key in lowered for key in {"level", "message", "msg", "text"}):
            return {"kind": "event", "event": parse_event_from_mapping(mapping), "raw": text}

    return {"kind": "log", "event": EventRecord(category="LOG", message=text), "raw": text}


def parse_waypoints(text: str) -> List[MissionWaypoint]:
    waypoints: List[MissionWaypoint] = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        tokens = [token for token in stripped.replace(";", ",").split(",") if token.strip()]
        if len(tokens) < 2:
            continue
        try:
            latitude = float(tokens[0])
            longitude = float(tokens[1])
            radius = float(tokens[2]) if len(tokens) >= 3 else 2.0
            waypoints.append(MissionWaypoint(latitude=latitude, longitude=longitude, radius_m=radius))
        except ValueError:
            continue
    return waypoints


def build_command(command: str, **payload: Any) -> str:
    data = {"cmd": command}
    data.update(payload)
    return json.dumps(data, separators=(",", ":")) + "\n"


def drive_command(left: int, right: int, duration_ms: int = 0) -> str:
    return build_command("drive", left=int(left), right=int(right), duration_ms=int(duration_ms))


def stop_command() -> str:
    return build_command("stop")


def estop_command() -> str:
    return build_command("estop")


def ping_command() -> str:
    return build_command("ping")


def status_command() -> str:
    return build_command("status")


def calibrate_magnetometer_command(duration_s: int = 15) -> str:
    return build_command("calibrate_mag", duration_s=int(duration_s))

def calibrate_magnetometer_stop_command() -> str:
    return build_command("calibrate_mag_stop")


def mission_start_packet(latitude: float, longitude: float, radius_cm: float) -> str:
    radius_cm_value = round(float(radius_cm), 2)
    radius_m_value = round(radius_cm_value / 100.0, 4)
    return json.dumps(
        {
            "mission": "start",
            "lat": round(float(latitude), 6),
            "lon": round(float(longitude), 6),
            "radius": radius_cm_value,
            "radius_cm": radius_cm_value,
            "radius_m": radius_m_value,
        },
        separators=(",", ":"),
    ) + "\n"


def mission_command(waypoints: List[MissionWaypoint], name: str = "mission") -> str:
    return build_command(
        "mission",
        mission_id=name,
        waypoints=[waypoint.as_dict() for waypoint in waypoints],
    )