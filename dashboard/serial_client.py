from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional
import time

import serial
from serial.tools import list_ports


def available_ports() -> List[str]:
    return [port.device for port in list_ports.comports()]


@dataclass
class SerialConnectionInfo:
    port: str
    baudrate: int


class RoverSerialClient:
    def __init__(self) -> None:
        self._serial: Optional[serial.Serial] = None
        self._buffer = ""
        self.last_error: Optional[str] = None

    @property
    def connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def connection(self) -> Optional[SerialConnectionInfo]:
        if not self._serial:
            return None
        return SerialConnectionInfo(port=self._serial.port, baudrate=self._serial.baudrate)

    def connect(self, port: str, baudrate: int = 115200, timeout: float = 0.05) -> None:
        self.disconnect()
        self._serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=timeout)
        time.sleep(1.8)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        self._buffer = ""
        self.last_error = None

    def disconnect(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None
        self._buffer = ""

    def send_line(self, line: str) -> None:
        if not self.connected:
            raise RuntimeError("Not connected to the rover")
        payload = line if line.endswith("\n") else f"{line}\n"
        assert self._serial is not None
        self._serial.write(payload.encode("utf-8"))
        self._serial.flush()

    def read_lines(self, max_lines: int = 200) -> List[str]:
        if not self.connected:
            return []
        assert self._serial is not None
        waiting = self._serial.in_waiting
        if waiting:
            chunk = self._serial.read(waiting).decode("utf-8", errors="ignore")
            self._buffer += chunk

        lines: List[str] = []
        while "\n" in self._buffer and len(lines) < max_lines:
            line, self._buffer = self._buffer.split("\n", 1)
            cleaned = line.strip("\r")
            if cleaned:
                lines.append(cleaned)

        return lines