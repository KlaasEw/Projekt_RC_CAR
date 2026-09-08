"""Serielle Schnittstelle: Portliste und Lesethread für TEL-CSV."""

from __future__ import annotations

import sys
import time

from PySide6.QtCore import QThread, Signal

from src.protocol import parse_tel

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "pyserial fehlt. Installieren mit:\n  python3 -m pip install -r requirements.txt"
    ) from exc

BAUD = 115200

_PORT_KEYWORDS = (
    "usbserial",
    "usbmodem",
    "wchusb",
    "cp210",
    "ch340",
    "ch910",
    "slab",
    "silicon",
    "uart",
    "esp32",
    "usb-serial",
)
_SKIP = ("bluetooth", "debug-console", "incoming", "soundcore", "airpod", "headset")
_USB_HINTS = ("usb", "serial", "slab", "cp210", "wch", "uart", "ch340", "ch910", "esp")


def find_ports():
    scored = []
    for port in list_ports.comports():
        blob = f"{port.device} {port.description} {port.manufacturer or ''}".lower()
        if any(word in blob for word in _SKIP):
            continue
        if sys.platform == "darwin" and port.device.startswith("/dev/tty."):
            continue
        if not any(word in blob for word in _USB_HINTS) and "cu." not in port.device:
            continue
        score = sum(1 for key in _PORT_KEYWORDS if key in blob)
        if "cp210" in blob or "silicon" in blob or "slab" in blob:
            score += 2
        if "usbserial" in blob or "usbmodem" in blob:
            score += 1
        scored.append((score, port))
    scored.sort(key=lambda item: item[0], reverse=True)
    return scored


def port_choices() -> list[tuple[str, str]]:
    """(device, label) für das Dropdown."""
    items = []
    for score, port in find_ports():
        extra = f" — {port.description}" if port.description else ""
        items.append((port.device, f"{port.device}{extra}"))
    return items


class SerialThread(QThread):
    packet = Signal(object)
    stats = Signal(float, int)  # Hz, seq
    status = Signal(str)
    error = Signal(str)

    def __init__(self):
        super().__init__()
        self.port = ""
        self.baud = BAUD
        self._running = False

    def start_port(self, port: str, baud: int = BAUD):
        self.stop_port()
        self.port = port
        self.baud = baud
        self._running = True
        self.start()

    def stop_port(self):
        self._running = False
        if self.isRunning():
            self.wait(2000)

    def run(self):
        ser = None
        pkt_times: list[float] = []
        last_seq = None
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.2)
            ser.reset_input_buffer()
            self.status.emit(f"verbunden: {self.port}")
            buffer = ""
            while self._running:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                buffer += chunk.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    raw, buffer = buffer.split("\n", 1)
                    tel = parse_tel(raw)
                    if tel is None:
                        continue
                    now = time.monotonic()
                    pkt_times.append(now)
                    pkt_times = [t for t in pkt_times if now - t < 1.0]
                    last_seq = tel.seq
                    self.packet.emit(tel)
                    self.stats.emit(float(len(pkt_times)), last_seq)
        except serial.SerialException as exc:
            self.error.emit(str(exc))
        except OSError as exc:
            self.error.emit(str(exc))
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
            self.status.emit("getrennt")
