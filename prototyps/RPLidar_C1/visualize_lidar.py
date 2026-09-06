#!/usr/bin/env python3
"""
Live-Visualisierung des Slamtec RPLIDAR C1 am Mac (USB).

Der Sensor spricht binaer ueber UART @ 460800 Baud. Dieses Skript findet den
USB-Adapter, startet den Scan und zeichnet jeden 360-Grad-Umlauf als Radar.

  python3 visualize_lidar.py
  python3 visualize_lidar.py --port /dev/cu.usbserial-XXXX
  python3 visualize_lidar.py --list

Tasten: +/- Zoom, R Ansicht zuruecksetzen, M Min-Distanz markieren, Esc Ende.
Mausrad zoomt ebenfalls. Den Serial-Monitor anderer Programme vorher schliessen.
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import time
import tkinter as tk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial fehlt. Installieren mit:")
    print("  python3 -m pip install -r requirements.txt")
    sys.exit(1)


BAUD = 460800
SYNC = 0xA5
SYNC2 = 0x5A
CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
CMD_MOTOR_PWM = 0xF0
CMD_MOTOR_RPM = 0xA8

MIN_RANGE_M = 0.05
MAX_RANGE_M = 12.0
DEFAULT_VIEW_M = 8.0
MOTOR_RPM = 600
MOTOR_PWM = 660

WIDTH, HEIGHT = 1100, 720
RADAR_MARGIN = 36


def dist_color(distance_m: float, view_m: float) -> str:
    """Nahe Punkte gruen, ferne rot."""
    t = max(0.0, min(1.0, distance_m / max(view_m, 0.5)))
    if t < 0.5:
        r = int(40 + 430 * t)
        g = 210
        b = 70
    else:
        u = (t - 0.5) * 2.0
        r = 255
        g = int(210 * (1.0 - u))
        b = int(70 * (1.0 - u))
    return "#%02x%02x%02x" % (min(255, r), max(0, g), max(0, b))


class ScanState:
    def __init__(self):
        self.lock = threading.Lock()
        self.points: list[tuple[float, float, int]] = []
        self.hz = 0.0
        self.point_count = 0
        self.min_dist = 0.0
        self.min_angle = 0.0
        self.max_dist = 0.0
        self.have_data = False
        self.error = ""
        self.port = ""
        self.info = ""
        self.health = ""
        self._scan_times: list[float] = []

    def set_scan(self, points: list[tuple[float, float, int]]):
        now = time.monotonic()
        valid = [
            (angle, dist, quality)
            for angle, dist, quality in points
            if MIN_RANGE_M <= dist <= MAX_RANGE_M and quality > 0
        ]
        with self.lock:
            self.points = valid
            self.point_count = len(valid)
            self.have_data = True
            self._scan_times.append(now)
            self._scan_times = [t for t in self._scan_times if now - t < 1.0]
            self.hz = float(len(self._scan_times))
            if valid:
                nearest = min(valid, key=lambda p: p[1])
                farthest = max(valid, key=lambda p: p[1])
                self.min_angle, self.min_dist = nearest[0], nearest[1]
                self.max_dist = farthest[1]
            else:
                self.min_dist = self.max_dist = self.min_angle = 0.0

    def snapshot(self):
        with self.lock:
            return (
                list(self.points),
                self.hz,
                self.point_count,
                self.min_dist,
                self.min_angle,
                self.max_dist,
                self.have_data,
                self.error,
                self.port,
                self.info,
                self.health,
            )


def find_ports():
    scored = []
    keywords = (
        "cp210",
        "slab",
        "silicon",
        "usbserial",
        "usbmodem",
        "wchusb",
        "ch340",
        "ch910",
        "uart",
        "usb-serial",
    )
    skip = ("bluetooth", "debug-console", "incoming", "soundcore", "airpod", "headset")
    usb_hints = ("usb", "serial", "slab", "cp210", "wch", "uart", "ch340", "ch910")
    for port in list_ports.comports():
        blob = f"{port.device} {port.description} {port.manufacturer or ''}".lower()
        if any(word in blob for word in skip):
            continue
        if not any(word in blob for word in usb_hints):
            continue
        score = sum(1 for key in keywords if key in blob)
        if "cp210" in blob or "silicon" in blob or "slab" in blob:
            score += 3
        if sys.platform == "darwin" and port.device.startswith("/dev/tty."):
            score -= 2
        scored.append((score, port))
    scored.sort(key=lambda item: item[0], reverse=True)
    return scored


def pick_port(explicit: str | None) -> str | None:
    if explicit:
        return explicit
    scored = find_ports()
    if not scored:
        return None
    return scored[0][1].device


def send_cmd(ser: serial.Serial, cmd: int, payload: bytes = b""):
    if payload:
        frame = bytes([SYNC, cmd, len(payload)]) + payload
        checksum = 0
        for byte in frame:
            checksum ^= byte
        ser.write(frame + bytes([checksum]))
    else:
        ser.write(bytes([SYNC, cmd]))
    ser.flush()


def read_descriptor(ser: serial.Serial, leftover: bytearray, timeout: float = 3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            leftover.extend(chunk)
        idx = leftover.find(bytes([SYNC, SYNC2]))
        if idx >= 0 and len(leftover) >= idx + 7:
            desc = bytes(leftover[idx : idx + 7])
            del leftover[: idx + 7]
            size = desc[2] | (desc[3] << 8) | (desc[4] << 16) | ((desc[5] & 0x3F) << 24)
            return size, desc[6], leftover
        time.sleep(0.005)
    raise TimeoutError("Keine Antwort vom Lidar (Descriptor fehlt).")


def read_exact(ser: serial.Serial, leftover: bytearray, size: int, timeout: float = 2.0):
    deadline = time.monotonic() + timeout
    while len(leftover) < size:
        if time.monotonic() > deadline:
            raise TimeoutError("Unvollstaendige Antwort vom Lidar.")
        chunk = ser.read(max(1, min(ser.in_waiting or 1, size - len(leftover))))
        if chunk:
            leftover.extend(chunk)
        else:
            time.sleep(0.005)
    data = bytes(leftover[:size])
    del leftover[:size]
    return data


def parse_info(data: bytes) -> str:
    model, fw_minor, fw_major, hardware = data[0], data[1], data[2], data[3]
    serial_hex = data[4:20].hex().upper()
    return (
        f"Modell 0x{model:02X}  FW {fw_major}.{fw_minor}  "
        f"HW {hardware}  SN {serial_hex[:12]}..."
    )


def parse_health(data: bytes) -> str:
    status = data[0]
    error_code = data[1] | (data[2] << 8)
    labels = {0: "OK", 1: "Warnung", 2: "Schutzstopp"}
    text = labels.get(status, f"Status {status}")
    if error_code:
        text += f"  Code {error_code}"
    return text


def parse_node(packet: bytes):
    s_flag = packet[0] & 0x01
    inv_s = (packet[0] >> 1) & 0x01
    check = packet[1] & 0x01
    if s_flag == inv_s or check != 1:
        return None
    quality = packet[0] >> 2
    angle = ((packet[1] >> 1) | (packet[2] << 7)) / 64.0
    distance_mm = (packet[3] | (packet[4] << 8)) / 4.0
    return s_flag == 1, angle % 360.0, distance_mm / 1000.0, quality


def start_lidar(ser: serial.Serial, state: ScanState, rpm: int, pwm: int):
    leftover = bytearray()

    send_cmd(ser, CMD_STOP)
    time.sleep(0.05)
    send_cmd(ser, CMD_RESET)
    time.sleep(2.0)
    ser.reset_input_buffer()
    leftover.clear()

    send_cmd(ser, CMD_GET_INFO)
    size, dtype, leftover = read_descriptor(ser, leftover)
    if dtype != 0x04:
        raise RuntimeError(f"Unerwartete Info-Antwort (Typ 0x{dtype:02X}).")
    info = parse_info(read_exact(ser, leftover, size))
    with state.lock:
        state.info = info

    send_cmd(ser, CMD_GET_HEALTH)
    size, dtype, leftover = read_descriptor(ser, leftover)
    if dtype != 0x06:
        raise RuntimeError(f"Unerwartete Health-Antwort (Typ 0x{dtype:02X}).")
    health_raw = read_exact(ser, leftover, size)
    health = parse_health(health_raw)
    with state.lock:
        state.health = health
    if health_raw[0] == 2:
        raise RuntimeError(
            "Lidar im Schutzstopp. Strom trennen, Sensor pruefen, erneut starten."
        )

    # C-Serie: Drehzahl in RPM. A-Serie/USB-Adapter: PWM. Beides senden ist harmlos.
    send_cmd(ser, CMD_MOTOR_RPM, struct.pack("<H", rpm))
    send_cmd(ser, CMD_MOTOR_PWM, struct.pack("<H", pwm))
    time.sleep(1.2)
    ser.reset_input_buffer()
    leftover.clear()

    send_cmd(ser, CMD_SCAN)
    size, dtype, leftover = read_descriptor(ser, leftover, timeout=4.0)
    if size != 5:
        raise RuntimeError(f"Unerwartete Scan-Paketgroesse ({size} statt 5).")
    return leftover


def serial_worker(
    port: str,
    baud: int,
    rpm: int,
    pwm: int,
    state: ScanState,
    stop_event: threading.Event,
):
    state.port = port
    ser = None
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            dsrdtr=False,
            rtscts=False,
        )
        ser.dtr = False
        ser.rts = False
        leftover = start_lidar(ser, state, rpm, pwm)
        scan: list[tuple[float, float, int]] = []
        buf = leftover

        while not stop_event.is_set():
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf.extend(chunk)
            while len(buf) >= 5:
                node = parse_node(bytes(buf[:5]))
                if node is None:
                    del buf[0]
                    continue
                del buf[:5]
                is_new, angle, dist, quality = node
                if is_new and scan:
                    state.set_scan(scan)
                    scan = []
                scan.append((angle, dist, quality))
            if not chunk:
                time.sleep(0.002)
    except (serial.SerialException, TimeoutError, RuntimeError, OSError) as exc:
        with state.lock:
            state.error = str(exc)
    finally:
        if ser is not None:
            try:
                send_cmd(ser, CMD_STOP)
                time.sleep(0.02)
                send_cmd(ser, CMD_MOTOR_RPM, struct.pack("<H", 0))
                send_cmd(ser, CMD_MOTOR_PWM, struct.pack("<H", 0))
            except Exception:
                pass
            ser.close()


class Visualizer:
    def __init__(self, port: str, baud: int, rpm: int, pwm: int, view_m: float):
        self.state = ScanState()
        self.stop = threading.Event()
        self.view_m = view_m
        self.show_min = True

        self.root = tk.Tk()
        self.root.title("RPLIDAR C1 – Live-Scan")
        self.root.geometry(f"{WIDTH}x{HEIGHT}")
        self.root.configure(bg="#10141a")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.canvas = tk.Canvas(
            self.root, width=WIDTH, height=HEIGHT, bg="#10141a", highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.root.bind("<Escape>", lambda _e: self.close())
        self.root.bind("q", lambda _e: self.close())
        self.root.bind("+", lambda _e: self.zoom(0.8))
        self.root.bind("=", lambda _e: self.zoom(0.8))
        self.root.bind("-", lambda _e: self.zoom(1.25))
        self.root.bind("r", lambda _e: self.reset_view())
        self.root.bind("m", lambda _e: self.toggle_min())
        self.canvas.bind("<MouseWheel>", self.on_wheel)
        self.canvas.bind("<Button-4>", lambda _e: self.zoom(0.8))
        self.canvas.bind("<Button-5>", lambda _e: self.zoom(1.25))

        self.thread = threading.Thread(
            target=serial_worker,
            args=(port, baud, rpm, pwm, self.state, self.stop),
            daemon=True,
        )
        self.thread.start()
        self.tick()

    def run(self):
        self.root.mainloop()

    def zoom(self, factor: float):
        self.view_m = max(1.0, min(MAX_RANGE_M, self.view_m * factor))

    def reset_view(self):
        self.view_m = DEFAULT_VIEW_M

    def toggle_min(self):
        self.show_min = not self.show_min

    def on_wheel(self, event):
        self.zoom(0.8 if event.delta > 0 else 1.25)

    def close(self):
        self.stop.set()
        self.root.destroy()

    def tick(self):
        self.redraw(self.state.snapshot())
        if not self.stop.is_set():
            self.root.after(33, self.tick)

    def radar_geom(self):
        width = self.canvas.winfo_width() or WIDTH
        height = self.canvas.winfo_height() or HEIGHT
        left_hud = 390
        cx = left_hud + (width - left_hud) / 2
        cy = height / 2 - 8
        radius = max(80.0, min(cx - left_hud, cy - RADAR_MARGIN, height - cy - 50) - 8)
        return width, height, cx, cy, radius

    def polar_to_screen(self, angle_deg: float, dist_m: float, cx, cy, radius):
        scale = radius / self.view_m
        rad = math.radians(angle_deg)
        x = cx + dist_m * math.sin(rad) * scale
        y = cy - dist_m * math.cos(rad) * scale
        return x, y

    def redraw(self, snap):
        points, hz, count, min_dist, min_angle, max_dist, have, error, port, info, health = snap
        width, height, cx, cy, radius = self.radar_geom()
        self.canvas.delete("all")
        self.draw_radar_background(cx, cy, radius)
        self.draw_scan_shape(points, cx, cy, radius)
        self.draw_radar_grid(cx, cy, radius)
        self.draw_points(points, cx, cy, radius)
        if self.show_min and have and min_dist > 0:
            self.draw_min_marker(min_angle, min_dist, cx, cy, radius)
        self.draw_sensor_marker(cx, cy)
        self.draw_hud(
            width, height, points, hz, count, min_dist, min_angle, max_dist, have, error, port, info, health
        )

    def draw_radar_background(self, cx, cy, radius):
        self.canvas.create_oval(
            cx - radius, cy - radius, cx + radius, cy + radius,
            fill="#0c1118", outline="#2a3340", width=2,
        )

    def draw_radar_grid(self, cx, cy, radius):
        for meters in (1, 2, 4, 6, 8, 10, 12):
            if meters >= self.view_m:
                continue
            r = radius * (meters / self.view_m)
            self.canvas.create_oval(
                cx - r, cy - r, cx + r, cy + r, outline="#2c3948"
            )
            self.canvas.create_text(
                cx + 8, cy - r + 10, text=f"{meters} m", fill="#6a7380",
                anchor="w", font=("Menlo", 10),
            )
        for angle in range(0, 360, 30):
            x, y = self.polar_to_screen(angle, self.view_m, cx, cy, radius)
            self.canvas.create_line(cx, cy, x, y, fill="#243040")
        for angle, label in ((0, "0° vorne"), (90, "90°"), (180, "180°"), (270, "270°")):
            x, y = self.polar_to_screen(angle, self.view_m * 1.06, cx, cy, radius)
            self.canvas.create_text(x, y, text=label, fill="#9aa3ad", font=("Menlo", 11))

    def draw_sensor_marker(self, cx, cy):
        self.canvas.create_polygon(
            cx, cy - 14, cx - 8, cy + 10, cx + 8, cy + 10,
            fill="#e8e8ee", outline="",
        )

    def scan_outline_coords(self, points, cx, cy, radius):
        if len(points) < 3:
            return []
        coords = []
        for angle, dist, _quality in sorted(points, key=lambda item: item[0]):
            x, y = self.polar_to_screen(angle, min(dist, self.view_m), cx, cy, radius)
            coords.extend((x, y))
        return coords

    def draw_scan_shape(self, points, cx, cy, radius):
        coords = self.scan_outline_coords(points, cx, cy, radius)
        if len(coords) < 6:
            return
        # Innenraum = sichtbarer Bereich, dahinter bleibt der dunkle Hintergrund.
        self.canvas.create_polygon(
            *coords,
            fill="#1c2a38",
            outline="#8ec8de",
            width=2,
            joinstyle=tk.ROUND,
        )

    def draw_points(self, points, cx, cy, radius):
        size = 3
        for angle, dist, _quality in points:
            if dist > self.view_m:
                continue
            x, y = self.polar_to_screen(angle, dist, cx, cy, radius)
            color = dist_color(dist, self.view_m)
            self.canvas.create_rectangle(
                x - size, y - size, x + size, y + size, fill=color, outline=""
            )

    def draw_min_marker(self, angle, dist, cx, cy, radius):
        x, y = self.polar_to_screen(angle, min(dist, self.view_m), cx, cy, radius)
        self.canvas.create_line(cx, cy, x, y, fill="#5ad2ff", width=2)
        self.canvas.create_oval(x - 6, y - 6, x + 6, y + 6, outline="#5ad2ff", width=2)
        self.canvas.create_text(
            x + 10, y - 10, text=f"{dist:.2f} m", fill="#5ad2ff",
            anchor="w", font=("Menlo", 12, "bold"),
        )

    def draw_hud(
        self, width, height, points, hz, count, min_dist, min_angle, max_dist,
        have, error, port, info, health,
    ):
        self.canvas.create_rectangle(16, 16, 372, 430, fill="#080a0e", outline="#2a3340")
        if error:
            status, status_color = f"Fehler: {error}", "#ff6e5a"
        elif have:
            status, status_color = f"Scan laeuft   {hz:.0f} Hz", "#5ad278"
        else:
            status, status_color = "verbinde / Motor hochfahren...", "#ffbe50"

        lines = [
            (f"Port: {port or '-'}", "#e6e6eb"),
            (status, status_color),
            (info or "Geraet: –", "#9aa3ad"),
            (f"Health: {health or '–'}", "#9aa3ad"),
            (f"Punkte   {count:4d}", "#d2d2d8"),
            (f"Naechstes Hindernis  {min_dist:5.2f} m  @ {min_angle:6.1f}°", "#5ad2ff"),
            (f"Weitester Punkt      {max_dist:5.2f} m", "#d2d2d8"),
            (f"Anzeige-Radius       {self.view_m:5.1f} m", "#d2d2d8"),
            ("C1: 0.05–12 m, ~10 U/s, 460800 Baud", "#6a7380"),
        ]
        y = 40
        for text, color in lines:
            self.canvas.create_text(
                28, y, text=text, fill=color, anchor="w", font=("Menlo", 12)
            )
            y += 40

        self.canvas.create_text(
            width - 24, 28, text="RPLIDAR C1", fill="#ebeef5",
            anchor="ne", font=("Menlo", 16, "bold"),
        )
        self.canvas.create_text(
            16, height - 28,
            text="+/- Zoom   |   M naechstes Hindernis   |   R Reset   |   Esc Ende",
            fill="#a0a8b2", anchor="w", font=("Menlo", 12),
        )


def main():
    parser = argparse.ArgumentParser(description="RPLIDAR C1 Live-Visualisierung")
    parser.add_argument("--port", help="Serielle Schnittstelle, z.B. /dev/cu.usbserial-0001")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--rpm", type=int, default=MOTOR_RPM, help="Motordrehzahl (C-Serie)")
    parser.add_argument("--pwm", type=int, default=MOTOR_PWM, help="Motor-PWM (USB-Adapter)")
    parser.add_argument("--range", type=float, default=DEFAULT_VIEW_M, dest="view_m")
    parser.add_argument("--list", action="store_true", help="Verfuegbare Ports anzeigen")
    args = parser.parse_args()

    if args.list:
        scored = find_ports()
        if not scored:
            print("Keine seriellen Ports gefunden. USB-Kabel und Adapter pruefen.")
            return
        print("Gefundene Ports (* = wahrscheinlich der C1):")
        for score, port in scored:
            mark = "*" if score > 0 else " "
            extra = f"  [{port.manufacturer}]" if port.manufacturer else ""
            print(f" {mark} {port.device:42s}  {port.description}{extra}")
        return

    port = pick_port(args.port)
    if not port:
        print("Kein serieller Port gefunden. Mit --list pruefen oder --port angeben.")
        sys.exit(1)

    print(f"Oeffne {port} @ {args.baud} Baud")
    print("Andere Serial-Monitore muessen geschlossen sein.")
    Visualizer(port, args.baud, args.rpm, args.pwm, args.view_m).run()


if __name__ == "__main__":
    main()
