#!/usr/bin/env python3
"""
Live-Visualisierung des MPU6050 am ESP32.

Liest die serielle Schnittstelle (115200 Baud) und zeigt ein kleines Auto,
das Lage und Beschleunigung des Modellautos folgt.

Montage: Sensor-Y von vorne nach hinten durch das Auto (Heck = +Y),
Z nach oben. Gegenueber der Standardlage um 90° in der Ebene gedreht.

  python3 visualize_car.py
  python3 visualize_car.py --port /dev/cu.usbserial-XXXX
  python3 visualize_car.py --list

Den PlatformIO-Serial-Monitor vorher schliessen.
Tasten: R Lage+Position nullen, Leertaste nur Position, X/Y/Z Achse invertieren.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import threading
import time
import tkinter as tk
from collections import deque

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("pyserial fehlt. Installieren mit:")
    print("  python3 -m pip install -r requirements.txt")
    sys.exit(1)


BAUD = 115200
WIDTH, HEIGHT = 1100, 700
ALPHA = 0.96
ACCEL_DEADZONE = 0.05  # g, unterdrueckt Rauschen
ACCEL_TO_UNITS = 16.0  # Sichtbewegung pro g
VEL_DAMP = 1.8  # 1/s, bremst die Fahrt
SPRING = 0.35  # zieht langsam zurueck, damit es im Bild bleibt
POS_LIMIT = 9.0
TRAIL_LEN = 90
ALT_TO_UNITS = 3.0  # 1 m relative Hoehe -> 3 Gittereinheiten

DATA_RE = re.compile(
    r"^DATA,(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),"
    r"(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)"
    r"(?:,(-?\d+\.?\d*))?"
)
ACC_RE = re.compile(
    r"Acc:\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+"
    r"Gyro:\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+"
    r"T:\s+(-?\d+\.?\d*)"
)


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def vec_norm(a):
    length = math.sqrt(vec_dot(a, a))
    if length < 1e-9:
        return [0.0, 0.0, 0.0]
    return vec_scale(a, 1.0 / length)


def mat_vec(m, v):
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]


def mat_mul(a, b):
    return [
        [a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] for j in range(3)]
        for i in range(3)
    ]


def rot_x(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def rot_y(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def rot_z(rad):
    c, s = math.cos(rad), math.sin(rad)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def orientation_matrix(roll_deg, pitch_deg, yaw_deg):
    return mat_mul(
        rot_z(math.radians(yaw_deg)),
        mat_mul(rot_y(math.radians(pitch_deg)), rot_x(math.radians(roll_deg))),
    )


def sensor_to_car(ax, ay, az, gx, gy, gz):
    """
    Sensor-Y laeuft von der Nase zum Heck (+Y = hinten), +Z nach oben.
    Auto-Koordinaten: +X nach vorne, +Y nach links, +Z nach oben.
    Entspricht 90° Drehung um Z:  x_auto = -y_sensor,  y_auto = x_sensor.
    """
    return ay, -ax, az, gy, -gx, gz


def rgb(color):
    return "#%02x%02x%02x" % color


def shade(color, amount):
    amount = max(0.22, min(1.0, amount))
    return tuple(min(255, int(c * amount)) for c in color)


def deadzone(value, limit):
    return 0.0 if abs(value) < limit else value


class ImuState:
    def __init__(self):
        self.lock = threading.Lock()
        self.ax = self.ay = self.az = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.temp = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.off_roll = self.off_pitch = self.off_yaw = 0.0
        self.inv_roll = self.inv_pitch = self.inv_yaw = 1.0
        self.px = self.py = 0.0
        self.vx = self.vy = 0.0
        self.lin_ax = self.lin_ay = 0.0
        self.alt = 0.0
        self.alt_off = 0.0
        self.trail = deque(maxlen=TRAIL_LEN)
        self.have_data = False
        self.hz = 0.0
        self.error = ""
        self.port = ""
        self._pkt_times = []

    def set_packet(self, ax, ay, az, gx, gy, gz, temp, alt=0.0):
        now = time.monotonic()
        ax, ay, az, gx, gy, gz = sensor_to_car(ax, ay, az, gx, gy, gz)
        with self.lock:
            self.ax, self.ay, self.az = ax, ay, az
            self.gx, self.gy, self.gz = gx, gy, gz
            self.temp = temp
            self.alt = alt
            self.have_data = True
            self._pkt_times.append(now)
            self._pkt_times = [t for t in self._pkt_times if now - t < 1.0]
            self.hz = float(len(self._pkt_times))

    def step(self, dt):
        with self.lock:
            if not self.have_data:
                return
            ax, ay, az = self.ax, self.ay, self.az
            gx = self.gx * self.inv_roll
            gy = self.gy * self.inv_pitch
            gz = self.gz * self.inv_yaw

            accel_roll = math.degrees(math.atan2(ay, az))
            accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            self.roll = ALPHA * (self.roll + gx * dt) + (1.0 - ALPHA) * accel_roll
            self.pitch = ALPHA * (self.pitch + gy * dt) + (1.0 - ALPHA) * accel_pitch
            self.yaw += gz * dt

            roll = math.radians((self.roll - self.off_roll) * self.inv_roll)
            pitch = math.radians((self.pitch - self.off_pitch) * self.inv_pitch)
            yaw = math.radians((self.yaw - self.off_yaw) * self.inv_yaw)

            # Schwerkraft abziehen. Spezifische Kraft zeigt entgegen der
            # Schieberichtung, deshalb fuer die Bewegung invertieren.
            g_x = -math.sin(pitch)
            g_y = math.sin(roll) * math.cos(pitch)
            lin_ax = deadzone(-(ax - g_x), ACCEL_DEADZONE)
            lin_ay = deadzone(-(ay - g_y), ACCEL_DEADZONE)
            self.lin_ax, self.lin_ay = lin_ax, lin_ay

            world = mat_vec(rot_z(yaw), [lin_ax, lin_ay, 0.0])
            self.vx += (world[0] * ACCEL_TO_UNITS - SPRING * self.px) * dt
            self.vy += (world[1] * ACCEL_TO_UNITS - SPRING * self.py) * dt
            damp = math.exp(-VEL_DAMP * dt)
            self.vx *= damp
            self.vy *= damp
            self.px += self.vx * dt
            self.py += self.vy * dt
            self.px = max(-POS_LIMIT, min(POS_LIMIT, self.px))
            self.py = max(-POS_LIMIT, min(POS_LIMIT, self.py))
            pz = (self.alt - self.alt_off) * ALT_TO_UNITS
            self.trail.append((self.px, self.py, pz))

    def pose(self):
        with self.lock:
            return (
                (self.roll - self.off_roll) * self.inv_roll,
                (self.pitch - self.off_pitch) * self.inv_pitch,
                (self.yaw - self.off_yaw) * self.inv_yaw,
                self.ax,
                self.ay,
                self.az,
                self.gx,
                self.gy,
                self.gz,
                self.temp,
                self.have_data,
                self.hz,
                self.error,
                self.port,
                self.px,
                self.py,
                (self.alt - self.alt_off) * ALT_TO_UNITS,
                self.alt - self.alt_off,
                self.lin_ax,
                self.lin_ay,
                list(self.trail),
            )

    def zero_pose(self):
        with self.lock:
            self.off_roll = self.roll
            self.off_pitch = self.pitch
            self.off_yaw = self.yaw

    def zero_position(self):
        with self.lock:
            self.px = self.py = 0.0
            self.vx = self.vy = 0.0
            self.alt_off = self.alt
            self.trail.clear()

    def zero(self):
        self.zero_pose()
        self.zero_position()

    def toggle_axis(self, axis):
        with self.lock:
            if axis == "x":
                self.inv_roll *= -1
            elif axis == "y":
                self.inv_pitch *= -1
            elif axis == "z":
                self.inv_yaw *= -1


def find_ports():
    scored = []
    keywords = (
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
    for port in list_ports.comports():
        blob = f"{port.device} {port.description} {port.manufacturer or ''}".lower()
        score = sum(1 for key in keywords if key in blob)
        if sys.platform == "darwin" and port.device.startswith("/dev/tty."):
            score -= 2
        scored.append((score, port))
    scored.sort(key=lambda item: item[0], reverse=True)
    return scored


def pick_port(explicit):
    if explicit:
        return explicit
    scored = find_ports()
    if not scored:
        return None
    return scored[0][1].device


def parse_line(line):
    match = DATA_RE.match(line.strip())
    if not match:
        match = ACC_RE.search(line)
    if not match:
        return None
    vals = tuple(float(match.group(i)) for i in range(1, 8))
    alt = float(match.group(8)) if match.lastindex >= 8 and match.group(8) else 0.0
    return vals + (alt,)


def serial_worker(port, baud, state: ImuState, stop_event: threading.Event):
    state.port = port
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as exc:
        state.error = str(exc)
        return

    try:
        ser.reset_input_buffer()
        buffer = ""
        while not stop_event.is_set():
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            buffer += chunk.decode("utf-8", errors="ignore")
            while "\n" in buffer:
                raw, buffer = buffer.split("\n", 1)
                parsed = parse_line(raw.replace("\r", ""))
                if parsed:
                    state.set_packet(*parsed)
    except serial.SerialException as exc:
        state.error = str(exc)
    finally:
        ser.close()


def box_mesh(center, size):
    cx, cy, cz = center
    hx, hy, hz = size[0] / 2, size[1] / 2, size[2] / 2
    corners = [
        [cx - hx, cy - hy, cz - hz],
        [cx + hx, cy - hy, cz - hz],
        [cx + hx, cy + hy, cz - hz],
        [cx - hx, cy + hy, cz - hz],
        [cx - hx, cy - hy, cz + hz],
        [cx + hx, cy - hy, cz + hz],
        [cx + hx, cy + hy, cz + hz],
        [cx - hx, cy + hy, cz + hz],
    ]
    faces = (
        (0, 1, 2, 3),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (2, 3, 7, 6),
        (1, 2, 6, 5),
        (0, 3, 7, 4),
    )
    return corners, faces


class Camera:
    def __init__(self):
        self.eye = [-10.5, 8.0, 6.8]
        self.target = [0.0, 0.0, 0.35]
        self.up = [0.0, 0.0, 1.0]
        self.fov = math.radians(42)

    def follow(self, px, py, pz, dt):
        desired_target = [px, py, pz + 0.35]
        desired_eye = [px - 10.5, py + 8.0, pz + 6.8]
        blend = min(1.0, 6.0 * dt)
        self.target = [
            self.target[i] + (desired_target[i] - self.target[i]) * blend
            for i in range(3)
        ]
        self.eye = [
            self.eye[i] + (desired_eye[i] - self.eye[i]) * blend for i in range(3)
        ]

    def project(self, point):
        forward = vec_norm(vec_sub(self.target, self.eye))
        right = vec_norm(vec_cross(forward, self.up))
        up = vec_cross(right, forward)
        rel = vec_sub(point, self.eye)
        x = vec_dot(rel, right)
        y = vec_dot(rel, up)
        z = vec_dot(rel, forward)
        if z < 0.25:
            z = 0.25
        scale = (HEIGHT / 2) / math.tan(self.fov / 2)
        sx = WIDTH / 2 + x * scale / z
        sy = HEIGHT / 2 + 20 - y * scale / z
        return sx, sy, z


def car_parts():
    # Modell: +X Nase (vorne), +Y links, +Z oben
    return [
        ((0.05, 0.0, 0.38), (3.4, 1.55, 0.42), (214, 48, 42)),
        ((-0.15, 0.0, 0.78), (1.7, 1.25, 0.46), (36, 48, 62)),
        ((0.25, 0.0, 0.86), (0.95, 1.05, 0.22), (150, 200, 230)),
        ((1.55, 0.0, 0.52), (0.45, 1.35, 0.18), (196, 40, 36)),
        ((-1.55, 0.0, 0.58), (0.35, 1.45, 0.12), (28, 28, 32)),
        ((1.05, 0.62, 0.48), (0.18, 0.12, 0.12), (255, 210, 70)),
        ((1.05, -0.62, 0.48), (0.18, 0.12, 0.12), (255, 210, 70)),
        ((1.15, 0.85, 0.28), (0.72, 0.32, 0.56), (22, 22, 24)),
        ((1.15, -0.85, 0.28), (0.72, 0.32, 0.56), (22, 22, 24)),
        ((-1.15, 0.85, 0.28), (0.72, 0.32, 0.56), (22, 22, 24)),
        ((-1.15, -0.85, 0.28), (0.72, 0.32, 0.56), (22, 22, 24)),
    ]


class Visualizer:
    def __init__(self, port, baud):
        self.state = ImuState()
        self.stop = threading.Event()
        self.camera = Camera()
        self.last_time = time.monotonic()
        self.light = vec_norm([0.35, 0.55, 0.85])

        self.root = tk.Tk()
        self.root.title("MPU6050 – RC-Auto Visualisierung")
        self.root.geometry(f"{WIDTH}x{HEIGHT}")
        self.root.configure(bg="#10141a")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.canvas = tk.Canvas(
            self.root, width=WIDTH, height=HEIGHT, bg="#10141a", highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.root.bind("<Escape>", lambda _e: self.close())
        self.root.bind("q", lambda _e: self.close())
        self.root.bind("r", lambda _e: self.state.zero())
        self.root.bind("<space>", lambda _e: self.state.zero_position())
        self.root.bind("x", lambda _e: self.state.toggle_axis("x"))
        self.root.bind("y", lambda _e: self.state.toggle_axis("y"))
        self.root.bind("z", lambda _e: self.state.toggle_axis("z"))

        self.thread = threading.Thread(
            target=serial_worker,
            args=(port, baud, self.state, self.stop),
            daemon=True,
        )
        self.thread.start()
        self.tick()

    def close(self):
        self.stop.set()
        self.root.destroy()

    def tick(self):
        now = time.monotonic()
        dt = min(0.05, now - self.last_time)
        self.last_time = now
        self.state.step(dt)
        pose = self.state.pose()
        self.camera.follow(pose[14], pose[15], pose[16], dt)
        self.redraw(pose)
        if not self.stop.is_set():
            self.root.after(16, self.tick)

    def redraw(self, pose):
        roll, pitch, yaw = pose[0], pose[1], pose[2]
        px, py, pz = pose[14], pose[15], pose[16]
        lin_ax, lin_ay = pose[18], pose[19]
        trail = pose[20]
        self.canvas.delete("all")
        self.canvas.create_rectangle(0, 0, WIDTH, HEIGHT * 0.42, fill="#18202c", outline="")
        self.draw_grid()
        self.draw_trail(trail)
        self.draw_car(roll, pitch, yaw, px, py, pz)
        self.draw_accel_arrow(roll, pitch, yaw, px, py, pz, lin_ax, lin_ay)
        self.draw_hud(pose)

    def draw_grid(self):
        for i in range(-10, 11):
            a = self.camera.project([-10.0, float(i), 0.0])
            b = self.camera.project([10.0, float(i), 0.0])
            self.canvas.create_line(a[0], a[1], b[0], b[1], fill="#303a46")
            a = self.camera.project([float(i), -10.0, 0.0])
            b = self.camera.project([float(i), 10.0, 0.0])
            self.canvas.create_line(a[0], a[1], b[0], b[1], fill="#303a46")
        origin = self.camera.project([0, 0, 0.02])
        tip = self.camera.project([2.2, 0, 0.02])
        self.canvas.create_line(origin[0], origin[1], tip[0], tip[1], fill="#3d5a73", width=2)

    def draw_trail(self, trail):
        if len(trail) < 2:
            return
        for i in range(1, len(trail)):
            a = self.camera.project([trail[i - 1][0], trail[i - 1][1], trail[i - 1][2] + 0.03])
            b = self.camera.project([trail[i][0], trail[i][1], trail[i][2] + 0.03])
            shade_amt = int(40 + 140 * (i / len(trail)))
            color = rgb((shade_amt, 80, 50))
            self.canvas.create_line(a[0], a[1], b[0], b[1], fill=color, width=3)

    def draw_car(self, roll, pitch, yaw, px, py, pz):
        rot = orientation_matrix(roll, pitch, yaw)
        origin = [px, py, pz]
        faces = []
        for center, size, color in car_parts():
            corners, indices = box_mesh(center, size)
            world = [vec_add(mat_vec(rot, c), origin) for c in corners]
            for idx in indices:
                pts = [world[i] for i in idx]
                projected = [self.camera.project(p) for p in pts]
                if any(p[2] < 0.3 for p in projected):
                    continue
                n = vec_norm(vec_cross(vec_sub(pts[1], pts[0]), vec_sub(pts[2], pts[0])))
                if vec_dot(n, vec_sub(self.camera.eye, pts[0])) <= 0:
                    n = vec_scale(n, -1)
                lit = shade(color, 0.35 + 0.75 * max(0.0, vec_dot(n, self.light)))
                avg_z = sum(p[2] for p in projected) / 4
                faces.append((avg_z, projected, lit))
        faces.sort(key=lambda item: item[0], reverse=True)
        for _, projected, color in faces:
            coords = []
            for p in projected:
                coords.extend((p[0], p[1]))
            self.canvas.create_polygon(
                *coords, fill=rgb(color), outline="#0c0e12", width=1
            )

    def draw_accel_arrow(self, roll, pitch, yaw, px, py, pz, lin_ax, lin_ay):
        mag = math.sqrt(lin_ax * lin_ax + lin_ay * lin_ay)
        if mag < 0.03:
            return
        rot = orientation_matrix(roll, pitch, yaw)
        start = [px, py, pz + 0.7]
        tip_body = [lin_ax * 3.2, lin_ay * 3.2, 0.7]
        tip = vec_add(mat_vec(rot, [tip_body[0], tip_body[1], 0.0]), start)
        a = self.camera.project(start)
        b = self.camera.project(tip)
        self.canvas.create_line(
            a[0], a[1], b[0], b[1], fill="#ffcc33", width=4, arrow=tk.LAST
        )

    def draw_hud(self, pose):
        roll, pitch, yaw, ax, ay, az, gx, gy, gz, temp, have, hz, error, port, px, py, pz, alt, lin_ax, lin_ay, _trail = pose
        self.canvas.create_rectangle(16, 16, 400, 340, fill="#080a0e", outline="#2a3340")
        if error:
            status = f"Fehler: {error}"
            status_color = "#ff6e5a"
        elif have:
            status = f"verbunden  {hz:.0f} Hz"
            status_color = "#5ad278"
        else:
            status = "warte auf DATA-Zeilen..."
            status_color = "#ffbe50"

        lines = [
            (f"Port: {port or '-'}", "#e6e6eb"),
            (status, status_color),
            (f"Acc  [g]     {ax:+6.2f}  {ay:+6.2f}  {az:+6.2f}", "#d2d2d8"),
            (f"Lin-a [g]    {lin_ax:+6.2f}  {lin_ay:+6.2f}", "#ffcc33"),
            (f"Gyro [deg/s] {gx:+6.1f}  {gy:+6.1f}  {gz:+6.1f}", "#d2d2d8"),
            (f"Roll {roll:+6.1f}  Pitch {pitch:+6.1f}  Yaw {yaw:+6.1f}", "#ffffff"),
            (f"Pos {px:+5.2f}  {py:+5.2f}   H {alt:+5.2f} m", "#d2d2d8"),
            (f"Temp {temp:4.1f} C  (BME280)", "#d2d2d8"),
        ]
        y = 38
        for text, color in lines:
            self.canvas.create_text(28, y, text=text, fill=color, anchor="w", font=("Menlo", 13))
            y += 36

        self.canvas.create_text(
            WIDTH - 24,
            28,
            text="Modellauto (MPU6050)",
            fill="#ebeef5",
            anchor="ne",
            font=("Menlo", 16, "bold"),
        )
        help_text = (
            "R  alles nullen   |   Leertaste Position   |   X/Y/Z invertieren   |   Esc Ende"
        )
        self.canvas.create_text(
            16, HEIGHT - 28, text=help_text, fill="#a0a8b2", anchor="w", font=("Menlo", 12)
        )

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description="MPU6050 Auto-Visualisierung")
    parser.add_argument("--port", help="Serielle Schnittstelle, z.B. /dev/cu.usbserial-0001")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--list", action="store_true", help="Verfuegbare Ports anzeigen")
    args = parser.parse_args()

    if args.list:
        scored = find_ports()
        if not scored:
            print("Keine seriellen Ports gefunden.")
            return
        print("Gefundene Ports:")
        for score, port in scored:
            mark = "*" if score > 0 else " "
            print(f" {mark} {port.device:40s}  {port.description}")
        return

    port = pick_port(args.port)
    if not port:
        print("Kein serieller Port gefunden. Mit --list pruefen oder --port angeben.")
        sys.exit(1)

    print(f"Oeffne {port} @ {args.baud} Baud")
    print("PlatformIO Serial Monitor muss geschlossen sein.")
    Visualizer(port, args.baud).run()


if __name__ == "__main__":
    main()
