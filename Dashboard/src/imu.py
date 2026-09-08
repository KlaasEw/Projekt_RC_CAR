"""Komplementärfilter und Positionsintegration aus visualize_car.py."""

from __future__ import annotations

import math
import threading
from collections import deque

ALPHA = 0.96
ACCEL_DEADZONE = 0.05
ACCEL_TO_UNITS = 16.0
VEL_DAMP = 1.8
SPRING = 0.35
POS_LIMIT = 9.0
TRAIL_LEN = 90
ALT_TO_UNITS = 3.0


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
    """
    return ay, -ax, az, gy, -gx, gz


def shade(color, amount):
    amount = max(0.22, min(1.0, amount))
    return tuple(min(255, int(c * amount)) for c in color)


def deadzone(value, limit):
    return 0.0 if abs(value) < limit else value


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


class Camera:
    def __init__(self):
        self.eye = [-10.5, 8.0, 6.8]
        self.target = [0.0, 0.0, 0.35]
        self.up = [0.0, 0.0, 1.0]
        self.fov = math.radians(42)

    def follow(self, px, py, pz, dt, zoom=1.0):
        desired_target = [px, py, pz + 0.35]
        desired_eye = [px - 10.5 * zoom, py + 8.0 * zoom, pz + 6.8 * zoom]
        blend = min(1.0, 6.0 * dt)
        self.target = [
            self.target[i] + (desired_target[i] - self.target[i]) * blend
            for i in range(3)
        ]
        self.eye = [
            self.eye[i] + (desired_eye[i] - self.eye[i]) * blend for i in range(3)
        ]

    def project(self, point, width, height):
        forward = vec_norm(vec_sub(self.target, self.eye))
        right = vec_norm(vec_cross(forward, self.up))
        up = vec_cross(right, forward)
        rel = vec_sub(point, self.eye)
        x = vec_dot(rel, right)
        y = vec_dot(rel, up)
        z = vec_dot(rel, forward)
        if z < 0.25:
            z = 0.25
        scale = (height / 2) / math.tan(self.fov / 2)
        sx = width / 2 + x * scale / z
        sy = height / 2 + 20 - y * scale / z
        return sx, sy, z


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

    def set_packet(self, ax, ay, az, gx, gy, gz, temp, alt=0.0):
        ax, ay, az, gx, gy, gz = sensor_to_car(ax, ay, az, gx, gy, gz)
        with self.lock:
            self.ax, self.ay, self.az = ax, ay, az
            self.gx, self.gy, self.gz = gx, gy, gz
            self.temp = temp
            self.alt = alt
            self.have_data = True

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
