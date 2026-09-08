"""3D-Bewegungsansicht des Autos, portiert aus visualize_car.py."""

from __future__ import annotations

import math
import time
from typing import Optional

from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QSizePolicy, QWidget

from src.imu import (
    Camera,
    ImuState,
    box_mesh,
    car_parts,
    mat_vec,
    orientation_matrix,
    shade,
    vec_add,
    vec_cross,
    vec_dot,
    vec_norm,
    vec_scale,
    vec_sub,
)
from src.protocol import Telemetry

SKY = QColor("#18202C")
BG = QColor("#10141A")
GRID = QColor("#303A46")
ACCENT = QColor("#FFCC33")


class MotionViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.imu = ImuState()
        self.camera = Camera()
        self.light = vec_norm([0.35, 0.55, 0.85])
        self._last = time.monotonic()
        self._mpu_ok = False
        self._cam_zoom = 1.0
        self.setMinimumSize(240, 200)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    def zoom_in(self):
        self._cam_zoom = max(0.35, self._cam_zoom * 0.8)

    def zoom_out(self):
        self._cam_zoom = min(2.8, self._cam_zoom * 1.25)

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.zoom_in()
        else:
            self.zoom_out()

    def push_telemetry(self, tel: Optional[Telemetry]):
        if tel is None:
            self._mpu_ok = False
            return
        self._mpu_ok = tel.mpu_ok()
        if self._mpu_ok:
            self.imu.set_packet(tel.ax, tel.ay, tel.az, tel.gx, tel.gy, tel.gz, tel.temp_c, tel.alt_rel_m)

    def zero_pose(self):
        self.imu.zero_pose()

    def zero_position(self):
        self.imu.zero_position()

    def zero_all(self):
        self.imu.zero()

    def toggle_axis(self, axis: str):
        self.imu.toggle_axis(axis)

    def _tick(self):
        now = time.monotonic()
        dt = min(0.05, now - self._last)
        self._last = now
        if self._mpu_ok:
            self.imu.step(dt)
        pose = self.imu.pose()
        self.camera.follow(pose[11], pose[12], pose[13], dt, zoom=self._cam_zoom)
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = max(1, self.width()), max(1, self.height())
        painter.fillRect(0, 0, w, int(h * 0.42), SKY)
        painter.fillRect(0, int(h * 0.42), w, h, BG)

        pose = self.imu.pose()
        roll, pitch, yaw = pose[0], pose[1], pose[2]
        px, py, pz = pose[11], pose[12], pose[13]
        lin_ax, lin_ay = pose[15], pose[16]
        trail = pose[17]
        have = pose[10]

        self._draw_grid(painter, w, h)
        self._draw_trail(painter, trail, w, h)
        self._draw_car(painter, roll, pitch, yaw, px, py, pz, w, h)
        self._draw_accel_arrow(painter, roll, pitch, yaw, px, py, pz, lin_ax, lin_ay, w, h)
        self._draw_title(painter, have)

    def _project(self, point, width, height):
        return self.camera.project(point, width, height)

    def _draw_grid(self, painter: QPainter, width, height):
        painter.setPen(QPen(GRID, 1))
        for i in range(-10, 11):
            a = self._project([-10.0, float(i), 0.0], width, height)
            b = self._project([10.0, float(i), 0.0], width, height)
            painter.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))
            a = self._project([float(i), -10.0, 0.0], width, height)
            b = self._project([float(i), 10.0, 0.0], width, height)
            painter.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))
        origin = self._project([0, 0, 0.02], width, height)
        tip = self._project([2.2, 0, 0.02], width, height)
        painter.setPen(QPen(QColor("#3D5A73"), 2))
        painter.drawLine(QPointF(origin[0], origin[1]), QPointF(tip[0], tip[1]))

    def _draw_trail(self, painter: QPainter, trail, width, height):
        if len(trail) < 2:
            return
        for i in range(1, len(trail)):
            a = self._project([trail[i - 1][0], trail[i - 1][1], trail[i - 1][2] + 0.03], width, height)
            b = self._project([trail[i][0], trail[i][1], trail[i][2] + 0.03], width, height)
            shade_amt = int(40 + 140 * (i / len(trail)))
            painter.setPen(QPen(QColor(shade_amt, 80, 50), 3))
            painter.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))

    def _draw_car(self, painter, roll, pitch, yaw, px, py, pz, width, height):
        rot = orientation_matrix(roll, pitch, yaw)
        origin = [px, py, pz]
        faces = []
        for center, size, color in car_parts():
            corners, indices = box_mesh(center, size)
            world = [vec_add(mat_vec(rot, c), origin) for c in corners]
            for idx in indices:
                pts = [world[i] for i in idx]
                projected = [self._project(p, width, height) for p in pts]
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
            poly = QPolygonF([QPointF(p[0], p[1]) for p in projected])
            painter.setBrush(QColor(*color))
            painter.setPen(QPen(QColor("#0C0E12"), 1))
            painter.drawPolygon(poly)

    def _draw_accel_arrow(self, painter, roll, pitch, yaw, px, py, pz, lin_ax, lin_ay, width, height):
        mag = math.sqrt(lin_ax * lin_ax + lin_ay * lin_ay)
        if mag < 0.03:
            return
        rot = orientation_matrix(roll, pitch, yaw)
        start = [px, py, pz + 0.7]
        tip = vec_add(mat_vec(rot, [lin_ax * 3.2, lin_ay * 3.2, 0.0]), start)
        a = self._project(start, width, height)
        b = self._project(tip, width, height)
        painter.setPen(QPen(ACCENT, 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(QPointF(a[0], a[1]), QPointF(b[0], b[1]))
        painter.setBrush(ACCENT)
        painter.setPen(Qt.PenStyle.NoPen)
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy) or 1.0
        ux, uy = dx / length, dy / length
        px_, py_ = -uy, ux
        poly = QPolygonF(
            [
                QPointF(b[0], b[1]),
                QPointF(b[0] - ux * 12 + px_ * 5, b[1] - uy * 12 + py_ * 5),
                QPointF(b[0] - ux * 12 - px_ * 5, b[1] - uy * 12 - py_ * 5),
            ]
        )
        painter.drawPolygon(poly)

    def _draw_title(self, painter: QPainter, have: bool):
        painter.setPen(QColor("#A8B0BD"))
        painter.setFont(QFont("Helvetica Neue", 10, QFont.Weight.DemiBold))
        painter.drawText(16, 22, "LAGE IM RAUM")
        if not have:
            painter.setPen(QColor("#FFBE50"))
            painter.setFont(QFont("Helvetica Neue", 12))
            painter.drawText(16, 44, "warte auf MPU-Daten…")
        elif not self._mpu_ok:
            painter.setPen(QColor("#FF6E5A"))
            painter.setFont(QFont("Helvetica Neue", 12))
            painter.drawText(16, 44, "MPU ungültig")
