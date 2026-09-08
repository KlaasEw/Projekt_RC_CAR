"""Draufsicht: Auto, Lenkung, Lidar-Sektoren."""

from __future__ import annotations

import math
from typing import Optional

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import (
    QColor,
    QFont,
    QPainter,
    QPainterPath,
    QPen,
    QRadialGradient,
)
from PySide6.QtWidgets import QSizePolicy, QWidget

from src.protocol import SECTOR_ANGLES, SECTOR_NAMES, Telemetry, danger_label

BG = QColor("#0C1118")
GRID = QColor("#243040")
RING_TEXT = QColor("#6A7380")
BODY = QColor("#D6302A")
BODY_DARK = QColor("#8E1E1C")
CABIN = QColor("#2A3648")
WHEEL = QColor("#1A1C20")
WHEEL_RIM = QColor("#3A4048")
ACCENT = QColor("#5AD2FF")
WISH = QColor("#FFBE50")
TEXT = QColor("#E8EEF5")
MUTED = QColor("#8A93A3")

DANGER_FILL = {
    0: QColor(90, 210, 120, 70),
    1: QColor(255, 190, 80, 90),
    2: QColor(255, 140, 60, 110),
    3: QColor(255, 110, 90, 140),
}
DANGER_EDGE = {
    0: QColor("#5AD278"),
    1: QColor("#FFBE50"),
    2: QColor("#FF8C3C"),
    3: QColor("#FF6E5A"),
}

VIEW_M_DEFAULT = 5.0
VIEW_M_MIN = 1.0
VIEW_M_MAX = 12.0
ZOOM_IN = 0.8
ZOOM_OUT = 1.25


def _polar(cx: float, cy: float, angle_deg: float, radius: float) -> QPointF:
    rad = math.radians(angle_deg)
    return QPointF(cx + radius * math.sin(rad), cy - radius * math.cos(rad))


def _sector_angles(a0: float, a1: float, steps: int = 24) -> list[float]:
    if a1 < a0:
        a1 += 360.0
    span = a1 - a0
    return [((a0 + span * i / steps) % 360.0) for i in range(steps + 1)]


class TopDownWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self._tel: Optional[Telemetry] = None
        self._view_m = VIEW_M_DEFAULT
        self.setMinimumSize(240, 200)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setAutoFillBackground(False)

    def zoom_in(self):
        self._view_m = max(VIEW_M_MIN, self._view_m * ZOOM_IN)
        self.update()

    def zoom_out(self):
        self._view_m = min(VIEW_M_MAX, self._view_m * ZOOM_OUT)
        self.update()

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.zoom_in()
        else:
            self.zoom_out()

    def set_telemetry(self, tel: Optional[Telemetry]):
        self._tel = tel
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, BG)

        margin = 28
        cx, cy = w / 2, h / 2 + 8
        radius = max(80.0, min(w, h) / 2 - margin)
        self._draw_radar(painter, cx, cy, radius)
        self._draw_sectors(painter, cx, cy, radius)
        self._draw_car(painter, cx, cy, radius)
        self._draw_header(painter, w)

    def _draw_header(self, painter: QPainter, width: int):
        painter.setPen(MUTED)
        painter.setFont(QFont("Helvetica Neue", 10, QFont.Weight.DemiBold))
        painter.drawText(16, 22, "DRAUFSICHT  ·  LIDAR")
        tel = self._tel
        if tel is None:
            painter.setPen(QColor("#FFBE50"))
            painter.setFont(QFont("Helvetica Neue", 12))
            painter.drawText(16, 44, "warte auf Telemetrie…")
            return
        used = SECTOR_NAMES[tel.used_sector_index()]
        painter.setPen(ACCENT)
        painter.setFont(QFont("Menlo", 11))
        painter.drawText(
            16,
            44,
            f"genutzt: {used}   Stufe {tel.d_used} {danger_label(tel.d_used)}",
        )

    def _draw_radar(self, painter: QPainter, cx, cy, radius):
        glow = QRadialGradient(cx, cy, radius)
        glow.setColorAt(0.0, QColor("#182230"))
        glow.setColorAt(1.0, QColor("#0C1118"))
        painter.setBrush(glow)
        painter.setPen(QPen(QColor("#2A3340"), 2))
        painter.drawEllipse(QPointF(cx, cy), radius, radius)

        painter.setPen(QPen(GRID, 1))
        painter.setFont(QFont("Menlo", 9))
        for meters in (0.5, 1, 2, 3, 4, 5, 6, 8, 10, 12):
            if meters >= self._view_m:
                continue
            r = radius * (meters / self._view_m)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setPen(QPen(GRID, 1))
            painter.drawEllipse(QPointF(cx, cy), r, r)
            painter.setPen(RING_TEXT)
            label = f"{meters:g} m"
            painter.drawText(int(cx + 6), int(cy - r + 12), label)

        for angle in range(0, 360, 30):
            p = _polar(cx, cy, angle, radius)
            painter.setPen(QPen(GRID, 1))
            painter.drawLine(QPointF(cx, cy), p)

        painter.setPen(MUTED)
        painter.setFont(QFont("Menlo", 10))
        for angle, label in ((0, "0° vorne"), (90, "90°"), (180, "180°"), (270, "270°")):
            p = _polar(cx, cy, angle, radius * 1.06)
            painter.drawText(int(p.x() - 28), int(p.y() + 4), 56, 16, Qt.AlignmentFlag.AlignCenter, label)

    def _draw_sectors(self, painter: QPainter, cx, cy, radius):
        tel = self._tel
        if tel is None:
            return
        used = tel.used_sector_index()
        distances = tel.lidar_mm
        dangers = tel.danger
        for i, ((a0, a1), dist_mm, danger) in enumerate(zip(SECTOR_ANGLES, distances, dangers)):
            valid = dist_mm > 0
            dist_m = dist_mm / 1000.0 if valid else 0.0
            r = radius * min(dist_m / self._view_m, 1.0) if valid else radius * 0.18
            path = QPainterPath()
            path.moveTo(QPointF(cx, cy))
            for ang in _sector_angles(a0, a1):
                path.lineTo(_polar(cx, cy, ang, r))
            path.closeSubpath()

            fill = DANGER_FILL.get(danger, DANGER_FILL[0]) if valid else QColor(80, 90, 100, 40)
            edge = DANGER_EDGE.get(danger, DANGER_EDGE[0]) if valid else QColor("#4A5564")
            painter.setBrush(fill)
            if i == used:
                painter.setPen(QPen(ACCENT, 3.5))
            else:
                style = Qt.PenStyle.DashLine if not valid else Qt.PenStyle.SolidLine
                painter.setPen(QPen(edge, 1.8, style))
            painter.drawPath(path)

            mid = (a0 + ((a1 + 360 if a1 < a0 else a1) - a0) / 2.0) % 360.0
            mark = _polar(cx, cy, mid, radius * 0.90)
            self._draw_sector_mark(
                painter, mark, SECTOR_NAMES[i][0], edge if valid else MUTED, i == used
            )

        self._draw_used_badge(painter, tel)

    def _draw_car(self, painter: QPainter, cx, cy, radius):
        scale = radius / 9.5
        body_len = 3.4 * scale
        body_w = 1.55 * scale
        painter.save()
        painter.translate(cx, cy)

        tel = self._tel
        cmd = float(tel.cmd_servo) if tel else 0.0
        rx = float(tel.rx_servo) if tel else 0.0

        # Wunsch-Lenkung als Pfeil, wenn sie von cmd abweicht
        if tel and tel.cmd_servo != tel.rx_servo:
            self._draw_steer_arrow(painter, rx, body_len * 0.85, WISH, 2.0)
        self._draw_steer_arrow(painter, cmd, body_len * 0.72, ACCENT, 3.0)

        # Karosserie (Nase oben = -Y)
        painter.setPen(QPen(QColor("#0C0E12"), 1))
        painter.setBrush(BODY)
        painter.drawRoundedRect(
            QRectF(-body_w / 2, -body_len / 2, body_w, body_len), 6, 6
        )
        painter.setBrush(BODY_DARK)
        painter.drawRoundedRect(
            QRectF(-body_w * 0.38, -body_len * 0.08, body_w * 0.76, body_len * 0.42),
            4,
            4,
        )
        painter.setBrush(CABIN)
        painter.drawRoundedRect(
            QRectF(-body_w * 0.32, -body_len * 0.06, body_w * 0.64, body_len * 0.28),
            3,
            3,
        )
        # Nase
        nose = QPainterPath()
        nose.moveTo(0, -body_len / 2 - 4)
        nose.lineTo(-body_w * 0.28, -body_len * 0.28)
        nose.lineTo(body_w * 0.28, -body_len * 0.28)
        nose.closeSubpath()
        painter.setBrush(QColor("#F04A40"))
        painter.drawPath(nose)

        wheel_w, wheel_h = 0.32 * scale, 0.72 * scale
        axles_y = (-body_len * 0.32, body_len * 0.32)
        axles_x = body_w * 0.62
        for i, wy in enumerate(axles_y):
            steer = cmd if i == 0 else 0.0
            for wx in (-axles_x, axles_x):
                painter.save()
                painter.translate(wx, wy)
                painter.rotate(steer)
                painter.setBrush(WHEEL)
                painter.setPen(QPen(WHEEL_RIM, 1))
                painter.drawRoundedRect(
                    QRectF(-wheel_w / 2, -wheel_h / 2, wheel_w, wheel_h), 3, 3
                )
                painter.restore()

        painter.restore()

    def _draw_sector_mark(self, painter: QPainter, pos: QPointF, letter: str, color: QColor, used: bool):
        r = 11.0
        painter.setBrush(QColor(8, 10, 14, 230))
        painter.setPen(QPen(ACCENT if used else color, 2.2 if used else 1.4))
        painter.drawEllipse(pos, r, r)
        painter.setPen(TEXT)
        painter.setFont(QFont("Menlo", 10, QFont.Weight.Bold))
        painter.drawText(
            QRectF(pos.x() - r, pos.y() - r, r * 2, r * 2),
            Qt.AlignmentFlag.AlignCenter,
            letter,
        )

    def _draw_used_badge(self, painter: QPainter, tel: Telemetry):
        painter.setFont(QFont("Menlo", 10, QFont.Weight.Bold))
        badge = f"{SECTOR_NAMES[tel.used_sector_index()]} · {danger_label(tel.d_used)}"
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(8, 10, 14, 210))
        rect = QRectF(self.width() - 188, 12, 176, 28)
        painter.drawRoundedRect(rect, 8, 8)
        painter.setPen(DANGER_EDGE.get(tel.d_used, ACCENT))
        painter.drawText(rect, Qt.AlignmentFlag.AlignCenter, badge)

    def _draw_steer_arrow(self, painter: QPainter, servo_deg: float, length: float, color: QColor, width: float):
        painter.save()
        painter.rotate(servo_deg)
        painter.setPen(QPen(color, width, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(QPointF(0, 0), QPointF(0, -length))
        tip = QPainterPath()
        tip.moveTo(0, -length - 6)
        tip.lineTo(-6, -length + 8)
        tip.lineTo(6, -length + 8)
        tip.closeSubpath()
        painter.setBrush(color)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawPath(tip)
        painter.restore()
