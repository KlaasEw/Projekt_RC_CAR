"""Flag-Leiste mit Warnsymbolen je nach Bit-Polarität."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPainterPath, QPen
from PySide6.QtWidgets import QHBoxLayout, QSizePolicy, QWidget

from src.protocol import Telemetry, danger_flag_label

OK_BG = QColor("#163022")
OK_FG = QColor("#5AD278")
WARN_BG = QColor("#3A2410")
WARN_FG = QColor("#FFBE50")
ORANGE_BG = QColor("#3A2410")
ORANGE_FG = QColor("#FF8C3C")
ERR_BG = QColor("#3A1C1C")
ERR_FG = QColor("#FF6E5A")
IDLE_BG = QColor("#1A2230")
IDLE_FG = QColor("#6A7380")
TEXT = QColor("#E8EEF5")

LEVEL_COLORS = {
    0: (OK_BG, OK_FG),
    1: (WARN_BG, WARN_FG),
    2: (ORANGE_BG, ORANGE_FG),
    3: (ERR_BG, ERR_FG),
}


class FlagChip(QWidget):
    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        self._label = label
        self._ok = True
        self._active = False
        self._detail = ""
        self._level: Optional[int] = None
        self.setMinimumHeight(40)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setToolTip(label)

    def set_state(
        self,
        active: bool,
        warning: bool,
        detail: str = "",
        label: str | None = None,
        level: int | None = None,
    ):
        self._active = active
        self._level = level
        if level is not None:
            self._ok = level <= 0
        else:
            self._ok = not warning
        self._detail = detail
        if label is not None:
            self._label = label
        self.setToolTip(detail or self._label)
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        if not self._active:
            bg, fg = IDLE_BG, IDLE_FG
        elif self._level is not None:
            bg, fg = LEVEL_COLORS.get(self._level, LEVEL_COLORS[3])
        elif self._ok:
            bg, fg = OK_BG, OK_FG
        else:
            bg, fg = (ERR_BG, ERR_FG) if "Failsafe" in self._label or "iBUS" in self._label else (WARN_BG, WARN_FG)
            if "Failsafe" in self._label:
                bg, fg = ERR_BG, ERR_FG

        painter.setPen(QPen(fg, 1.2))
        painter.setBrush(bg)
        painter.drawRoundedRect(QRectF(1, 1, self.width() - 2, self.height() - 2), 10, 10)

        icon_x, icon_y = 12, self.height() / 2
        if not self._active:
            painter.setPen(QPen(fg, 2))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawEllipse(QRectF(icon_x - 7, icon_y - 7, 14, 14))
        elif self._ok:
            painter.setPen(QPen(fg, 2.4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap, Qt.PenJoinStyle.RoundJoin))
            path = QPainterPath()
            path.moveTo(icon_x - 5, icon_y + 1)
            path.lineTo(icon_x - 1, icon_y + 5)
            path.lineTo(icon_x + 6, icon_y - 5)
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawPath(path)
        else:
            path = QPainterPath()
            path.moveTo(icon_x, icon_y - 8)
            path.lineTo(icon_x + 8, icon_y + 7)
            path.lineTo(icon_x - 8, icon_y + 7)
            path.closeSubpath()
            painter.setBrush(fg)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawPath(path)
            painter.setPen(QPen(bg, 2))
            painter.drawLine(int(icon_x), int(icon_y - 3), int(icon_x), int(icon_y + 2))
            painter.drawPoint(int(icon_x), int(icon_y + 5))

        painter.setPen(TEXT if self._active else IDLE_FG)
        painter.setFont(QFont("Helvetica Neue", 11, QFont.Weight.DemiBold))
        painter.drawText(
            QRectF(28, 0, self.width() - 36, self.height()),
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
            self._label,
        )


class FlagBar(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        self.mpu = FlagChip("MPU")
        self.bme = FlagChip("BME")
        self.lidar = FlagChip("Lidar")
        self.vbat = FlagChip("Akku")
        self.ibus = FlagChip("iBUS")
        self.failsafe = FlagChip("Failsafe")
        self.danger = FlagChip("Gefahr: frei")
        for chip in (
            self.mpu,
            self.bme,
            self.lidar,
            self.vbat,
            self.ibus,
            self.failsafe,
            self.danger,
        ):
            layout.addWidget(chip)
        self.clear()

    def clear(self):
        for chip in (
            self.mpu,
            self.bme,
            self.lidar,
            self.vbat,
            self.ibus,
            self.failsafe,
            self.danger,
        ):
            chip.set_state(False, False, "keine Daten")
        self.danger.set_state(False, False, "keine Daten", label="Gefahr: frei", level=0)

    def set_telemetry(self, tel: Optional[Telemetry]):
        if tel is None:
            self.clear()
            return
        self.mpu.set_state(True, not tel.mpu_ok(), "MPU gültig" if tel.mpu_ok() else "MPU ungültig")
        self.bme.set_state(True, not tel.bme_ok(), "BME gültig" if tel.bme_ok() else "BME ungültig")
        self.lidar.set_state(True, not tel.lidar_ok(), "Lidar-Scan gültig" if tel.lidar_ok() else "Lidar ungültig")
        self.vbat.set_state(True, not tel.vbat_ok(), "Akkuspannung gültig" if tel.vbat_ok() else "Akkuspannung ungültig")
        self.ibus.set_state(True, not tel.ibus_ok(), "iBUS frisch" if tel.ibus_ok() else "kein frisches iBUS-Frame")
        self.failsafe.set_state(True, tel.failsafe(), "Failsafe aktiv" if tel.failsafe() else "Failsafe inaktiv")
        level = max(0, min(3, int(tel.d_used)))
        self.danger.set_state(
            True,
            level > 0,
            danger_flag_label(level),
            label=danger_flag_label(level),
            level=level,
        )
