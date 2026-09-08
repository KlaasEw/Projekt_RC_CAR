"""Wertkarten für alle TEL-Felder."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QFrame, QGridLayout, QLabel, QSizePolicy, QVBoxLayout, QWidget

from src.protocol import SECTOR_NAMES, Telemetry, control_label, danger_label

MUTED = "#8A93A3"
TEXT = "#E8EEF5"
ACCENT = "#5AD2FF"
WARN = "#FFBE50"
ERR = "#FF6E5A"
OK = "#5AD278"
DIM = "#5A6370"


class BarWidget(QWidget):
    def __init__(self, vmin: float, vmax: float, parent=None):
        super().__init__(parent)
        self._min = vmin
        self._max = vmax
        self._value = 0.0
        self._cmd = None
        self.setFixedHeight(10)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def set_values(self, value: float, cmd: float | None = None):
        self._value = value
        self._cmd = cmd
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#1A2230"))
        painter.drawRoundedRect(QRectF(0, 0, w, h), 4, 4)
        span = self._max - self._min or 1.0
        mid = (0.0 - self._min) / span * w
        painter.setBrush(QColor("#5AD2FF"))
        x = (self._value - self._min) / span * w
        painter.drawRoundedRect(QRectF(min(mid, x), 1, abs(x - mid), h - 2), 3, 3)
        if self._cmd is not None and abs(self._cmd - self._value) > 0.01:
            cx = (self._cmd - self._min) / span * w
            painter.setPen(QPen(QColor(WARN), 2))
            painter.drawLine(int(cx), 0, int(cx), h)


class MetricCard(QFrame):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setObjectName("card")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(6)
        self.title = QLabel(title.upper())
        self.title.setObjectName("panelTitle")
        layout.addWidget(self.title)
        self.body = QLabel("–")
        self.body.setWordWrap(True)
        self.body.setTextFormat(Qt.TextFormat.RichText)
        layout.addWidget(self.body)
        self.bar = None

    def add_bar(self, vmin: float, vmax: float) -> BarWidget:
        self.bar = BarWidget(vmin, vmax)
        self.layout().addWidget(self.bar)
        return self.bar

    def set_html(self, html: str):
        self.body.setText(html)


def _row(label: str, value: str, color: str = TEXT, dim: bool = False) -> str:
    c = DIM if dim else color
    return (
        f"<tr>"
        f"<td style='color:{MUTED}; padding-right:12px;'>{label}</td>"
        f"<td style='color:{c}; font-family:Menlo,monospace;'>{value}</td>"
        f"</tr>"
    )


def _table(rows: str) -> str:
    return f"<table cellspacing='0' cellpadding='2'>{rows}</table>"


class TelemetryPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(10)

        self.gas = MetricCard("Gas")
        self.gas.add_bar(-100, 100)
        self.servo = MetricCard("Lenkung")
        self.servo.add_bar(-45, 45)
        self.mode = MetricCard("Empfänger")
        self.mode.setToolTip(
            "Assistenz kommt vom Fernsteuer-Schalter (iBUS-Kanal 5).\n"
            "an = Notbremsassistent aktiv, Lidar begrenzt das Gas.\n"
            "aus = Empfängerwerte werden ungefiltert durchgereicht."
        )
        self.imu = MetricCard("IMU  MPU6050")
        self.env = MetricCard("Umgebung")
        self.batt = MetricCard("Akku")
        self.lidar = MetricCard("Lidar-Sektoren")

        grid.addWidget(self.gas, 0, 0)
        grid.addWidget(self.servo, 0, 1)
        grid.addWidget(self.mode, 0, 2)
        grid.addWidget(self.batt, 0, 3)
        grid.addWidget(self.imu, 1, 0, 1, 2)
        grid.addWidget(self.env, 1, 2)
        grid.addWidget(self.lidar, 1, 3)
        for col in range(4):
            grid.setColumnStretch(col, 1)
        self._tel = None
        self._pose = None
        self.clear()

    def set_pose(self, pose):
        self._pose = pose

    def clear(self):
        empty = _table(_row("Status", "keine Daten", WARN))
        for card in (self.gas, self.servo, self.mode, self.imu, self.env, self.batt, self.lidar):
            card.set_html(empty)
        if self.gas.bar:
            self.gas.bar.set_values(0)
        if self.servo.bar:
            self.servo.bar.set_values(0)

    def set_telemetry(self, tel: Optional[Telemetry]):
        if tel is None:
            self._tel = None
            self.clear()
            return
        self._tel = tel

        gas_color = ERR if tel.cmd_gas != tel.rx_gas else TEXT
        servo_color = ERR if tel.cmd_servo != tel.rx_servo else TEXT
        self.gas.set_html(
            _table(
                _row("rx", f"{tel.rx_gas:+d} %")
                + _row("cmd", f"{tel.cmd_gas:+d} %", gas_color)
            )
        )
        self.gas.bar.set_values(tel.rx_gas, tel.cmd_gas)
        self.servo.set_html(
            _table(
                _row("rx", f"{tel.rx_servo:+d} °")
                + _row("cmd", f"{tel.cmd_servo:+d} °", servo_color)
            )
        )
        self.servo.bar.set_values(tel.rx_servo, tel.cmd_servo)

        ibus_color = OK if tel.ibus_ok() else ERR
        assist_on = tel.rx_control != 1
        self.mode.set_html(
            _table(
                _row("Assistenz", control_label(tel.rx_control), WARN if assist_on else ACCENT)
                + _row("iBUS", "ok" if tel.ibus_ok() else "stale", ibus_color)
                + _row("seq", str(tel.seq), MUTED)
                + _row("t_ms", str(tel.t_ms), MUTED)
            )
        )

        imu_dim = not tel.mpu_ok()
        pose = self._pose
        derived = ""
        if pose is not None:
            roll, pitch, yaw = pose[0], pose[1], pose[2]
            lin_ax, lin_ay = pose[15], pose[16]
            px, py, alt = pose[11], pose[12], pose[14]
            derived = (
                _row("Lage [°]", f"{roll:+6.1f}  {pitch:+6.1f}  {yaw:+6.1f}", dim=imu_dim)
                + _row("Lin-a [g]", f"{lin_ax:+6.2f}  {lin_ay:+6.2f}", dim=imu_dim)
                + _row("Pos / H", f"{px:+5.2f}  {py:+5.2f}   {alt:+5.2f} m", dim=imu_dim)
            )
        self.imu.set_html(
            _table(
                _row("Acc [g]", f"{tel.ax:+7.3f}  {tel.ay:+7.3f}  {tel.az:+7.3f}", dim=imu_dim)
                + _row("Gyro [°/s]", f"{tel.gx:+7.2f}  {tel.gy:+7.2f}  {tel.gz:+7.2f}", dim=imu_dim)
                + derived
            )
        )

        env_dim = not tel.bme_ok() and not tel.mpu_ok()
        self.env.set_html(
            _table(
                _row("Temp", f"{tel.temp_c:5.1f} °C", dim=env_dim)
                + _row("Höhe rel.", f"{tel.alt_rel_m:+6.3f} m", dim=not tel.bme_ok())
            )
        )

        batt_dim = not tel.vbat_ok()
        self.batt.set_html(
            _table(
                _row("Spannung", f"{tel.vbat_v:5.2f} V" if tel.vbat_mv else "–", dim=batt_dim)
                + _row("roh", f"{tel.vbat_mv} mV", dim=batt_dim)
            )
        )

        lidar_rows = ""
        for i, name in enumerate(SECTOR_NAMES):
            mm = tel.lidar_mm[i]
            dng = tel.danger[i]
            dim = not tel.lidar_ok() or mm == 0
            color = TEXT
            if not dim:
                color = {0: OK, 1: WARN, 2: "#FF8C3C", 3: ERR}.get(dng, TEXT)
            used = "  ←" if i == tel.used_sector_index() else ""
            dist = f"{mm} mm" if mm else "–"
            lidar_rows += _row(name, f"{dist}   {danger_label(dng)}{used}", color, dim=dim)
        lidar_rows += _row("genutzt", f"Stufe {tel.d_used}  {danger_label(tel.d_used)}", ACCENT)
        self.lidar.set_html(_table(lidar_rows))
