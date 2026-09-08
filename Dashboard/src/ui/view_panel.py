"""Karte mit Zeichenfläche und fester Fußzeile (Legende + Zoom)."""

from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from src.protocol import SECTOR_NAMES, Telemetry, danger_label

DANGER_HEX = {
    0: "#5AD278",
    1: "#FFBE50",
    2: "#FF8C3C",
    3: "#FF6E5A",
}


class SectorLegend(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("sectorLegend")
        self.setTextFormat(Qt.TextFormat.RichText)
        self.setText("keine Telemetrie")
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

    def set_telemetry(self, tel: Optional[Telemetry]):
        if tel is None:
            self.setText("keine Telemetrie")
            return
        used = tel.used_sector_index()
        parts = []
        for i, name in enumerate(SECTOR_NAMES):
            mm = tel.lidar_mm[i]
            dng = tel.danger[i]
            color = DANGER_HEX.get(dng, "#8A93A3") if mm > 0 else "#6A7380"
            dist = f"{mm / 1000.0:.2f} m" if mm > 0 else "—"
            weight = "700" if i == used else "500"
            arrow = " ←" if i == used else ""
            parts.append(
                f"<span style='color:{color}; font-weight:{weight};'>"
                f"{name[0]} {dist}{arrow}</span>"
            )
        used_name = SECTOR_NAMES[used]
        used_color = DANGER_HEX.get(tel.d_used, "#5AD2FF")
        parts.append(
            f"<span style='color:{used_color};'> · {used_name} {danger_label(tel.d_used)}</span>"
        )
        self.setText("&nbsp;&nbsp;".join(parts))


class ViewPanel(QFrame):
    """Abgerundete Karte: Canvas oben, Zoom und optionale Legende unten im Layout."""

    def __init__(self, canvas: QWidget, extra: QWidget | None = None, parent=None):
        super().__init__(parent)
        self.setObjectName("panel")
        self.canvas = canvas
        canvas.setMinimumSize(240, 200)
        canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)
        layout.addWidget(canvas, 1)

        footer = QHBoxLayout()
        footer.setContentsMargins(2, 0, 2, 2)
        footer.setSpacing(8)
        if extra is not None:
            footer.addWidget(extra, 1)
        else:
            footer.addStretch(1)

        self.zoom_out_btn = QPushButton("−")
        self.zoom_in_btn = QPushButton("+")
        for btn in (self.zoom_out_btn, self.zoom_in_btn):
            btn.setObjectName("zoomButton")
            btn.setFixedSize(36, 36)
            btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
            btn.setStyleSheet("padding: 0px; margin: 0px;")
        self.zoom_out_btn.setToolTip("Herauszoomen")
        self.zoom_in_btn.setToolTip("Hineinzoomen")
        self.zoom_out_btn.clicked.connect(canvas.zoom_out)
        self.zoom_in_btn.clicked.connect(canvas.zoom_in)
        footer.addWidget(self.zoom_out_btn)
        footer.addWidget(self.zoom_in_btn)
        layout.addLayout(footer)
