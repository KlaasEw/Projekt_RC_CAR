"""Hauptfenster: Verbindung, Grafiken, Werte, Aufzeichnung."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Optional

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QComboBox,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QSizePolicy,
    QSplitter,
    QStackedWidget,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

from src.protocol import Telemetry
from src.recorder import PlaybackThread, Recorder, recordings_dir
from src.serial_worker import BAUD, SerialThread, port_choices
from src.ui.flag_bar import FlagBar
from src.ui.motion_view import MotionViewWidget
from src.ui.telemetry_panel import TelemetryPanel
from src.ui.top_down import TopDownWidget
from src.ui.view_panel import SectorLegend, ViewPanel

STATUS_OK = "#5AD278"
STATUS_WAIT = "#FFBE50"
STATUS_ERR = "#FF6E5A"
STATUS_IDLE = "#8A93A3"


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RC Car Dashboard")
        self.resize(1440, 900)
        self.setMinimumSize(1100, 720)

        self._tel: Optional[Telemetry] = None
        self._serial = SerialThread()
        self._playback = PlaybackThread()
        self._recorder = Recorder()
        self._connected = False
        self._playing = False

        self._serial.packet.connect(self._on_packet)
        self._serial.stats.connect(self._on_stats)
        self._serial.status.connect(self._on_serial_status)
        self._serial.error.connect(self._on_serial_error)
        self._playback.packet.connect(self._on_packet)
        self._playback.progress.connect(self._on_playback_progress)
        self._playback.finished_ok.connect(self._on_playback_finished)
        self._playback.error.connect(self._on_playback_error)

        self.top_down = TopDownWidget()
        self.motion = MotionViewWidget()
        self.sector_legend = SectorLegend()
        self.flags = FlagBar()
        self.telemetry = TelemetryPanel()
        self.flags.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        self.telemetry.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        self._expanded_panel: Optional[ViewPanel] = None
        self._placeholder: Optional[QWidget] = None
        self._home_splitter: Optional[QSplitter] = None

        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(0, 0, 0, 0)
        root_layout.setSpacing(0)

        self._stack = QStackedWidget()
        root_layout.addWidget(self._stack)

        dash = QWidget()
        layout = QVBoxLayout(dash)
        layout.setContentsMargins(14, 14, 14, 8)
        layout.setSpacing(10)

        layout.addWidget(self._build_header())
        layout.addWidget(self._build_toolbar())

        self.top_panel = ViewPanel(self.top_down, extra=self.sector_legend, title="Draufsicht · Lidar")
        self.motion_panel = ViewPanel(self.motion, title="Lage im Raum")
        self.top_panel.expand_requested.connect(lambda: self._expand_panel(self.top_panel))
        self.motion_panel.expand_requested.connect(lambda: self._expand_panel(self.motion_panel))

        views = QSplitter(Qt.Orientation.Horizontal)
        views.setChildrenCollapsible(False)
        views.addWidget(self.top_panel)
        views.addWidget(self.motion_panel)
        views.setStretchFactor(0, 1)
        views.setStretchFactor(1, 1)

        bottom = QWidget()
        bottom_layout = QVBoxLayout(bottom)
        bottom_layout.setContentsMargins(0, 0, 0, 0)
        bottom_layout.setSpacing(10)
        bottom_layout.addWidget(self.flags)
        bottom_layout.addWidget(self.telemetry)

        splitter = QSplitter(Qt.Orientation.Vertical)
        splitter.setChildrenCollapsible(False)
        splitter.addWidget(views)
        splitter.addWidget(bottom)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([560, 260])
        layout.addWidget(splitter, 1)
        self._stack.addWidget(dash)
        self._stack.addWidget(self._build_expand_page())

        status = QStatusBar()
        self.setStatusBar(status)
        self._status_path = QLabel("")
        status.addPermanentWidget(self._status_path)

        self._bind_keys()
        self.refresh_ports()

        self._stale_timer = QTimer(self)
        self._stale_timer.setInterval(400)
        self._stale_timer.timeout.connect(self._check_stale)
        self._stale_timer.start()
        self._last_packet_at = 0.0

    def _build_expand_page(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(14, 14, 14, 8)
        layout.setSpacing(10)

        bar = QHBoxLayout()
        self._expand_heading = QLabel()
        self._expand_heading.setObjectName("titleLabel")
        back = QPushButton("Zurück zur Übersicht")
        back.setObjectName("primaryButton")
        back.clicked.connect(self._collapse_panel)
        bar.addWidget(self._expand_heading)
        bar.addStretch(1)
        bar.addWidget(back)
        layout.addLayout(bar)

        self._expand_slot = QVBoxLayout()
        self._expand_slot.setContentsMargins(0, 0, 0, 0)
        layout.addLayout(self._expand_slot, 1)
        return page

    def _expand_panel(self, panel: ViewPanel):
        if self._expanded_panel is not None:
            return
        parent = panel.parentWidget()
        if not isinstance(parent, QSplitter):
            return
        self._home_splitter = parent
        idx = parent.indexOf(panel)
        self._placeholder = QFrame()
        self._placeholder.setObjectName("panel")
        ph = QVBoxLayout(self._placeholder)
        msg = QLabel(f"{panel.title}\nvergrößert")
        msg.setAlignment(Qt.AlignmentFlag.AlignCenter)
        msg.setObjectName("subtitleLabel")
        ph.addWidget(msg)
        parent.replaceWidget(idx, self._placeholder)

        self._expanded_panel = panel
        panel.fs_btn.setVisible(False)
        self._expand_heading.setText(panel.title)
        self._expand_slot.addWidget(panel)
        self._stack.setCurrentIndex(1)

    def _collapse_panel(self):
        panel = self._expanded_panel
        if panel is None:
            return
        self._expand_slot.removeWidget(panel)
        if self._home_splitter is not None and self._placeholder is not None:
            idx = self._home_splitter.indexOf(self._placeholder)
            if idx >= 0:
                self._home_splitter.replaceWidget(idx, panel)
            self._placeholder.deleteLater()
        self._placeholder = None
        self._home_splitter = None
        self._expanded_panel = None
        panel.fs_btn.setVisible(True)
        self._stack.setCurrentIndex(0)

    def _build_header(self) -> QWidget:
        box = QWidget()
        row = QHBoxLayout(box)
        row.setContentsMargins(4, 0, 4, 0)
        title = QLabel("RC Car Dashboard")
        title.setObjectName("titleLabel")
        subtitle = QLabel("Telemetrie v1  ·  115200 8N1  ·  Prefix TEL")
        subtitle.setObjectName("subtitleLabel")
        col = QVBoxLayout()
        col.setSpacing(0)
        col.addWidget(title)
        col.addWidget(subtitle)
        row.addLayout(col)
        row.addStretch()
        self.hz_label = QLabel("0 Hz")
        self.hz_label.setObjectName("subtitleLabel")
        self.seq_label = QLabel("seq –")
        self.seq_label.setObjectName("subtitleLabel")
        row.addWidget(self.hz_label)
        row.addWidget(self.seq_label)
        return box

    def _build_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName("toolbar")
        row = QHBoxLayout(bar)
        row.setContentsMargins(12, 8, 12, 8)
        row.setSpacing(8)

        row.addWidget(QLabel("Port"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(320)
        row.addWidget(self.port_combo)

        self.refresh_btn = QPushButton("Aktualisieren")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        row.addWidget(self.refresh_btn)

        self.connect_btn = QPushButton("Verbinden")
        self.connect_btn.setObjectName("primaryButton")
        self.connect_btn.clicked.connect(self._toggle_connection)
        row.addWidget(self.connect_btn)

        self.status_dot = QLabel("● getrennt")
        self._set_status("getrennt", STATUS_IDLE)
        row.addWidget(self.status_dot)

        row.addSpacing(12)

        self.record_btn = QPushButton("Aufzeichnen")
        self.record_btn.clicked.connect(self._toggle_record)
        row.addWidget(self.record_btn)

        self.play_btn = QPushButton("Abspielen")
        self.play_btn.clicked.connect(self._toggle_playback)
        row.addWidget(self.play_btn)

        self.pause_btn = QPushButton("Pause")
        self.pause_btn.setEnabled(False)
        self.pause_btn.clicked.connect(self._toggle_pause)
        row.addWidget(self.pause_btn)

        self.progress = QProgressBar()
        self.progress.setFixedWidth(160)
        self.progress.setValue(0)
        self.progress.setFormat("%v / %m")
        row.addWidget(self.progress)

        row.addStretch()

        zero_pose = QPushButton("Lage nullen")
        zero_pose.clicked.connect(self.motion.zero_pose)
        row.addWidget(zero_pose)
        zero_pos = QPushButton("Position nullen")
        zero_pos.clicked.connect(self.motion.zero_position)
        row.addWidget(zero_pos)
        return bar

    def _bind_keys(self):
        QShortcut(QKeySequence("R"), self, activated=self.motion.zero_all)
        QShortcut(QKeySequence("Space"), self, activated=self.motion.zero_position)
        QShortcut(QKeySequence("X"), self, activated=lambda: self.motion.toggle_axis("x"))
        QShortcut(QKeySequence("Y"), self, activated=lambda: self.motion.toggle_axis("y"))
        QShortcut(QKeySequence("Z"), self, activated=lambda: self.motion.toggle_axis("z"))
        QShortcut(QKeySequence("Escape"), self, activated=self._collapse_panel)

    def refresh_ports(self):
        current = self.port_combo.currentData()
        self.port_combo.clear()
        for device, label in port_choices():
            self.port_combo.addItem(label, device)
        if current:
            idx = self.port_combo.findData(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)
        if self.port_combo.count() == 0:
            self.port_combo.addItem("kein Port gefunden", "")

    def _set_status(self, text: str, color: str):
        self.status_dot.setText(f"● {text}")
        self.status_dot.setStyleSheet(f"color: {color}; font-weight: 600;")

    def _toggle_connection(self):
        if self._connected:
            self._disconnect()
            return
        self._stop_playback()
        port = self.port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "Kein Port", "Bitte eine serielle Schnittstelle wählen.")
            return
        self.motion.zero_all()
        self._serial.start_port(port, BAUD)
        self._connected = True
        self.connect_btn.setText("Trennen")
        self.connect_btn.setObjectName("dangerButton")
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)
        self.port_combo.setEnabled(False)
        self._set_status("verbinde…", STATUS_WAIT)

    def _disconnect(self):
        self._serial.stop_port()
        self._connected = False
        self.connect_btn.setText("Verbinden")
        self.connect_btn.setObjectName("primaryButton")
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)
        self.port_combo.setEnabled(True)
        self._set_status("getrennt", STATUS_IDLE)
        self.hz_label.setText("0 Hz")

    def _on_packet(self, tel: Telemetry):
        self._tel = tel
        self._last_packet_at = datetime.now().timestamp()
        self.top_down.set_telemetry(tel)
        self.motion.push_telemetry(tel)
        self.telemetry.set_pose(self.motion.imu.pose())
        self.telemetry.set_telemetry(tel)
        self.sector_legend.set_telemetry(tel)
        self.flags.set_telemetry(tel)
        if self._recorder.is_recording and not self._playing:
            self._recorder.write(tel)

    def _on_stats(self, hz: float, seq: int):
        self.hz_label.setText(f"{hz:.0f} Hz")
        self.seq_label.setText(f"seq {seq}")
        if self._connected:
            self._set_status("verbunden", STATUS_OK)

    def _on_serial_status(self, text: str):
        if text.startswith("verbunden"):
            self._set_status("warte auf TEL…", STATUS_WAIT)
            self.statusBar().showMessage(text)
        elif text == "getrennt" and not self._connected:
            self._set_status("getrennt", STATUS_IDLE)

    def _on_serial_error(self, message: str):
        self._connected = False
        self.connect_btn.setText("Verbinden")
        self.connect_btn.setObjectName("primaryButton")
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)
        self.port_combo.setEnabled(True)
        self._set_status("Fehler", STATUS_ERR)
        self.statusBar().showMessage(message)
        QMessageBox.critical(self, "Serial-Fehler", message)

    def _check_stale(self):
        if not self._connected or self._playing:
            return
        if self._last_packet_at and datetime.now().timestamp() - self._last_packet_at > 1.5:
            self._set_status("warte auf TEL…", STATUS_WAIT)

    def _toggle_record(self):
        if self._recorder.is_recording:
            path = self._recorder.stop()
            self.record_btn.setText("Aufzeichnen")
            self.record_btn.setObjectName("")
            self.record_btn.style().unpolish(self.record_btn)
            self.record_btn.style().polish(self.record_btn)
            if path:
                self._status_path.setText(str(path))
                self.statusBar().showMessage(f"Aufnahme gespeichert: {path}")
            return
        if self._playing:
            QMessageBox.information(self, "Wiedergabe", "Während der Wiedergabe kann nicht aufgezeichnet werden.")
            return
        name = datetime.now().strftime("tel_%Y%m%d_%H%M%S.csv")
        path = recordings_dir() / name
        self._recorder.start(path)
        self.record_btn.setText("Aufnahme stoppen")
        self.record_btn.setObjectName("recordButton")
        self.record_btn.style().unpolish(self.record_btn)
        self.record_btn.style().polish(self.record_btn)
        self._status_path.setText(f"zeichnet auf: {path.name}")

    def _toggle_playback(self):
        if self._playing:
            self._stop_playback()
            return
        start = str(recordings_dir())
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Aufnahme öffnen",
            start,
            "CSV (*.csv);;Alle Dateien (*)",
        )
        if not path:
            return
        count = self._playback.load(Path(path))
        if count == 0:
            QMessageBox.warning(self, "Leere Datei", "Keine gültigen TEL-Zeilen gefunden.")
            return
        if self._connected:
            self._disconnect()
        self.motion.zero_all()
        self.progress.setMaximum(count)
        self.progress.setValue(0)
        self._playing = True
        self.play_btn.setText("Wiedergabe stoppen")
        self.pause_btn.setEnabled(True)
        self.pause_btn.setText("Pause")
        self._status_path.setText(Path(path).name)
        self._set_status("Wiedergabe", "#5AD2FF")
        self._playback.start_playback()

    def _toggle_pause(self):
        if not self._playing:
            return
        paused = not self._playback.is_paused()
        self._playback.pause(paused)
        self.pause_btn.setText("Weiter" if paused else "Pause")
        self._set_status("Pause" if paused else "Wiedergabe", STATUS_WAIT if paused else "#5AD2FF")

    def _stop_playback(self):
        self._playback.stop_playback()
        self._playing = False
        self.play_btn.setText("Abspielen")
        self.pause_btn.setEnabled(False)
        self.pause_btn.setText("Pause")
        self._set_status("getrennt", STATUS_IDLE)

    def _on_playback_progress(self, current: int, total: int):
        self.progress.setMaximum(total)
        self.progress.setValue(current)
        if self._tel is not None:
            self.seq_label.setText(f"seq {self._tel.seq}")
            self.hz_label.setText(f"{current}/{total}")

    def _on_playback_finished(self):
        self._playing = False
        self.play_btn.setText("Abspielen")
        self.pause_btn.setEnabled(False)
        self._set_status("Wiedergabe Ende", STATUS_OK)
        self.statusBar().showMessage("Wiedergabe beendet")

    def _on_playback_error(self, message: str):
        self._stop_playback()
        QMessageBox.warning(self, "Wiedergabe", message)

    def closeEvent(self, event):
        self._collapse_panel()
        self._recorder.stop()
        self._playback.stop_playback()
        self._serial.stop_port()
        event.accept()
