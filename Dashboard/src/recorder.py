"""CSV-Aufzeichnung und Wiedergabe mit Original-Timing über t_ms."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

from PySide6.QtCore import QThread, Signal

from src.protocol import CSV_COLUMNS, Telemetry, parse_tel

MIN_DELAY_MS = 1
MAX_DELAY_MS = 200
DEFAULT_DELAY_MS = 30


def recordings_dir() -> Path:
    path = Path(__file__).resolve().parent.parent / "recordings"
    path.mkdir(parents=True, exist_ok=True)
    return path


def load_telemetry_file(path: Path) -> list[Telemetry]:
    packets: list[Telemetry] = []
    text = path.read_text(encoding="utf-8", errors="ignore")
    for raw in text.splitlines():
        tel = parse_tel(raw)
        if tel is not None:
            packets.append(tel)
    return packets


class Recorder:
    def __init__(self):
        self._file = None
        self.path: Optional[Path] = None

    @property
    def is_recording(self) -> bool:
        return self._file is not None

    def start(self, path: Path):
        self.stop()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self._file = path.open("w", encoding="utf-8", newline="\n")
        self._file.write(",".join(CSV_COLUMNS) + "\n")
        self._file.flush()

    def write(self, tel: Telemetry):
        if self._file is None:
            return
        self._file.write(tel.to_csv_row() + "\n")
        self._file.flush()

    def stop(self) -> Optional[Path]:
        path = self.path
        if self._file is not None:
            try:
                self._file.close()
            except Exception:
                pass
            self._file = None
        self.path = None
        return path


class PlaybackThread(QThread):
    packet = Signal(object)
    progress = Signal(int, int)
    finished_ok = Signal()
    error = Signal(str)

    def __init__(self):
        super().__init__()
        self._packets: list[Telemetry] = []
        self._running = False
        self._paused = False

    def load(self, path: Path) -> int:
        self._packets = load_telemetry_file(path)
        return len(self._packets)

    def start_playback(self):
        self.stop_playback()
        if not self._packets:
            self.error.emit("Keine TEL-Pakete in der Datei.")
            return
        self._running = True
        self._paused = False
        self.start()

    def pause(self, paused: bool):
        self._paused = paused

    def is_paused(self) -> bool:
        return self._paused

    def stop_playback(self):
        self._running = False
        self._paused = False
        if self.isRunning():
            self.wait(2000)

    def run(self):
        n = len(self._packets)
        prev_t = None
        try:
            for i, tel in enumerate(self._packets):
                if not self._running:
                    break
                while self._paused and self._running:
                    self.msleep(40)
                if not self._running:
                    break
                delay = DEFAULT_DELAY_MS
                if prev_t is not None:
                    raw = int(tel.t_ms) - int(prev_t)
                    if raw < 0:
                        raw = DEFAULT_DELAY_MS
                    delay = max(MIN_DELAY_MS, min(MAX_DELAY_MS, raw))
                prev_t = tel.t_ms
                self.packet.emit(tel)
                self.progress.emit(i + 1, n)
                self.msleep(delay)
            if self._running:
                self.finished_ok.emit()
        except Exception as exc:
            self.error.emit(str(exc))
