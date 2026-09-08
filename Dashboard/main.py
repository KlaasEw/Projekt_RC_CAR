#!/usr/bin/env python3
"""RC Car Visualisierungs-Dashboard."""

from __future__ import annotations

import sys
from pathlib import Path

from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import QApplication

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.ui.main_window import MainWindow


def _dark_palette() -> QPalette:
    palette = QPalette()
    bg = QColor("#0B0F14")
    card = QColor("#121821")
    text = QColor("#E8EEF5")
    disabled = QColor("#6A7380")
    highlight = QColor("#1B3A4A")
    palette.setColor(QPalette.ColorRole.Window, bg)
    palette.setColor(QPalette.ColorRole.WindowText, text)
    palette.setColor(QPalette.ColorRole.Base, card)
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor("#10161E"))
    palette.setColor(QPalette.ColorRole.Text, text)
    palette.setColor(QPalette.ColorRole.Button, QColor("#1A2230"))
    palette.setColor(QPalette.ColorRole.ButtonText, text)
    palette.setColor(QPalette.ColorRole.Highlight, highlight)
    palette.setColor(QPalette.ColorRole.HighlightedText, text)
    palette.setColor(QPalette.ColorRole.ToolTipBase, card)
    palette.setColor(QPalette.ColorRole.ToolTipText, text)
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, disabled)
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, disabled)
    return palette


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("RC Car Dashboard")
    app.setOrganizationName("Projekt_RC_CAR")
    app.setStyle("Fusion")
    app.setFont(QFont("Helvetica Neue", 13))
    app.setPalette(_dark_palette())
    qss = ROOT / "src" / "theme.qss"
    if qss.exists():
        app.setStyleSheet(qss.read_text(encoding="utf-8"))
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
