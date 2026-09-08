"""Telemetrie v1: TEL-CSV laut Note.md."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

TEL_PREFIX = "TEL"
TEL_VERSION = 1
TEL_FIELD_COUNT = 26  # ohne Prefix

FLAG_MPU = 1 << 0
FLAG_BME = 1 << 1
FLAG_LIDAR = 1 << 2
FLAG_FAILSAFE = 1 << 3
FLAG_DANGER = 1 << 4
FLAG_VBAT = 1 << 5

STEER_DEADZONE = 15  # Grad, wie firmware_RC_Car

SECTOR_LEFT = 0
SECTOR_CENTER = 1
SECTOR_RIGHT = 2
SECTOR_NAMES = ("Links", "Mitte", "Rechts")
SECTOR_ANGLES = (
    (315.0, 345.0),
    (345.0, 15.0),
    (15.0, 45.0),
)

CSV_COLUMNS = (
    "version",
    "seq",
    "t_ms",
    "rx_gas",
    "rx_servo",
    "rx_control",
    "rx_ok",
    "cmd_gas",
    "cmd_servo",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "temp_c",
    "alt_rel_m",
    "vbat_mv",
    "lidar_l",
    "lidar_c",
    "lidar_r",
    "d_l",
    "d_c",
    "d_r",
    "d_used",
    "flags",
)


@dataclass(slots=True)
class Telemetry:
    version: int
    seq: int
    t_ms: int
    rx_gas: int
    rx_servo: int
    rx_control: int
    rx_ok: int
    cmd_gas: int
    cmd_servo: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    temp_c: float
    alt_rel_m: float
    vbat_mv: int
    lidar_l: int
    lidar_c: int
    lidar_r: int
    d_l: int
    d_c: int
    d_r: int
    d_used: int
    flags: int

    @property
    def lidar_mm(self) -> tuple[int, int, int]:
        return (self.lidar_l, self.lidar_c, self.lidar_r)

    @property
    def danger(self) -> tuple[int, int, int]:
        return (self.d_l, self.d_c, self.d_r)

    @property
    def vbat_v(self) -> float:
        return self.vbat_mv / 1000.0 if self.vbat_mv else 0.0

    def has_flag(self, bit: int) -> bool:
        return bool(self.flags & bit)

    def mpu_ok(self) -> bool:
        return self.has_flag(FLAG_MPU)

    def bme_ok(self) -> bool:
        return self.has_flag(FLAG_BME)

    def lidar_ok(self) -> bool:
        return self.has_flag(FLAG_LIDAR)

    def failsafe(self) -> bool:
        return self.has_flag(FLAG_FAILSAFE)

    def danger_active(self) -> bool:
        return self.has_flag(FLAG_DANGER)

    def vbat_ok(self) -> bool:
        return self.has_flag(FLAG_VBAT)

    def ibus_ok(self) -> bool:
        return self.rx_ok == 1

    def cmd_differs(self) -> bool:
        return self.cmd_gas != self.rx_gas or self.cmd_servo != self.rx_servo

    def used_sector_index(self) -> int:
        """Sektor, den die Firmware für danger_used heranzieht."""
        if self.rx_servo > STEER_DEADZONE:
            return SECTOR_RIGHT
        if self.rx_servo < -STEER_DEADZONE:
            return SECTOR_LEFT
        return SECTOR_CENTER

    def to_csv_row(self) -> str:
        return ",".join(
            (
                str(self.version),
                str(self.seq),
                str(self.t_ms),
                str(self.rx_gas),
                str(self.rx_servo),
                str(self.rx_control),
                str(self.rx_ok),
                str(self.cmd_gas),
                str(self.cmd_servo),
                f"{self.ax:.3f}",
                f"{self.ay:.3f}",
                f"{self.az:.3f}",
                f"{self.gx:.3f}",
                f"{self.gy:.3f}",
                f"{self.gz:.3f}",
                f"{self.temp_c:.3f}",
                f"{self.alt_rel_m:.3f}",
                str(self.vbat_mv),
                str(self.lidar_l),
                str(self.lidar_c),
                str(self.lidar_r),
                str(self.d_l),
                str(self.d_c),
                str(self.d_r),
                str(self.d_used),
                str(self.flags),
            )
        )


def _to_int(text: str) -> int:
    return int(float(text.strip()))


def _to_float(text: str) -> float:
    return float(text.strip())


def telemetry_from_fields(fields: list[str]) -> Optional[Telemetry]:
    if len(fields) != TEL_FIELD_COUNT:
        return None
    try:
        version = _to_int(fields[0])
        if version != TEL_VERSION:
            return None
        return Telemetry(
            version=version,
            seq=_to_int(fields[1]) & 0xFFFF,
            t_ms=_to_int(fields[2]),
            rx_gas=_to_int(fields[3]),
            rx_servo=_to_int(fields[4]),
            rx_control=_to_int(fields[5]),
            rx_ok=_to_int(fields[6]),
            cmd_gas=_to_int(fields[7]),
            cmd_servo=_to_int(fields[8]),
            ax=_to_float(fields[9]),
            ay=_to_float(fields[10]),
            az=_to_float(fields[11]),
            gx=_to_float(fields[12]),
            gy=_to_float(fields[13]),
            gz=_to_float(fields[14]),
            temp_c=_to_float(fields[15]),
            alt_rel_m=_to_float(fields[16]),
            vbat_mv=_to_int(fields[17]),
            lidar_l=_to_int(fields[18]),
            lidar_c=_to_int(fields[19]),
            lidar_r=_to_int(fields[20]),
            d_l=_to_int(fields[21]),
            d_c=_to_int(fields[22]),
            d_r=_to_int(fields[23]),
            d_used=_to_int(fields[24]),
            flags=_to_int(fields[25]),
        )
    except ValueError:
        return None


def parse_tel(line: str) -> Optional[Telemetry]:
    """Parst eine Serial-Zeile. Nicht-TEL-Zeilen werden ignoriert."""
    text = line.strip().replace("\r", "")
    if not text:
        return None
    if text.startswith(TEL_PREFIX + ","):
        parts = text.split(",")
        if len(parts) != TEL_FIELD_COUNT + 1:
            return None
        return telemetry_from_fields(parts[1:])
    if text.startswith("version,") or text.startswith(CSV_COLUMNS[0] + ","):
        return None
    parts = text.split(",")
    if len(parts) == TEL_FIELD_COUNT:
        return telemetry_from_fields(parts)
    return None


def danger_label(level: int) -> str:
    return {0: "frei", 1: "Hinweis", 2: "Warnung", 3: "Stopp"}.get(level, str(level))


def danger_flag_label(level: int) -> str:
    return {
        0: "Gefahr: frei",
        1: "Gefahr: Hinweis",
        2: "Gefahr: Warnung",
        3: "Gefahr: Stop",
    }.get(level, f"Gefahr: {level}")


def control_label(value: int) -> str:
    """rx_control: 1 = Assistent aus, 0 = Assistent an (Lidar begrenzt Gas)."""
    return "aus (durchreichen)" if value == 1 else "an (Gas begrenzt)"
