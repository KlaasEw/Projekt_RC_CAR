# RC Car Dashboard

Live-Telemetrie vom Empfänger-ESP32 über USB-Serial. Dunkles PySide6-Dashboard auf dem Mac: Draufsicht mit Lidar-Sektoren, Lage im Raum, Werte, Flag-Warnungen, CSV-Aufzeichnung und Wiedergabe.

Datenpfad: Auto-ESP → ESP-Now → Empfänger-ESP (`firmware_Reciver`) → USB 115200 8N1 → diese App. Protokoll laut `Note.md` (CSV-Zeilen mit Prefix `TEL`).

## Start

Python 3.9 oder neuer. Im Ordner `Dashboard/`:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 main.py
```

## Verbinden

1. Empfänger-ESP32 per USB anschließen.
2. **PlatformIO-Serial-Monitor, Arduino-IDE und andere Programme schließen.** Der Port darf nur einmal geöffnet sein.
3. Im Dashboard den Port wählen (auf macOS `/dev/cu.…`, nicht `/dev/tty.…`) und **Verbinden**.
4. Baudrate ist fest **115200**. Debug-Zeilen ohne `TEL` werden ignoriert.

Ports aktualisieren, falls der Stick später eingesteckt wird.

## Bedienung

| Aktion | Wirkung |
| ------ | ------- |
| Verbinden / Trennen | Serial öffnen oder schließen |
| Aufzeichnen | Live-Pakete nach `recordings/tel_YYYYMMDD_HHMMSS.csv` |
| Abspielen | vorhandene CSV mit Original-Timing über `t_ms` |
| Pause | Wiedergabe anhalten |
| Lage nullen / `R` | Roll, Pitch, Yaw auf aktuelle Lage setzen |
| Position nullen / Leertaste | Spur und Position zurücksetzen |
| `X` / `Y` / `Z` | IMU-Achse invertieren |

## Anzeige

- **Draufsicht:** Auto von oben, Vorderräder nach `cmd_servo`. Weicht `rx_servo` ab, erscheint ein gelber Wunsch-Pfeil. Drei Lidar-Sektoren (Links 315–345°, Mitte 345–15°, Rechts 15–45°), Farbe nach Gefahrenstufe. Der von der Firmware genutzte Sektor (`rx_servo`-Deadzone 15°) ist cyan umrandet. `+` / `−` oder Mausrad zoomen.
- **Lage im Raum:** Komplementärfilter und Bewegung wie im Prototyp `Visualisierung_Car_Movment`. Ebenfalls `+` / `−` zum Zoomen.
- **Empfänger / Assistenz:** Fernsteuer-Schalter (iBUS-Kanal 5). *an* = Notbremsassistent aktiv (Lidar begrenzt Gas), *aus* = Werte werden durchgereicht.
- **Flags:** Warn-Dreieck, sobald der ungünstige Zustand vorliegt (MPU/BME/Lidar/Akku ungültig, Failsafe, iBUS stale). Gefahr folgt `danger_used`: Hinweis, Warnung oder Stop.
- **Werte:** `rx` vs. `cmd`, IMU, Temperatur, relative Höhe, Akkuspannung in Volt, Lidar mm.

Gefahrenstufen und `danger_used` kommen aus dem Paket, sie werden nicht neu berechnet.

## Aufnahmen

Ordner: `Dashboard/recordings/`. Erste Zeile ist der Header (`version,seq,t_ms,…`), danach dieselben Spalten wie die TEL-Zeile ohne Prefix. Live-`TEL,…`-Dumps können ebenfalls abgespielt werden. Nicht-TEL-Zeilen werden übersprungen.
