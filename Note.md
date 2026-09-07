# Mac Adressen

ESP_32_Auto:    1C:C3:AB:BC:05:10
ESP_32_Reciver: 1C:C3:AB:BC:17:28

# Telemetrie-Datenformat v1

Pfad: Auto-ESP → ESP-Now → Empfänger-ESP → USB-Serial → Visualisierung.

Zwei Darstellungen, gleicher Inhalt:

- **ESP-Now:** gepacktes Binärstruct (`__attribute__((packed))`), Little-Endian, fest unter 250 Byte.
- **USB zum Rechner:** eine CSV-Zeile pro Paket, Prefix `TEL`.

Magic `0xA5`, Version `1`. Felder nur hinten anhängen; Version hochzählen.

`magic` ist ein festes Erkennungsbyte am Paketanfang. Der Empfänger prüft zuerst: erstes Byte `0xA5`? Sonst kein Telemetriepaket (Rauschen, falscher Sender, abgeschnittene Daten). `0xA5` = `10100101`, bewusst ungewöhnlich, damit es selten zufällig passt. Entspricht dem Prefix `TEL` in der CSV-Zeile.

## Sektoren und Gefahrenstufen

Lidar-Sektoren (0° = vorne, je 30°), Index in Arrays:


| Index | Name   | Winkel    |
| ----- | ------ | --------- |
| 0     | Links  | 315°–345° |
| 1     | Mitte  | 345°–15°  |
| 2     | Rechts | 15°–45°   |


Entfernung: kleinste gültige Distanz im Sektor, **mm**. `0` = kein Echo / ungültig.

Gefahrenstufe je Sektor (`danger[i]`), Abstandsschwellen grob zum Start (später kalibrieren):


| Stufe | Bedeutung | Distanz (Richtwert) |
| ----- | --------- | ------------------- |
| 0     | frei      | > 4000 mm           |
| 1     | Hinweis   | 2000–4000 mm        |
| 2     | Warnung   | 1000–2000 mm        |
| 3     | Stopp     | < 1000 mm           |


`danger_used` = Stufe, die **aktuell** Gas/Lenkung begrenzt (typisch Maximum der relevanten Sektoren, nicht nur Anzeige).

## ESP-Now Struct

Sender und Empfänger müssen byteidentisch sein. Keine `int`/`bool` (plattformabhängig).

```c
typedef struct __attribute__((packed)) {
  uint8_t  magic;          // 0xA5
  uint8_t  version;        // 1
  uint16_t seq;            // laufende Nummer, Wrap bei 65535
  uint32_t t_ms;           // millis() im Auto

  // Empfänger (iBUS, gemappt wie im IBUS-Prototyp)
  int16_t  rx_gas;         // -100 … 100
  int16_t  rx_servo;       // -45 … 45 (links +, rechts −)
  uint8_t  rx_control;     // 0 = begrenzt, 1 = voll
  uint8_t  rx_ok;          // 1 = frisches iBUS-Frame (< 100 ms)

  // Ansteuerung nach Failsafe / Gefahrenlogik
  int16_t  cmd_gas;        // -100 … 100
  int16_t  cmd_servo;      // -45 … 45

  // MPU6050
  float    ax, ay, az;     // g
  float    gx, gy, gz;     // °/s, gyro-kalibriert
  float    temp_c;         // °C (BME280, Fallback MPU)

  // BME280
  float    alt_rel_m;      // m relativ zur Kalibrierhöhe

  // Akku
  uint16_t vbat_mv;        // mV, 0 = ungültig

  // Lidar + Gefahr
  uint16_t lidar_mm[3];    // L / M / R, 0 = ungültig
  uint8_t  danger[3];      // L / M / R, 0…3
  uint8_t  danger_used;    // 0…3, aktiv verwendete Stufe

  uint8_t  flags;          // siehe unten
} TelemetryV1;
```

Größe: **63 Byte**. ESP-Now-Limit: 250 Byte.

`flags` Bitmaske:


| Bit | Bedeutung                        |
| --- | -------------------------------- |
| 0   | MPU gültig                       |
| 1   | BME gültig                       |
| 2   | Lidar-Scan gültig                |
| 3   | Failsafe aktiv (kein iBUS)       |
| 4   | Gefahr greift ein (`cmd` ≠ `rx`) |
| 5   | Akkuspannung gültig              |


Ungültige Sensorwerte auf `0` setzen und das zugehörige Flag löschen. Nicht senden, wenn `magic`/`version` nicht passen oder `len != sizeof(TelemetryV1)`.

## Serial-CSV (Empfänger → PC)

Eine Zeile, 115200 8N1, kein Leerzeichen:

```
TEL,<ver>,<seq>,<t_ms>,<rx_gas>,<rx_servo>,<rx_control>,<rx_ok>,<cmd_gas>,<cmd_servo>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<temp_c>,<alt_rel_m>,<vbat_mv>,<lidar_l>,<lidar_c>,<lidar_r>,<d_l>,<d_c>,<d_r>,<d_used>,<flags>
```

Floats mit 3 Nachkommastellen. Ganzzahlen ungeändert. Debug-Zeilen ohne Prefix `TEL` ignoriert die Visualisierung.

Beispiel:

```
TEL,1,42,123456,-20,8,1,1,-10,8,0.012,-0.980,0.040,1.20,-0.30,0.05,24.6,0.120,11840,1850,920,2400,1,2,0,2,39
```



## Hinweise für die Pipeline

- Cadence: ca. **20–50 ms** (20–50 Hz). Lidar-Umlauf ~10 Hz: Sektorwerte bis zum nächsten Scan halten.
- `seq` und `t_ms` zum Erkennen von Verlust und Schätzen der Latenz.
- `rx_*` = Wunsch der Fernsteuerung, `cmd_*` = was Motor/Servo wirklich bekommen. So sieht die Visualisierung Eingriffe.
- Keine volle Lidar-Punktewolke über ESP-Now (zu groß). Nur die drei Sektor-Minima.
- `vbat_mv` in Millivolt (z. B. `11840` = 11,840 V). `0` plus Flag-Bit 5 gelöscht, wenn die Messung fehlt.
- Optional später (Struct hinten anhängen, Version 2): VL53L0X `tof_mm[3]`, ESP-Now-RSSI nur lokal am Empfänger (steht nicht im Auto-Paket).

