# RPLIDAR C1 am Mac anschließen und visualisieren

Schritt-für-Schritt: Sensor per USB mit dem Mac verbinden und die Live-Punktewolke mit `visualize_lidar.py` anzeigen.

## Was du brauchst

- Slamtec **RPLIDAR C1**
- Das mitgelieferte **USB-Adapterboard** (meist CP210x, USB-A oder USB-C)
- Das kurze **XH2.54-Kabel** zwischen Lidar und Adapter (oft schon steckt)
- Einen freien USB-Anschluss am Mac (bei USB-C-Macs ggf. USB-A-Adapter)
- Python 3.9 oder neuer (`python3 --version`)

Der C1 braucht **5 V** und etwa **230 mA**. Ein normaler Mac-USB-Port reicht. Ungepowerte USB-Hubs lieber vermeiden.

Der Sensor ist Laserklasse 1. Trotzdem nicht direkt in das optische Fenster starren und das Fenster nicht abdecken.

---

## 1. Hardware verbinden

1. Lege den C1 auf eine **ebene, freie Fläche**. Der Markierungsstrich bzw. das Kabel am Sockel zeigt nach vorne (0° im Radar).
2. Stecke das **5-polige Kabel** vom Lidar in das USB-Adapterboard. Die Stecker sind verpolungssicher – nicht mit Gewalt umdrehen.
3. Verbinde das Adapterboard **per USB mit dem Mac**.
4. Der Motor darf sich jetzt noch nicht drehen. Die Laser-Einheit startet erst, wenn das Skript den Scan-Befehl schickt.

Pinbelegung nur zur Kontrolle (falls du ohne Adapterboard misst):

| Signal | Richtung | Bemerkung        |
| ------ | -------- | ---------------- |
| VCC    | Versorgung | **5 V**, nicht 3,3 V |
| TX     | Lidar → Mac | UART-Daten       |
| RX     | Mac → Lidar | UART-Befehle     |
| GND    | Masse    | gemeinsam        |

Logikpegel der UART-Leitungen: 3,3 V. Versorgung trotzdem 5 V.

---

## 2. Prüfen, ob macOS den Adapter sieht

Ab macOS 11 (Big Sur) ist der **CP210x-Treiber von Apple** schon dabei. Einen extra SiLabs-Treiber brauchst du in der Regel nicht. Einen alten, selbst installierten SiLabs-Treiber besser entfernen – der kann mit dem Apple-Treiber kollidieren.

Terminal öffnen und **alle** `cu`-Geräte listen (zsh-tauglich, ohne Glob-Fehler):

```bash
ls /dev/cu.*
```

Relevant sind Einträge mit `usbserial`, `SLAB`, `wchusbserial` oder `usbmodem`. Gezielter:

```bash
ls /dev/cu.* | grep -iE 'usbserial|SLAB|wchusb|usbmodem'
```

Erwartetes Ergebnis, zum Beispiel:

```text
/dev/cu.usbserial-1130
```

Nicht verwenden: `ls /dev/cu.usbserial* /dev/cu.SLAB* /dev/cu.wchusbserial*`. In zsh bricht der Befehl ab, sobald **eines** der Muster nichts findet – auch wenn der Lidar schon als `usbserial-…` da ist.

Wichtig: Immer **`/dev/cu.*`** verwenden, nicht `/dev/tty.*`. Die `tty.*`-Geräte warten auf ein Modem-Signal und blockieren.

Wenn nichts erscheint:

1. Anderen USB-Anschluss bzw. anderen Adapter versuchen.
2. Systemeinstellungen → Allgemein → Info → Systembericht → USB: nach „CP210x“, „Silicon Labs“ oder „USB-Serial“ suchen.
3. Kabel und Adapterboard prüfen. Manche Boards brauchen erst den Lidar-Stecker, bevor sie am USB enumerieren.
4. Falls wirklich kein Treiber da ist: [CP210x VCP Driver von Silicon Labs](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers).

---

## 3. Python-Umgebung einrichten

Im Projektordner:

```bash
cd prototyps/RPLidar_C1
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
```

Es wird nur **pyserial** benötigt. Die Grafik läuft mit Tkinter, das bei der macOS-Python-Installation schon dabei ist.

Falls `tkinter` fehlt (selten, z. B. bei manchen Homebrew-Pythons):

```bash
brew install python-tk
```

---

## 4. Visualisierung starten

Zuerst die Ports listen:

```bash
python3 visualize_lidar.py --list
```

Der Eintrag mit `*` ist der vermutete C1 (CP210x / usbserial). Wenn gleichzeitig ein ESP32 steckt, den C1-Port explizit angeben.

Dann starten:

```bash
python3 visualize_lidar.py
```

oder fest:

```bash
python3 visualize_lidar.py --port /dev/cu.usbserial-1130
```

Nach 1–2 Sekunden sollte sich der Kopf drehen und im Fenster ein Radar erscheinen: Sensor in der Mitte, 0° oben = vorne.

### Bedienung

| Taste / Geste | Wirkung |
| ------------- | ------- |
| `+` / `-` oder Mausrad | Zoom (Anzeige-Radius) |
| `M` | Nächstes Hindernis ein/aus |
| `R` | Zoom auf 8 m zurück |
| `Esc` oder `Q` | Beenden, Motor stoppen |

### Nützliche Optionen

```bash
python3 visualize_lidar.py --range 4      # Innenraum, 4 m Radius
python3 visualize_lidar.py --baud 460800  # Standard des C1, nur ändern wenn nötig
```

---

## 5. Wenn etwas nicht klappt

**Fenster bleibt gelb bei „verbinde / Motor hochfahren“**

- Port mit `--list` prüfen und `--port` setzen.
- Arduino-IDE, PlatformIO-Serial-Monitor und andere Programme schließen. Der Port darf nur einmal geöffnet sein.
- USB kurz ziehen und wieder einstecken, dann Skript neu starten.

**Fehler „Keine Antwort vom Lidar“**

- Baudrate muss **460800** sein (nicht 115200 wie beim A1).
- Kabel TX/RX nicht vertauscht (beim Original-Adapterboard kein Thema).
- Nach einem harten Abbruch 2 Sekunden warten: Der C1 braucht nach einem Reset etwas Zeit.

**Motor dreht nicht**

- 5-V-Versorgung über USB prüfen (kein passiver Hub).
- Skript beendet den Motor sauber. Nach Absturz USB kurz trennen.

**Punktewolke verdreht**

- Sensor so drehen, dass Kabel/Markierung nach vorne zeigt. 0° ist die Blickrichtung nach vorne.

**„Resource busy“ / Permission denied**

- Anderen Prozess vom Port nehmen: `lsof /dev/cu.usbserial-*`
- Nicht `/dev/tty.usbserial-*` verwenden.

**Schutzstopp**

- Optisches Fenster frei? Sensor nicht festgeklemmt?
- USB trennen, ein paar Sekunden warten, wieder einstecken.

---

## Kurzüberblick C1

| Parameter            | Wert              |
| -------------------- | ----------------- |
| Reichweite           | 0,05–12 m (weiß)  |
| Scanfrequenz         | typisch 10 Hz     |
| Winkelauflösung      | ca. 0,72°         |
| Schnittstelle        | UART 8N1          |
| Baudrate             | **460800**        |
| Versorgung           | 5 V / ~230 mA     |

Das Skript spricht das Slamtec-Protokoll direkt (STOP, RESET, GET_INFO, GET_HEALTH, SCAN). Die bekannten Python-Pakete `rplidar` / `pyrplidar` sind für den C1 oft unzuverlässig (Hardware-Handshake `dsrdtr` blockiert den Datenstrom).
