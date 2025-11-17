# 🎯 Filament Encoder Calibration

**Präzise `rotation_distance` Auto-Kalibrierung für Klipper 3D-Drucker mit AS5048A Magnetic Encoder**

[![Hardware](https://img.shields.io/badge/Hardware-Raspberry_Pi_Pico_W-green)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
[![Sensor](https://img.shields.io/badge/Sensor-AS5048A-blue)](https://ams.com/as5048a)
[![Klipper](https://img.shields.io/badge/Klipper-Compatible-red)](https://www.klipper3d.org/)
[![License](https://img.shields.io/badge/License-GPL_v3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

---

## 💡 Was macht dieses Projekt?

Dieses System **misst die tatsächlich extrudierte Filament-Länge** mit einem hochpräzisen Encoder und **kalibriert automatisch** Klippers `rotation_distance` Parameter. 

**Warum ist das wichtig?**
- ✅ Extruder-Steps sind oft **ungenau** (Getriebe-Spiel, Compression)
- ✅ Manuelles Messen ist **fehleranfällig** (Lineal-Methode ±5%)
- ✅ Dieser Encoder liefert **±0.01mm Genauigkeit** (14-bit Auflösung)
- ✅ **Vollautomatisch** - einfach `START_ENCODER_CALIBRATION` ausführen!

**Das System:**
1. Extrudiert 100mm Filament @ 2mm/s
2. Encoder misst tatsächliche Länge (z.B. 97.3mm)
3. Berechnet neue `rotation_distance` automatisch
4. Wiederholt bis ±1% Genauigkeit erreicht
5. Speichert in `printer.cfg` ✅

**Zusätzlich:**
- 🔧 Sensor-Alignment-Test (Magnet-Zentrierung prüfen)
- 📊 Live-Diagnostics (Echtzeit Magnetfeld-Überwachung)
- 📏 Hall-Sensor Integration (Filament-Durchmesser-Messung)
- 📡 Bluetooth LE (kabellos zwischen Pico W ↔ Raspberry Pi)

---

## ⚡ Quick Start

**Neu hier? Folge diesem 5-Schritte-Plan:**

1. 📦 **[Hardware kaufen](#-stückliste-bill-of-materials)** - ~20-25€, dauert 1 Woche Lieferzeit
2. 🔌 **[Verkabeln](#-verkabelung)** - 6 Kabel verbinden (10 Minuten)
3. � **[Firmware flashen](#1️⃣-pico-w-firmware-flashen)** - BOOTSEL drücken, UF2 kopieren (2 Minuten)
4. ⚙️ **[Klipper installieren](#2️⃣-klipper-module-installieren)** - Python + Config kopieren (5 Minuten)
5. 🎯 **[Erste Kalibrierung](#2️⃣-erste-kalibrierung-durchführen)** - `START_ENCODER_CALIBRATION` (5 Minuten)

**Gesamt-Zeit:** ~30 Minuten Setup + 1 Woche Lieferzeit

---

## � Inhaltsverzeichnis

- [Quick Start](#-quick-start)
- [Was macht dieses Projekt?](#-was-macht-dieses-projekt)
- [Features](#-features)
- [Hardware](#-hardware)
  - [Stückliste](#-stückliste-bill-of-materials)
  - [Verkabelung](#-verkabelung)
- [Installation](#-installation)
  - [Pico W flashen](#1️⃣-pico-w-firmware-flashen)
  - [Klipper installieren](#2️⃣-klipper-module-installieren)
- [Erste Schritte](#-erste-schritte-nach-installation)
- [Verwendung](#-verwendung)
- [G-Code Makros & Befehle](#-g-code-makros--befehle)
- [Konfiguration](#️-konfiguration)
- [Troubleshooting](#-troubleshooting)
- [Best Practices](#-best-practices)

---

## ✨ Features

✅ **Automatische Kalibrierung** - Präzise `rotation_distance` Messung in 1-5 Iterationen  
✅ **Bluetooth LE** - Kabellose Verbindung zwischen Pico W und Raspberry Pi  
✅ **Plug & Play** - Automatische Verbindung und Reconnect nach Stromverlust  
✅ **Präzisions-Encoder** - AS5048A mit 14-bit Auflösung (16384 steps/rev)  
✅ **Adaptive Noise Calibration** - Filtert Sensor-Rauschen automatisch  
✅ **Echtzeit-Feedback** - Live Filament-Geschwindigkeit und Position  
✅ **Iterative Verfeinerung** - Wiederholt Messung bis Toleranz erreicht  
🆕 **Sensor Alignment Tool** - Automatischer Test zur optimalen Sensor-Positionierung  
🆕 **Live Diagnostics** - Echtzeit Magnetfeld-Überwachung für perfekte Montage  
🆕 **Hall-Sensor Integration** - SS49E Filament-Durchmesser-Messung via virtuelle ADC-Pins  
🆕 **Automatische Extrusion-Anpassung** - Kompatibel mit Klipper's `hall_filament_width_sensor`  

---

## 🛠️ Hardware

### 📦 Stückliste (Bill of Materials)

| # | Komponente | Beschreibung | Menge | Kosten (ca.) | Bezugsquelle |
|---|------------|--------------|-------|--------------|--------------|
| 1 | **Raspberry Pi Pico W** | RP2040 Microcontroller mit WiFi/BLE | 1x | ~7€ | [Reichelt](https://www.reichelt.de/) / [Berrybase](https://www.berrybase.de/) |
| 2 | **AS5048A Breakout Board** | 14-bit magnetischer Encoder (SPI) | 1x | ~8€ | [AliExpress](https://aliexpress.com/) / [eBay](https://ebay.de/) |
| 3 | **Neodym-Magnet** | 6mm Ø × 2mm, **diametral magnetisiert** | 1x | ~2€ | [Amazon](https://www.amazon.de/) / [AliExpress](https://aliexpress.com/) |
| 4 | **Encoder-Rad** | 10-15mm Durchmesser (STL im Repo) | 1x | ~0€ | Selbst 3D-drucken |
| 5 | **Dupont-Kabel** | Female-Female, ~15cm | 6x | ~3€ | [Amazon](https://www.amazon.de/) / [Reichelt](https://www.reichelt.de/) |
| 6 | **SS49E Hall-Sensor** *(Optional)* | Analog Linear Hall-Sensor | 2x | ~3€ | [AliExpress](https://aliexpress.com/) / [eBay](https://ebay.de/) |
| 7 | **USB-C Kabel** | Für Pico W Stromversorgung | 1x | ~5€ | [Amazon](https://www.amazon.de/) |
| | | | | **TOTAL:** | **~20-25€** |

**⚠️ MAGNET-HINWEIS:** Achte darauf dass der Magnet **diametral** (North-South gegenüber) und **NICHT axial** (North oben, South unten) magnetisiert ist!

**🔍 Suchbegriffe:**
- Amazon/AliExpress: "AS5048A SPI magnetic encoder"
- Magnet: "6x2mm diametral neodymium magnet" oder "D6x2mm N S pole"

### Verkabelung

#### 🔌 AS5048A Encoder (Pflicht)

| Pico W Pin | GPIO | Funktion | → | AS5048A Pin | Beschreibung |
|-----------|------|----------|---|-------------|--------------|
| Pin 21 | GP16 | SPI0 RX (MISO) | → | **MISO** | Daten vom Sensor |
| Pin 22 | GP17 | SPI0 CSn | → | **CS** | Chip Select (LOW = aktiv) |
| Pin 24 | GP18 | SPI0 SCK | → | **CLK** | Takt (1 MHz) |
| Pin 25 | GP19 | SPI0 TX (MOSI) | → | **MOSI** | Daten zum Sensor |
| Pin 36 | 3V3(OUT) | Stromversorgung | → | **VCC** | 3.3V (100mA max) |
| Pin 38 | GND | Ground | → | **GND** | Masse |

**⚠️ WICHTIG:** Verwende 3.3V, **NICHT** 5V! AS5048A ist 3.3V only!

---

#### 🔌 SS49E Hall-Sensoren (Optional - für Filament-Durchmesser)

| Pico W Pin | GPIO | Funktion | → | SS49E #1 | SS49E #2 |
|-----------|------|----------|---|----------|----------|
| Pin 31 | GP26 | ADC0 | → | **Signal** | - |
| Pin 32 | GP27 | ADC1 | → | - | **Signal** |
| Pin 36 | 3V3(OUT) | Stromversorgung | → | **VCC** | **VCC** |
| Pin 38 | GND | Ground | → | **GND** | **GND** |

**💡 TIPP:** Hall-Sensoren gegenüber montieren (90° versetzt) für beste Messung!

### Mechanischer Aufbau

```
Filament
   │
   ▼
┌─────────────┐
│  Extruder   │
│             │
└──────┬──────┘
       │
    ╔══╧══╗ ← Encoder-Rad (Durchmesser genau messen!)
    ║     ║
    ║  🧲 ║ ← Magnet (im Rad eingebettet)
    ╚═════╝
       │
    [ AS5048A ] ← Sensor (1-2mm Abstand zum Magnet)
```

**⚠️ WICHTIG:** Magnet muss **diametral magnetisiert** sein (N-S gegenüberliegend)!

---

## 📥 Installation

### 1️⃣ Pico W Firmware flashen

#### Kompilierte Firmware verwenden (EINFACH ✅)

1. **Download:** `encoder_calibration.uf2` aus dem `build/` Ordner
2. **BOOTSEL drücken:** Halte den BOOTSEL-Button auf dem Pico W gedrückt
3. **USB einstecken:** Verbinde Pico W mit PC (während BOOTSEL gedrückt)
4. **Laufwerk erscheint:** Pico W als "RPI-RP2" USB-Laufwerk erkannt
5. **Firmware kopieren:** Drag & Drop `encoder_calibration.uf2` auf Laufwerk
6. **Automatischer Neustart:** Pico bootet automatisch mit neuer Firmware
7. **✅ Fertig!** LED sollte blinken, Pico sendet BLE Advertisement

#### Selbst kompilieren (FORTGESCHRITTEN)

**Voraussetzungen:**
- WSL2 (Windows) oder Linux
- Pico SDK installiert

**Build-Schritte:**
```bash
# Pico SDK Path setzen
export PICO_SDK_PATH=/home/user/pico-sdk

# In encoder-pico Ordner wechseln
cd encoder-pico

# Build-Ordner erstellen
mkdir build && cd build

# CMake konfigurieren
cmake ..

# Kompilieren (4 Threads)
make -j4

# Resultat: build/encoder_calibration.uf2
```

**Windows (PowerShell):**
```powershell
# UF2 kopieren (E: = RPI-RP2 Laufwerk)
copy encoder_calibration.uf2 E:\
```

**Linux/WSL:**
```bash
# UF2 kopieren
cp encoder_calibration.uf2 /media/username/RPI-RP2/
```

---

### 2️⃣ Klipper Module installieren

```bash
# SSH zum Raspberry Pi:
ssh pi@mainsailos.local

# Python Dependencies installieren:
~/klippy-env/bin/pip install bleak

# Config-Dateien kopieren:
cd ~/printer_data/config
```

**Von Windows:**
```bash
scp encoder_calibration.py pi@mainsailos.local:~/klipper/klippy/extras/
scp encoder_calibration.cfg pi@mainsailos.local:~/printer_data/config/
```

---

### 3️⃣ Klipper Config anpassen

**In `printer.cfg` einfügen:**
```ini
[include encoder_calibration.cfg]
```

**In `encoder_calibration.cfg` anpassen:**
```ini
[encoder_calibration]
ble_address: 28:CD:C1:07:90:00  # ← Deine Pico MAC-Adresse!
wheel_diameter: 15.0             # ← Durchmesser genau messen!
```

**Klipper neu starten:**
```bash
sudo systemctl restart klipper
```

---

### 4️⃣ Automatische Updates (Optional)

**Moonraker Update Manager einrichten:**

Füge in `moonraker.conf` hinzu:
```ini
[update_manager encoder_calibration]
type: git_repo
channel: dev
path: ~/filament-encoder-calibration
origin: https://github.com/Printfail/filament-encoder-calibration.git
managed_services: klipper
primary_branch: main
install_script: install.sh
```

**Dann:**
```bash
sudo systemctl restart moonraker
```

**Ab jetzt:** Updates erscheinen automatisch in Mainsail/Fluidd unter "Machine" → "Update Manager" 🎉

---

## 🔗 Verbindung

### Automatische Verbindung

Das System verbindet **automatisch** beim Klipper-Start:

```
1. Klipper startet
   ↓
2. Encoder-Modul lädt
   ↓
3. Wartet 5 Sekunden (für andere BLE Geräte)
   ↓
4. Sucht nach Pico W (28:CD:C1:07:90:00)
   ↓
5. Verbindet automatisch
   ↓
6. Synchronisiert Config (wheel_diameter)
   ↓
7. ✅ "Encoder ready - calibrated and connected!"
```

### Status prüfen

```gcode
ENCODER_STATUS
```

**Ausgabe:**
```
════════════════════════════════════════
📊  ENCODER STATUS
════════════════════════════════════════
BLE Address: 28:CD:C1:07:90:00
Connected: ✅ Yes
Wheel Diameter: 15.000mm
Tolerance: ±1.0%
Max Iterations: 5
════════════════════════════════════════
```

---

## 🎯 Erste Schritte nach Installation

### 1️⃣ Sensor-Ausrichtung prüfen (WICHTIG!)

**Bevor du kalibrierst, stelle sicher dass der Sensor optimal ausgerichtet ist:**

```gcode
ENCODER_ALIGNMENT_TEST
```

**Ablauf:**
1. Befehl starten
2. Encoder-Rad 10 Sekunden langsam per Hand drehen
3. Ergebnis auswerten:
   - ✅ **Variation < 50**: Perfekt! Weiter mit Kalibrierung
   - ⚠️ **Variation 50-200**: Gut genug, aber optimierbar
   - ❌ **Variation > 200**: Sensor neu positionieren!

**Bei schlechter Ausrichtung:**
- Prüfe ob Magnet mittig auf der Welle sitzt
- Prüfe Abstand Sensor ↔ Magnet (2-4mm optimal)
- Nutze `ENCODER_DIAGNOSTICS_LIVE` für Echtzeit-Feedback beim Justieren

---

### 2️⃣ Erste Kalibrierung durchführen

**Wenn Alignment ✅ ist:**

```gcode
# Hotend aufheizen:
M104 S210
M109 S210

# Kalibrierung starten:
START_ENCODER_CALIBRATION
```

**System kalibriert vollautomatisch!**

---

## 🚀 Verwendung

### Vollautomatische Kalibrierung

```gcode
# 1. Hotend aufheizen:
M104 S210
M109 S210

# 2. Kalibrierung starten:
START_ENCODER_CALIBRATION

# Optional: Parameter überschreiben:
START_ENCODER_CALIBRATION TOLERANCE=1.0 MAX_ITERATIONS=5 LENGTH=100
```

**Was passiert:**
1. ✅ Encoder Position wird auf 0 zurückgesetzt
2. ✅ System extrudiert 100mm Filament @ 2mm/s
3. ✅ Encoder misst tatsächliche Distanz (z.B. 97.3mm)
4. ✅ Berechnet neue `rotation_distance`:
   ```
   neue_rd = alte_rd × (SOLL / IST)
   neue_rd = 46.78 × (100 / 97.3) = 48.09
   ```
5. ✅ Wendet neuen Wert an
6. ✅ Wiederholt bis Toleranz erreicht (±1%)
7. ✅ Speichert automatisch in `printer.cfg`!

---

### Manuelle Position ablesen

```gcode
ENCODER_GET_POSITION
```

**Ausgabe:**
```
Encoder Position: 62.832mm (3277 steps) | Speed: 0.00mm/s
```

---

### Encoder zurücksetzen

```gcode
ENCODER_RESET_POSITION
```

**Setzt Position auf 0 - nützlich vor manuellen Tests.**

---

### 🔧 Sensor-Ausrichtung testen (NEU!)

```gcode
ENCODER_ALIGNMENT_TEST
```

**Was passiert:**
1. ✅ Misst Magnetfeld-Stärke (Magnitude) für 10 Sekunden
2. ✅ Du drehst das Encoder-Rad langsam per Hand
3. ✅ System berechnet Variation der Magnitude
4. ✅ Gibt Bewertung aus:
   - ✅ **PERFEKT** (Variation < 50) - Sensor optimal zentriert
   - ✅ **GUT** (Variation < 200) - Akzeptabel
   - ⚠️ **MITTEL** (Variation < 500) - Könnte besser sein
   - ❌ **SCHLECHT** (Variation > 500) - Sensor ist exzentrisch!

**Ausgabe:**
```
📊 ALIGNMENT ERGEBNIS:
━━━━━━━━━━━━━━━━━━━━━━━━━━
Magnitude:
  Min: 3498  Max: 3507  Variation: 9
  Durchschnitt: 3502.3  StdDev: 2.8
  AGC Durchschnitt: 128

✅ PERFEKT ZENTRIERT!
Sensor ist optimal ausgerichtet.
```

**Wofür?**
- Hilft beim Montieren des Sensors
- Zeigt ob Magnet mittig auf der Welle sitzt
- Prüft ob Abstand Sensor ↔ Magnet optimal ist

---

### 🔍 Live Diagnostics anzeigen (NEU!)

```gcode
ENCODER_DIAGNOSTICS_LIVE    # Start
ENCODER_DIAGNOSTICS_LIVE    # Stop (Toggle)
```

**Was passiert:**
- Zeigt **Magnitude** (Magnetfeld-Stärke) live an
- Zeigt **AGC** (Automatic Gain Control) live an
- Warnt bei zu schwachem/starkem Magnet
- Toggle: Nochmal aufrufen zum Stoppen

**Ausgabe:**
```
Mag: 3502 | AGC: 128 | ✅
Mag: 3498 | AGC: 127 | ✅
Mag: 4200 | AGC: 156 | ⚠️ ZU STARK
```

**Wofür?**
- Echtzeit-Feedback beim Justieren
- Sensor-Position optimieren
- Prüfen ob Magnet erkannt wird

---

### 📏 Filament-Durchmesser Messung (Optional - NEU!)

**Hardware:** 2x SS49E Hall-Sensoren an GPIO 26 + 27

**Setup:**
```ini
# In printer.cfg oder separate Datei:
[hall_filament_width_sensor]
adc1: encoder:adc1  # ← Virtueller Pin vom Pico!
adc2: encoder:adc2  # ← Virtueller Pin vom Pico!
cal_dia1: 1.50
cal_dia2: 2.00
raw_dia1: 9500
raw_dia2: 10500
default_nominal_filament_diameter: 1.75
max_difference: 0.200
measurement_delay: 70
enable: False
```

**Kalibrierung:**
```gcode
# 1. Filament mit 1.50mm einlegen
QUERY_RAW_FILAMENT_WIDTH
# Notiere RAW-Wert (z.B. 9500)

# 2. Filament mit 2.00mm einlegen
QUERY_RAW_FILAMENT_WIDTH
# Notiere RAW-Wert (z.B. 10500)

# 3. Werte in Config eintragen
```

**Verwendung:**
```gcode
ENABLE_FILAMENT_WIDTH_SENSOR   # Aktivieren
ENABLE_FILAMENT_WIDTH_LOG      # Logging aktivieren
QUERY_FILAMENT_WIDTH           # Aktuellen Durchmesser anzeigen
```

**Was passiert:**
- Pico liest SS49E Sensoren (12-bit ADC)
- Sendet Werte via BLE
- Klipper berechnet Filament-Durchmesser
- Passt automatisch Extrusion an (M221)

**Beispiel:**
```
Filament: 1.70mm (statt 1.75mm)
→ Extrusion wird um ~3% reduziert
```

---

## ⚙️ Konfiguration

### encoder_calibration.cfg

```ini
[encoder_calibration]
# BLE Verbindung
ble_address: 28:CD:C1:07:90:00  # MAC des Pico W (schnellere Verbindung!)
connection_timeout: 30          # Sekunden

# Hardware-Parameter
encoder_resolution: 16384       # AS5048A: 14-bit = 16384 steps/rev
wheel_diameter: 15.0            # mm - GENAU MESSEN mit Schieblehre!

# Kalibrierungs-Parameter
extrude_length: 100             # mm - Filament pro Iteration
extrude_speed: 2                # mm/s - Langsam für Genauigkeit
tolerance_percent: 1.0          # % - ±1% = 99-101mm bei 100mm
max_iterations: 5               # Maximale Versuche

# Safety-Parameter
min_sane_value: 50              # mm - Minimum bei 100mm SOLL
max_sane_value: 150             # mm - Maximum bei 100mm SOLL
iteration_delay: 2.0            # Sekunden zwischen Iterationen

# Erweiterte Einstellungen
encoder_direction: 1            # 1 oder -1 (Richtung invertieren)
```

---

## 🔧 Troubleshooting

### Problem: "Device not found"

**Ursache:** BLE Advertisement nicht sichtbar

**Lösung:**
```bash
# 1. Prüfe ob Pico läuft (Serial Monitor):
sudo minicom -D /dev/ttyACM0 -b 115200
# Erwarte: "Advertising as 'Encoder-PicoW'"

# 2. BLE Scan vom Pi:
sudo timeout 10 bluetoothctl scan on
# Erwarte: [CHG] Device 28:CD:C1:07:90:00 RSSI: -55

# 3. Falls nicht sichtbar:
# - Pico näher an Pi bringen (<1m)
# - Bluetooth neu starten:
sudo systemctl restart bluetooth
```

---

### Problem: "Connection timeout"

**Ursache:** Mehrere BLE Module konkurrieren (z.B. Nevermore)

**Lösung 1:** Startup-Delay bereits implementiert (5 Sekunden)

**Lösung 2:** Temporär andere BLE Module deaktivieren:
```ini
# In printer.cfg:
# [include nevermore.cfg]  ← Auskommentieren
[include encoder_calibration.cfg]
```

---

### Problem: Encoder zeigt falsche Werte

**Ursache:** `wheel_diameter` falsch

**Lösung:**
```bash
# Messe Rad-Durchmesser mit Schieblehre!
# Jeder 0.1mm Fehler = 0.67% Fehler in Messung

# Update in encoder_calibration.cfg:
wheel_diameter: 14.8  # ← Dein echter Wert!
```

---

### Problem: "No magnet detected"

**Ursache:** Magnet zu schwach, zu weit, oder falsche Polarität

**Lösung:**
- ✅ Magnet 1-2mm vom Sensor entfernt
- ✅ Diametral magnetisiert (N-S gegenüber)
- ✅ Stark genug (mind. 6mm x 2mm Neodym)

**Test:**
```bash
# Serial Monitor öffnen:
# Erwarte: "AS5048A: Initialized successfully"
# NICHT: "AS5048A: ERROR"
```

---

## 📊 Technische Details

### BLE Charakteristiken

| UUID | Name | Typ | Beschreibung |
|------|------|-----|--------------|
| `00002A63-...` | Position | Read (12 bytes) | `steps (int32)` + `distance_mm (float)` + `speed_mm_s (float)` |
| `00002A64-...` | Reset | Write (1 byte) | Setzt Position auf 0 |
| `00002A65-...` | Config | Write (4 bytes) | `wheel_diameter (float)` |

---

### Noise Calibration

**Beim Start kalibriert der Encoder automatisch:**
```cpp
// Sammelt 100 Samples im Stillstand
// Berechnet Mean & StdDev
// Setzt Threshold = 3 × StdDev

Beispiel:
  Mean: 12605.3
  StdDev: 1.32
  Threshold: 4.0 steps (0.008mm)
```

**Nur Bewegungen > Threshold werden gezählt!**

---

### Geschwindigkeits-Berechnung

```cpp
float calculate_speed() {
    uint64_t now = time_us_64();
    uint64_t dt = now - last_update_time_us_;
    
    float distance_delta = distance_mm_ - last_distance_mm_;
    float time_delta_s = dt / 1000000.0;
    
    speed_mm_per_sec_ = distance_delta / time_delta_s;
}
```

**Update-Rate:** 50ms (20Hz)

---

### Überlauf-Erkennung

```cpp
// AS5048A zählt 0 → 16383 → 0 (14-bit)
int16_t detect_overflow(uint16_t current_angle) {
    int32_t delta = current_angle - last_angle_;
    
    // Überlauf vorwärts: 16383 → 0
    if (delta < -8192) return +1;
    
    // Überlauf rückwärts: 0 → 16383
    if (delta > +8192) return -1;
    
    return 0;
}
```

---

## 📜 G-Code Makros & Befehle

### Kalibrierung

| Befehl | Beschreibung | Parameter | Beispiel |
|--------|--------------|-----------|----------|
| `START_ENCODER_CALIBRATION` | **Automatische rotation_distance Kalibrierung** - Extrudiert Filament und misst tatsächliche Länge | `TOLERANCE=1.0` `MAX_ITERATIONS=5` `LENGTH=100` | `START_ENCODER_CALIBRATION` |
| `CALIBRATE_ENCODER_WHEEL` | **Kalibriert Rad-Durchmesser** - Extrudiert bekannte Länge und berechnet Durchmesser | `LENGTH=100` | `CALIBRATE_ENCODER_WHEEL LENGTH=100` |
| `ENCODER_CALIBRATE_WHEEL_DIRECT` | **Manuelle Rad-Kalibrierung** - Ohne Heizen! Filament per Hand durchschieben | `LENGTH=100` | `ENCODER_CALIBRATE_WHEEL_DIRECT LENGTH=100` |
| `ENCODER_CALIBRATE_WHEEL_DIRECT_DONE` | **Beendet manuelle Kalibrierung** - Berechnet Durchmesser aus gemessener Länge | - | `ENCODER_CALIBRATE_WHEEL_DIRECT_DONE` |
| `SAVE_ENCODER_WHEEL_DIAMETER` | **Speichert Rad-Durchmesser** in Config | - | `SAVE_ENCODER_WHEEL_DIAMETER` |

### Position & Status

| Befehl | Beschreibung | Parameter | Beispiel |
|--------|--------------|-----------|----------|
| `ENCODER_READ` | **Zeigt aktuelle Position** - Steps, mm, Geschwindigkeit | - | `ENCODER_READ` |
| `ENCODER_ZERO` | **Position auf 0 zurücksetzen** - Nützlich vor manuellen Tests | - | `ENCODER_ZERO` |
| `ENCODER_STATUS` | **System-Status anzeigen** - BLE-Verbindung, Config, Diagnostics | - | `ENCODER_STATUS` |
| `ENCODER_TEST` | **Verbindungstest** - Prüft BLE und Encoder-Funktion | - | `ENCODER_TEST` |

### Sensor-Diagnose (NEU!)

| Befehl | Beschreibung | Parameter | Beispiel |
|--------|--------------|-----------|----------|
| `ENCODER_ALIGNMENT_TEST` | **10-Sekunden Test** - Rad drehen, System misst Magnet-Zentrierung und bewertet Ausrichtung | - | `ENCODER_ALIGNMENT_TEST` |
| `ENCODER_DIAGNOSTICS_LIVE` | **Live-Monitor (Toggle)** - Zeigt Magnitude & AGC in Echtzeit (NUR via SSH/Serial!) | - | `ENCODER_DIAGNOSTICS_LIVE` |

### Filament-Durchmesser (Optional - Hall-Sensor)

| Befehl | Beschreibung | Parameter | Beispiel |
|--------|--------------|-----------|----------|
| `QUERY_FILAMENT_WIDTH` | **Zeigt gemessenen Durchmesser** in mm | - | `QUERY_FILAMENT_WIDTH` |
| `QUERY_RAW_FILAMENT_WIDTH` | **Zeigt RAW ADC-Werte** für Kalibrierung | - | `QUERY_RAW_FILAMENT_WIDTH` |
| `ENABLE_FILAMENT_WIDTH_SENSOR` | **Aktiviert automatische Extrusions-Anpassung** | - | `ENABLE_FILAMENT_WIDTH_SENSOR` |
| `DISABLE_FILAMENT_WIDTH_SENSOR` | **Deaktiviert Sensor** | - | `DISABLE_FILAMENT_WIDTH_SENSOR` |
| `ENABLE_FILAMENT_WIDTH_LOG` | **Aktiviert Console-Logging** | - | `ENABLE_FILAMENT_WIDTH_LOG` |
| `RESET_FILAMENT_WIDTH_SENSOR` | **Setzt Sensor zurück** auf Nominal-Wert | - | `RESET_FILAMENT_WIDTH_SENSOR` |

---

## 🎓 Best Practices

### ✅ DO:
- **Wheel-Durchmesser präzise messen** (Schieblehre!)
- **Filament-Temperatur stabilisieren** (M109 statt M104)
- **Langsame Extrusion** (2mm/s für Genauigkeit)
- **Mehrere Iterationen** (min. 3-5)
- **Pico immer am Strom** lassen

### ❌ DON'T:
- **Nicht** während des Drucks kalibrieren
- **Nicht** mit kaltem Hotend messen
- **Nicht** zu schnell extrudieren (>5mm/s)
- **Nicht** Encoder-Rad berühren während Messung
- **Nicht** Magnet zu weit vom Sensor (>3mm)

---

## 📈 Beispiel-Kalibrierung

```
══════════════════════════════════════
🎯  ENCODER AUTO-KALIBRIERUNG
══════════════════════════════════════
• Extrusions-Länge : 100.0 mm
• Toleranz         : ±1.0%
• Max. Versuche    : 5
══════════════════════════════════════

Iteration 1:
  Soll: 100.00mm | Ist: 97.34mm | Abweichung: -2.66% ❌
  Alte rotation_distance: 46.779
  Neue rotation_distance: 48.088
  Angewendet und teste erneut...

Iteration 2:
  Soll: 100.00mm | Ist: 99.87mm | Abweichung: -0.13% ✅
  ✅ Kalibrierung erfolgreich!
  
Finale rotation_distance: 48.088
Gespeichert in printer.cfg!
══════════════════════════════════════
```

---

## 🤝 Contributing

**Verbesserungen willkommen!**

- Bug Reports: GitHub Issues
- Feature Requests: GitHub Discussions
- Pull Requests: Gerne!

---

## 📝 Lizenz

GNU General Public License v3.0

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.

---

## 🙏 Credits

- **Klipper** - https://www.klipper3d.org/
- **BTstack** - Bluetooth Stack für Pico
- **Bleak** - Python BLE Library
- **Raspberry Pi Pico SDK** - https://github.com/raspberrypi/pico-sdk

---

## 📧 Support

**Bei Fragen:**
- GitHub Issues öffnen
- Klipper Discord: #encoder-calibration

---

**Made with ❤️ for the 3D Printing Community**
