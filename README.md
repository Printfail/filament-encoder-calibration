# 🎯 Filament Encoder Calibration

**Automatische `rotation_distance` Kalibrierung für Klipper 3D-Drucker mittels AS5048A Encoder**

[![Hardware](https://img.shields.io/badge/Hardware-Raspberry_Pi_Pico_W-green)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
[![Sensor](https://img.shields.io/badge/Sensor-AS5048A-blue)](https://ams.com/as5048a)
[![Klipper](https://img.shields.io/badge/Klipper-Compatible-red)](https://www.klipper3d.org/)

---

## 📖 Inhaltsverzeichnis

- [Features](#-features)
- [Hardware](#-hardware)
- [Installation](#-installation)
- [Verbindung](#-verbindung)
- [Erste Schritte](#-erste-schritte-nach-installation)
- [Verwendung](#-verwendung)
- [Konfiguration](#-konfiguration)
- [Troubleshooting](#-troubleshooting)
- [Technische Details](#-technische-details)

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

### Komponenten

| Komponente | Beschreibung | Link |
|------------|--------------|------|
| **Raspberry Pi Pico W** | Microcontroller mit BLE Support | [Link](https://www.raspberrypi.com/products/raspberry-pi-pico/) |
| **AS5048A** | 14-bit Magnetischer Encoder | [Link](https://ams.com/as5048a) |
| **Neodym-Magnet** | 6mm x 2mm (diametral magnetisiert) | Amazon/Aliexpress |
| **Encoder-Rad** | 10-15mm Durchmesser (genau messen!) | 3D-gedruckt |
| **SS49E Hall-Sensoren** *(Optional)* | 2x Analog Hall-Sensoren für Filament-Durchmesser | Amazon/Aliexpress |

### Anschluss-Schema

#### AS5048A Encoder (Pflicht)
```
Raspberry Pi Pico W          AS5048A Encoder
┌────────────────┐          ┌──────────────┐
│                │          │              │
│  GP16 (MISO) ◄─┼──────────┤ MISO         │
│  GP17 (CS)   ◄─┼──────────┤ CS           │
│  GP18 (CLK)  ◄─┼──────────┤ CLK          │
│  GP19 (MOSI) ◄─┼──────────┤ MOSI         │
│                │          │              │
│  3.3V        ◄─┼──────────┤ VCC          │
│  GND         ◄─┼──────────┤ GND          │
│                │          │              │
└────────────────┘          └──────────────┘
```

#### SS49E Hall-Sensoren (Optional - für Filament-Durchmesser)
```
Raspberry Pi Pico W          SS49E Sensor 1    SS49E Sensor 2
┌────────────────┐          ┌──────────────┐  ┌──────────────┐
│                │          │              │  │              │
│  GP26 (ADC0) ◄─┼──────────┤ Signal       │  │              │
│  GP27 (ADC1) ◄─┼──────────┼──────────────┼──┤ Signal       │
│                │          │              │  │              │
│  3.3V        ◄─┼──────────┤ VCC          ├──┤ VCC          │
│  GND         ◄─┼──────────┤ GND          ├──┤ GND          │
│                │          │              │  │              │
└────────────────┘          └──────────────┘  └──────────────┘
```

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

```bash
# Build Firmware (WSL oder Linux):
cd encoder-pico
mkdir build && cd build
cmake ..
make -j4

# Flash:
# 1. Halte BOOTSEL Button auf Pico
# 2. Stecke USB ein
# 3. Kopiere encoder_calibration.uf2 auf RPI-RP2 Laufwerk
```

**Windows:**
```powershell
copy encoder_calibration.uf2 E:\
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

## 📜 G-Code Befehle

| Befehl | Beschreibung | Parameter |
|--------|--------------|-----------|
| `ENCODER_STATUS` | Zeigt Verbindungsstatus | - |
| `ENCODER_GET_POSITION` | Liest aktuelle Position | - |
| `ENCODER_RESET_POSITION` | Setzt Position auf 0 | - |
| `START_ENCODER_CALIBRATION` | Startet Auto-Kalibrierung | `TOLERANCE`, `MAX_ITERATIONS`, `LENGTH` |
| `ENCODER_CONNECTION_TEST` | Testet BLE Verbindung | - |

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
