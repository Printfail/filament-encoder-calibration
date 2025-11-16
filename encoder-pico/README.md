# 🎯 Filament Encoder Calibration - Pico W Firmware

Automatische rotation_distance Kalibrierung für 3D-Drucker mit AS5048A Encoder.

[![Hardware](https://img.shields.io/badge/MCU-Raspberry_Pi_Pico_W-green)](https://www.raspberrypi.com/products/raspberry-pi-pico/)
[![Sensor](https://img.shields.io/badge/Sensor-AS5048A-blue)](https://ams.com/as5048a)
[![Interface](https://img.shields.io/badge/Interface-SPI-orange)](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface)

## 🔧 Hardware

- **MCU:** Raspberry Pi Pico W (RP2040)
- **Encoder:** AS5048A (14-bit magnetischer Encoder, 16384 steps/rev)
- **Kommunikation:** Bluetooth LE (BLE) via CYW43 Chip
- **Interface:** SPI (4-wire)

## ✨ Features

- ✅ **AS5048A Encoder Support** (14-bit, SPI @ 1MHz)
- ✅ **Filament-Distanz Tracking** in mm mit 0.008mm Präzision
- ✅ **Bluetooth LE GATT Service** mit 5 Characteristics
- ✅ **Überlauf-Erkennung** (unbegrenzte Distanz, 360°+ Tracking)
- ✅ **Dynamischer Wellendurchmesser** via BLE konfigurierbar
- ✅ **Echtzeit-Geschwindigkeit** (mm/s) @ 20Hz Update-Rate
- 🆕 **Sensor Diagnostics** (Magnitude, AGC für Alignment-Check)
- 🆕 **Hall-Sensor ADC** (2x SS49E für Filament-Durchmesser via GPIO 26+27)
- ✅ **Adaptive Noise Calibration** (100 Samples beim Start)
- ✅ **Auto-Reconnect** nach BLE Disconnect

---

## 📌 Pin-Belegung

| Pico Pin | Funktion | AS5048A Pin | Beschreibung |
|----------|----------|-------------|--------------|
| **GP16** | SPI0 MISO | DO | Data Out (Encoder → Pico) |
| **GP17** | SPI0 CS | CS | Chip Select (active low) |
| **GP18** | SPI0 CLK | CLK | Clock Signal |
| **GP19** | SPI0 MOSI | DI | Data In (Pico → Encoder) |
| **GP26** | ADC0 | SS49E #1 Signal | Hall-Sensor 1 (Optional) |
| **GP27** | ADC1 | SS49E #2 Signal | Hall-Sensor 2 (Optional) |
| **3.3V** | Power | VDD / VDD5V | Stromversorgung |
| **GND** | Ground | GND | Gemeinsame Masse |

---

## 🔌 Verdrahtung AS5048A

```
Raspberry Pi Pico W          AS5048A Breakout
┌────────────────┐          ┌──────────────┐
│                │          │              │
│  GP16 (MISO) ◄─┼──────────┤ DO           │
│  GP17 (CS)   ◄─┼──────────┤ CS           │
│  GP18 (CLK)  ◄─┼──────────┤ CLK          │
│  GP19 (MOSI) ◄─┼──────────┤ DI           │
│                │          │              │
│  3.3V        ◄─┼──────────┤ VDD / VDD5V  │
│  GND         ◄─┼──────────┤ GND          │
│                │          │              │
└────────────────┘          └──────────────┘

         │
         ▼
    [ MAGNET ]  ← 6mm x 2mm Neodym (diametral magnetisiert)
                  Abstand: 1-2mm zum Sensor
```

### ⚠️ Wichtig:

- **Magnet:** Diametral magnetisiert (N-S gegenüberliegend, NICHT axial!)
- **Abstand:** 1-2mm optimal (0.5-3mm funktioniert)
- **Ausrichtung:** Magnet zentriert über Sensor-Chip
- **Stromversorgung:** 3.3V oder 5V (AS5048A hat Regler)

## 🔨 Build

### Voraussetzungen

#### 1️⃣ Pico SDK installieren

```bash
cd ~
git clone https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
export PICO_SDK_PATH=~/pico-sdk

# Permanent in ~/.bashrc eintragen:
echo 'export PICO_SDK_PATH=~/pico-sdk' >> ~/.bashrc
```

#### 2️⃣ Build Tools installieren

**Linux (Ubuntu/Debian):**
```bash
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

**Windows (WSL2):**
```bash
# WSL Ubuntu/Debian installieren, dann wie Linux
sudo apt update
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
```

**macOS:**
```bash
brew install cmake
brew install --cask gcc-arm-embedded
```

---

### 🏗️ Kompilieren

```bash
cd encoder-pico

# Clean Build (empfohlen):
rm -rf build
mkdir build
cd build

# CMake konfigurieren (WICHTIG: PICO_BOARD=pico_w!)
cmake ..

# Kompilieren (4 Cores parallel):
make -j4
```

**Output:** `build/encoder_calibration.uf2` (~812KB)

#### Häufige Build-Fehler:

**Fehler:** `pico/cyw43_arch.h: No such file`
```bash
# Lösung: PICO_BOARD nicht gesetzt
# CMakeLists.txt prüfen:
grep "PICO_BOARD" CMakeLists.txt
# Sollte sein: set(PICO_BOARD pico_w CACHE STRING "Board type")
```

**Fehler:** `PICO_SDK_PATH not set`
```bash
export PICO_SDK_PATH=~/pico-sdk
```

## 📤 Flash

### 1️⃣ BOOTSEL-Modus aktivieren

```
1. Halte BOOTSEL-Button auf Pico W
2. Stecke USB-Kabel ein (oder drücke RESET)
3. Lasse BOOTSEL los
4. Pico erscheint als USB-Laufwerk (RPI-RP2)
```

### 2️⃣ Firmware kopieren

**Linux/macOS:**
```bash
cp build/encoder_calibration.uf2 /media/$USER/RPI-RP2/
```

**Windows (PowerShell):**
```powershell
copy build\encoder_calibration.uf2 E:\
```

**Windows (WSL):**
```bash
cp build/encoder_calibration.uf2 /mnt/e/
```

### 3️⃣ Automatischer Neustart

Pico startet automatisch nach dem Kopieren und bootet die neue Firmware.

## ⚙️ Konfiguration

### Compile-Time Einstellungen

**In `src/main.cpp`:**

```cpp
#define WHEEL_DIAMETER_MM 10.0f   // Durchmesser der Encoder-Welle (Default)
#define ENCODER_DIRECTION 1       // 1 oder -1 (invertieren)
#define ENCODER_UPDATE_INTERVAL_MS 50  // Update-Rate: 50ms = 20Hz
```

**⚠️ Hinweis:** `WHEEL_DIAMETER_MM` wird automatisch von Klipper via BLE überschrieben!

### Runtime Config (via BLE)

Der Wheel-Durchmesser kann **ohne Neukompilierung** via BLE geändert werden:

```python
# In encoder_calibration.cfg:
wheel_diameter: 15.0  # ← Wird automatisch zum Pico gesendet!
```

Der Pico empfängt beim Connect:
```cpp
void on_config_update(float wheel_diameter) {
    g_tracker->set_wheel_diameter(wheel_diameter);
    printf("Config updated: %.3fmm\n", wheel_diameter);
}
```

### SPI Konfiguration

**In `src/encoder/as5048a.hpp`:**

```cpp
static constexpr uint SPI_INSTANCE = 0;  // SPI0
static constexpr uint PIN_MISO = 16;
static constexpr uint PIN_CS = 17;
static constexpr uint PIN_CLK = 18;
static constexpr uint PIN_MOSI = 19;
static constexpr uint SPI_BAUDRATE = 1000000;  // 1MHz
```

---

## 📡 BLE GATT Characteristics

**Service UUID:** `12345678-1234-1234-1234-123456789ABC`

| Characteristic | UUID | Type | Beschreibung | Format |
|----------------|------|------|--------------|--------|
| **Position** | `...789001` | READ, NOTIFY | Position, Distanz, Geschwindigkeit | 12 bytes: int32 + float + float |
| **Reset** | `...789002` | WRITE | Reset Position auf 0 | 1 byte: 0x01 |
| **Config** | `...789003` | READ, WRITE | Wellendurchmesser (mm) | 4 bytes: float |
| **Diagnostics** 🆕 | `...789004` | READ, NOTIFY | Magnitude, AGC, Flags | 4 bytes: uint16 + uint8 + uint8 |
| **ADC** 🆕 | `...789005` | READ, NOTIFY | Hall-Sensor Werte | 4 bytes: uint16 + uint16 |

**Diagnostics Format:**
```c
Byte 0-1: Magnitude (uint16_t)  // 0-16383, optimal: 3000-4000
Byte 2:   AGC (uint8_t)          // 0-255, optimal: 64-192
Byte 3:   Flags (uint8_t)        // Bit 3: Magnet zu stark, Bit 5: zu schwach
```

**ADC Format:**
```c
Byte 0-1: ADC1 (uint16_t)  // 0-4095, 12-bit from GPIO 26
Byte 2-3: ADC2 (uint16_t)  // 0-4095, 12-bit from GPIO 27
```

---

### BLE UUID Anpassung

**Falls UUIDs bereits verwendet werden, ändere in `gatt/encoder.gatt`:**

```gatt
PRIMARY_SERVICE, GAT_SERVICE_ENCODER
    // Position
    CHARACTERISTIC, GAT_CHAR_POSITION, READ | NOTIFY,
    00002A63-0000-1000-8000-00805F9B34FB  ← Deine UUID
```

**Nach Änderung neu kompilieren!**

## Verwendung

### 1. BLE Verbindung testen

Nach dem Flashen:
- Serial Monitor öffnen (115200 baud)
- Suche nach "Encoder-PicoW" in BLE-Geräten
- MAC-Adresse notieren

### 2. Klipper Integration

Siehe `encoder_calibration.cfg` und `encoder_calibration.py`

### 3. Kalibrierung

```gcode
START_ENCODER_CALIBRATION
```

## 🔧 Troubleshooting

### ❌ "Failed to initialize encoder!"

**Symptom:**
```
ERROR: Failed to initialize encoder!
AS5048A: Error reading diagnostic register
```

**Lösung:**
1. **Prüfe SPI Verkabelung:**
   ```
   GP16 → DO ✅
   GP17 → CS ✅
   GP18 → CLK ✅
   GP19 → DI ✅
   3.3V → VDD ✅
   GND → GND ✅
   ```

2. **Prüfe Magnet:**
   - Abstand: 1-2mm (optimal)
   - **Diametral magnetisiert** (N-S gegenüber, NICHT axial!)
   - Zentriert über Sensor-Chip

3. **AS5048A LED:**
   - Sollte **dauerhaft leuchten** (nicht blinken)
   - Falls aus: Kein Strom oder Breakout defekt

---

### ⚠️ "No magnet detected"

**Symptom:**
```
AS5048A: WARNING - No magnet detected or magnet too weak/strong
```

**Ursachen:**
- ✅ **Normal:** Magnet zu weit (>3mm) oder zu nah (<0.5mm)
- ✅ **Normal:** Falscher Magnettyp (axial statt diametral)
- ✅ **Normal:** Magnet zu schwach (<6mm Neodym)

**Lösung:**
- Verwende **6mm x 2mm Neodym** (diametral magnetisiert)
- Abstand **1-2mm** einstellen
- **Firmware läuft trotzdem weiter!** (Warnung ignorierbar wenn Sensor stabil)

---

### 📡 BLE verbindet nicht

**Symptom:**
```
Device 28:CD:C1:07:90:00 not found
Connection timeout
```

**Lösung:**

1. **BLE Scan vom Host:**
   ```bash
   sudo timeout 10 bluetoothctl scan on
   # Erwarte: [CHG] Device 28:CD:C1:07:90:00 RSSI: -55
   ```

2. **Falls nicht sichtbar:**
   - Pico näher bringen (<1m)
   - Bluetooth neu starten:
     ```bash
     sudo systemctl restart bluetooth
     ```

3. **BLE Konflikt mit anderen Geräten (z.B. Nevermore):**
   - Temporär andere BLE Module deaktivieren
   - Oder: Pico 5s später verbinden lassen (Startup Delay)

---

### 📏 Falsche Distanz-Messung

**Symptom:**
```
Erwartet: 100mm
Gemessen: 67mm  ← 33% Fehler!
```

**Ursache:** `wheel_diameter` falsch!

**Lösung:**
```bash
# Messe Rad-Durchmesser mit Schieblehre!
# Jeder 0.1mm Fehler = 0.67% Messfehler

# In encoder_calibration.cfg:
wheel_diameter: 14.8  # ← Dein echter Wert!

# Firmware_Restart
# Pico empfängt neuen Wert automatisch via BLE
```

**Test:**
```gcode
ENCODER_RESET_POSITION
M83  # Relative Extrusion
G1 E100 F120  # Extrudiere 100mm @ 2mm/s
ENCODER_GET_POSITION
# Sollte ~100mm anzeigen (±1mm)
```

---

### 🔄 Encoder zählt rückwärts

**Lösung:**

**Option 1: Via Klipper Config:**
```ini
[encoder_calibration]
encoder_direction: -1  # ← Invertiert
```

**Option 2: Im Pico Code:**
```cpp
// src/main.cpp:
#define ENCODER_DIRECTION -1
```

---

### 📊 Noise / Jitter in Messung

**Symptom:**
```
Threshold: 15.0 steps (0.029mm)  ← Zu hoch!
Position springt: 0.1mm → -0.05mm → 0.08mm
```

**Ursache:**
- Magnet wackelt (mechanisch nicht fixiert)
- Vibration am Sensor
- Elektromagnetische Störung (Motoren)

**Lösung:**
- Magnet fest im Rad einpressen
- Sensor mechanisch entkoppeln (Gummi-Dämpfer)
- Encoder-Rad ausbalancieren
- Kabel abschirmen

## 📟 Serial Debugging

### Serial Monitor öffnen

**Linux/macOS:**
```bash
# minicom (empfohlen)
sudo minicom -D /dev/ttyACM0 -b 115200
# Beenden: STRG+A dann Q

# screen (alternative)
screen /dev/ttyACM0 115200
# Beenden: STRG+A dann K
```

**Windows:**
```powershell
# PuTTY GUI
# - Connection Type: Serial
# - COM Port: COM3 (Device Manager prüfen!)
# - Speed: 115200

# Oder Python:
python -m serial.tools.miniterm COM3 115200
```

### Erwarteter Output beim Start:

```
========================================
  Encoder Calibration - Pico W
  AS5048A Filament Tracker
========================================

Initializing SPI...
SPI initialized: MISO=16, CLK=18, MOSI=19, CS=17
Initializing AS5048A encoder...
AS5048A: WARNING - No magnet detected or magnet too weak/strong
AS5048A: Initialized successfully via SPI (angle=12606)
Encoder initialized successfully. Current angle: 12603

FilamentTracker: Initialized with diameter=10.000mm, circumference=31.416mm
FilamentTracker: Starting noise calibration (keep still!)...
FilamentTracker: Reset to 0 (encoder at 12606)

Initializing Bluetooth...
GATT: Initializing...
EncoderService: Initialized
GATT: Initialized successfully
Bluetooth initialized

========================================
  System Ready!
  BLE Name: Encoder-PicoW
  Wheel Diameter: 10.000mm
========================================

GATT: Bluetooth ready
BLE MAC Address: 28:CD:C1:07:90:00
GATT: Advertisement size = 18 bytes
GATT: Advertising enabled
GATT: Advertising as 'Encoder-PicoW'

INFO: Magnet detection marginal (AGC) - but sensor is stable!

FilamentTracker: Noise calibration complete!
  Mean: 12605.3, StdDev: 1.32, Threshold: 4.0 steps (0.008mm)

[Warte auf BLE Connection...]

GATT: Client connected
GATT: LE Connection complete
EncoderService: Config update - diameter=15.000mm
CONFIG: Updating wheel diameter to 15.000mm
FilamentTracker: Diameter updated to 15.000mm (circumference=47.124mm)
```

### Debug-Ausgaben

**Bei Encoder-Bewegung (50ms Updates):**
```
steps=1024, dist=1.920mm, speed=12.50mm/s
steps=2048, dist=3.840mm, speed=38.40mm/s
steps=3072, dist=5.760mm, speed=38.40mm/s
```

**Bei BLE Disconnect:**
```
GATT: Client disconnected
GATT: Advertising enabled  ← Automatisches Re-Advertisement
```

---

## 📜 Lizenz

GNU General Public License v3.0

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.

---

## 🙏 Credits

- **Pico SDK:** [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk)
- **BTstack:** [BlueKitchen BTstack](https://github.com/bluekitchen/btstack)
- **AS5048A:** [ams OSRAM AS5048A Datasheet](https://ams.com/as5048a)
- **Klipper:** [Klipper 3D Printer Firmware](https://www.klipper3d.org/)

---

## 📧 Support

**Issues:** GitHub Issues  
**Discord:** Klipper Discord #encoder-calibration  
**Documentation:** See main [README.md](../README.md)

---

**Made with ❤️ for the 3D Printing Community**
