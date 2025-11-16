# Encoder Calibration Module for Klipper
#
# Automatic rotation_distance calibration using AS5048A encoder on Pico W
# Communication via Bluetooth LE
#
# Copyright (C) 2025
# This file may be distributed under the terms of the GNU GPLv3 license.

import asyncio
import logging
import math
import threading
import time
from typing import Optional, Tuple
from uuid import UUID

# Klipper ADC interface
try:
    from . import bus
except ImportError:
    bus = None

try:
    import bleak
    from bleak import BleakClient, BleakScanner
    from bleak.backends.characteristic import BleakGATTCharacteristic
except ImportError:
    raise Exception(
        "Bleak library required for encoder_calibration. "
        "Install with: ~/klippy-env/bin/pip install bleak"
    )


# BLE UUIDs - Encoder Service
SERVICE_UUID = UUID("12345678-1234-1234-1234-123456789abc")
CHAR_UUID_POSITION = UUID("12345678-1234-1234-1234-123456789001")
CHAR_UUID_RESET = UUID("12345678-1234-1234-1234-123456789002")
CHAR_UUID_CONFIG = UUID("12345678-1234-1234-1234-123456789003")
CHAR_UUID_DIAGNOSTICS = UUID("12345678-1234-1234-1234-123456789004")
CHAR_UUID_ADC = UUID("12345678-1234-1234-1234-123456789005")


class EncoderBackground:
    """Background thread that handles BLE communication"""
    
    def __init__(self, name, calibration, ble_address, connection_timeout):
        self.name = name
        self.calibration = calibration
        self.ble_address = ble_address
        self.connection_timeout = connection_timeout
        
        self.log = logging.getLogger(f"encoder_bg.{name}")
        
        # Threading
        self._thread = None
        self._loop = None
        self._connected = threading.Event()
        self._should_stop = threading.Event()
        
        # BLE Client
        self.client = None
        self.char_position = None
        self.char_reset = None
        self.char_config = None
        self.char_diagnostics = None
        self.char_adc = None
    
    def start(self):
        """Start background thread"""
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()
    
    def stop(self):
        """Stop background thread"""
        self._should_stop.set()
        if self._thread:
            self._thread.join(timeout=5.0)
    
    def is_connected(self):
        """Check if BLE is connected"""
        return self._connected.is_set()
        
    def start_calibration(self):
        """Start encoder calibration mode"""
        if not self._connected.is_set() or not self.char_config:
            raise RuntimeError("Not connected to encoder")
            
        # Send command to start calibration
        cmd = b"START_CAL"
        return self._run_in_loop(self._write_characteristic, self.char_config, cmd)
    
    def end_calibration(self):
        """End calibration and save LUT"""
        if not self._connected.is_set() or not self.char_config:
            raise RuntimeError("Not connected to encoder")
            
        # Send command to end calibration and save LUT
        cmd = b"END_CAL"
        return self._run_in_loop(self._write_characteristic, self.char_config, cmd)
    
    def _worker(self):
        """Background worker thread"""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        try:
            self._loop.run_until_complete(self._worker_main())
        except Exception as e:
            self.log.exception("Worker failed", exc_info=e)
        finally:
            self._loop.close()
    
    async def _worker_main(self):
        """Main async worker"""
        # WICHTIG: Initial delay um BLE-Konflikte mit anderen Modulen (z.B. Nevermore) zu vermeiden
        self.log.info("Waiting 5 seconds before BLE scan (avoiding conflicts)...")
        await asyncio.sleep(5)
        
        while not self._should_stop.is_set():
            try:
                await self._connect_and_run()
            except Exception as e:
                self.log.error(f"Connection failed: {e}")
                self._connected.clear()
                
                if not self._should_stop.is_set():
                    self.log.info("Retrying in 10 seconds...")
                    await asyncio.sleep(10)
    
    async def _connect_and_run(self):
        """Connect to encoder and run"""
        self.log.info("Searching for encoder...")
        
        # Discover device
        if self.ble_address:
            device = await BleakScanner.find_device_by_address(
                self.ble_address, timeout=self.connection_timeout
            )
            if not device:
                raise Exception(f"Device {self.ble_address} not found")
        else:
            devices = await BleakScanner.discover(timeout=10.0)
            encoder_devices = [d for d in devices if d.name and "Encoder" in d.name]
            
            if not encoder_devices:
                raise Exception("No encoder devices found")
            if len(encoder_devices) > 1:
                self.log.warning(
                    f"Multiple encoders found. Using first: {encoder_devices[0].address}"
                )
            
            device = encoder_devices[0]
        
        self.log.info(f"Connecting to {device.address}...")
        
        # Connect
        async with BleakClient(device, timeout=self.connection_timeout) as client:
            self.client = client
            self.log.info(f"Connected to {device.address}")
            
            # Get characteristics
            await self._discover_characteristics()
            
            # Mark as connected
            self._connected.set()
            self.log.info("✅ Encoder ready - calibrated and connected!")
            
            # Sende Config zum Pico (wichtig: wheel_diameter muss übereinstimmen!)
            try:
                wheel_diameter = self.calibration.wheel_diameter
                await self.write_config(wheel_diameter)
                self.log.info(f"⚙️  Config synchronized: wheel_diameter={wheel_diameter}mm")
            except Exception as e:
                self.log.error(f"Failed to send config to Pico: {e}")
            
            # Lese erste Position um zu zeigen dass es funktioniert
            try:
                steps, mm, speed = await self.read_position()
                self.log.info(f"📊 Position: {mm:.3f}mm ({steps} steps) | Speed: {speed:.2f}mm/s")
            except Exception as e:
                self.log.warning(f"Initial position read failed: {e}")
            
            # Stay connected until stop
            try:
                while not self._should_stop.is_set() and client.is_connected:
                    await asyncio.sleep(1.0)
            finally:
                self._connected.clear()
                self.log.info("Disconnected")
    
    async def _discover_characteristics(self):
        """Discover GATT characteristics"""
        self.char_position = self.client.services.get_characteristic(CHAR_UUID_POSITION)
        self.char_reset = self.client.services.get_characteristic(CHAR_UUID_RESET)
        self.char_config = self.client.services.get_characteristic(CHAR_UUID_CONFIG)
        self.char_diagnostics = self.client.services.get_characteristic(CHAR_UUID_DIAGNOSTICS)
        self.char_adc = self.client.services.get_characteristic(CHAR_UUID_ADC)
        
        if not self.char_position:
            raise Exception("Position characteristic not found")
        if not self.char_reset:
            raise Exception("Reset characteristic not found")
        if not self.char_config:
            raise Exception("Config characteristic not found")
        if not self.char_diagnostics:
            raise Exception("Diagnostics characteristic not found")
        if not self.char_adc:
            self.log.warning("ADC characteristic not found - hall sensors not available")
        
        self.log.info("All characteristics discovered")
    
    async def read_position(self) -> Tuple[int, float, float]:
        """Read encoder position (steps, mm, speed_mm_per_sec)"""
        if not self.is_connected() or not self.char_position:
            raise Exception("Not connected")
        
        data = await self.client.read_gatt_char(self.char_position)
        
        if len(data) < 12:
            raise Exception(f"Invalid position data length: {len(data)} (expected 12)")
        
        import struct
        # Parse: int32 steps + float mm + float speed_mm_per_sec
        steps = struct.unpack('<i', data[0:4])[0]
        mm = struct.unpack('<f', data[4:8])[0]
        speed_mm_per_sec = struct.unpack('<f', data[8:12])[0]
        
        return steps, mm, speed_mm_per_sec
    
    async def reset_position(self):
        """Reset encoder position to zero"""
        if not self.is_connected() or not self.char_reset:
            raise Exception("Not connected")
        
        # Schreibe 0x01 um Reset auszulösen
        await self.client.write_gatt_char(self.char_reset, bytes([0x01]))
        self.log.info("Position reset to 0")
    
    async def start_diagnostics_serial_mode(self):
        """Start diagnostics serial output on Pico"""
        if not self.is_connected() or not self.char_reset:
            raise Exception("Not connected")
        
        # Schreibe 0x02 um Diagnostics Mode zu starten
        await self.client.write_gatt_char(self.char_reset, bytes([0x02]))
        self.log.info("Diagnostics serial mode STARTED on Pico")
    
    async def stop_diagnostics_serial_mode(self):
        """Stop diagnostics serial output on Pico"""
        if not self.is_connected() or not self.char_reset:
            raise Exception("Not connected")
        
        # Schreibe 0x03 um Diagnostics Mode zu stoppen
        await self.client.write_gatt_char(self.char_reset, bytes([0x03]))
        self.log.info("Diagnostics serial mode STOPPED on Pico")
    
    async def write_config(self, wheel_diameter: float):
        """Write wheel diameter configuration"""
        if not self.is_connected() or not self.char_config:
            raise Exception("Not connected")
        
        import struct
        # Schreibe float (4 bytes, little-endian) wie Pico es erwartet
        data = struct.pack('<f', wheel_diameter)
        await self.client.write_gatt_char(self.char_config, data)
        self.log.info(f"Config sent to Pico: wheel_diameter={wheel_diameter}mm")
    
    async def read_adc(self) -> Tuple[int, int]:
        """Read ADC values from hall sensors (adc1, adc2)"""
        if not self.is_connected() or not self.char_adc:
            raise Exception("Not connected or ADC characteristic not available")
        
        data = await self.client.read_gatt_char(self.char_adc)
        
        if len(data) < 4:
            raise Exception(f"Invalid ADC data length: {len(data)} (expected 4)")
        
        import struct
        # Parse: uint16 adc1 + uint16 adc2 (little-endian, 12-bit values 0-4095)
        adc1 = struct.unpack('<H', data[0:2])[0]
        adc2 = struct.unpack('<H', data[2:4])[0]
        
        return adc1, adc2
    
    async def read_diagnostics(self) -> Tuple[int, int, int]:
        """Read encoder diagnostics (magnitude, agc, diagnostics_flags)"""
        if not self.is_connected() or not self.char_diagnostics:
            raise Exception("Not connected")
        
        data = await self.client.read_gatt_char(self.char_diagnostics)
        
        if len(data) < 4:
            raise Exception(f"Invalid diagnostics data length: {len(data)} (expected 4)")
        
        import struct
        # Parse: uint16 magnitude + uint8 agc + uint8 diagnostics
        magnitude = struct.unpack('<H', data[0:2])[0]
        agc = data[2]
        diagnostics_flags = data[3]
        
        return magnitude, agc, diagnostics_flags
    
    def run_async(self, coro):
        """Run coroutine in background loop"""
        if not self._loop:
            raise Exception("Background loop not running")
        
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=10.0)
    
    async def _write_characteristic(self, char, data):
        """Helper to write to a characteristic"""
        if not self.client or not self._connected.is_set():
            raise RuntimeError("Not connected to encoder")
        return await self.client.write_gatt_char(char.uuid, data, response=True)
    
    def _run_in_loop(self, func, *args, **kwargs):
        """Run a BLE operation in the event loop"""
        if not self._loop or not self._loop.is_running():
            raise RuntimeError("Event loop not running")
        
        future = asyncio.run_coroutine_threadsafe(
            func(*args, **kwargs), self._loop
        )
        return future.result(timeout=10.0)  # 10 second timeout


class EncoderCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.gcode = self.printer.lookup_object("gcode")
        
        # BLE Config
        self.ble_address = config.get("ble_address", None)
        self.connection_timeout = config.getfloat("connection_timeout", 30.0)
        
        # Hardware Config
        self.encoder_resolution = config.getint("encoder_resolution", 16384)
        self.wheel_diameter = config.getfloat("wheel_diameter", 10.0)
        wheel_circ = config.getfloat("wheel_circumference", None)
        self.wheel_circumference = wheel_circ if wheel_circ else (math.pi * self.wheel_diameter)
        self.encoder_direction = config.getint("encoder_direction", 1)
        if self.encoder_direction not in [1, -1]:
            raise config.error("encoder_direction must be 1 or -1")
        
        # Calibration Config
        self.extrude_length = config.getfloat("extrude_length", 100.0)
        self.extrude_speed = config.getfloat("extrude_speed", 2.0)
        self.tolerance_percent = config.getfloat("tolerance_percent", 1.0)
        self.max_iterations = config.getint("max_iterations", 5)
        
        # Safety Config
        self.min_sane_value = config.getfloat("min_sane_value", 50.0)
        self.max_sane_value = config.getfloat("max_sane_value", 150.0)
        self.iteration_delay = config.getfloat("iteration_delay", 2.0)
        
        # Background worker
        self.bg = EncoderBackground(
            self.name, self, self.ble_address, self.connection_timeout
        )
        self.bg._printer = self.printer  # For pin manager
        
        # Register virtual ADC pins for hall sensors
        reactor = self.printer.get_reactor()
        self.encoder_pins = EncoderPins(self.bg, reactor)
        ppins = self.printer.lookup_object('pins')
        ppins.register_chip('encoder', self.encoder_pins)
        
        # Register G-Code commands
        self.gcode.register_command(
            "ENCODER_AUTO_CALIBRATE",
            self.cmd_ENCODER_AUTO_CALIBRATE,
            desc="Start automatic encoder-based calibration",
        )
        self.gcode.register_command(
            "ENCODER_CONNECTION_TEST",
            self.cmd_ENCODER_CONNECTION_TEST,
            desc="Test encoder BLE connection",
        )
        self.gcode.register_command(
            "ENCODER_RESET_POSITION",
            self.cmd_ENCODER_RESET_POSITION,
            desc="Reset encoder position to zero",
        )
        self.gcode.register_command(
            "ENCODER_GET_POSITION",
            self.cmd_ENCODER_GET_POSITION,
            desc="Read current encoder position",
        )
        self.gcode.register_command(
            "ENCODER_PRINT_STATUS",
            self.cmd_ENCODER_PRINT_STATUS,
            desc="Print encoder system status",
        )
        self.gcode.register_command(
            "_ENCODER_ALIGNMENT_TEST",
            self.cmd_ENCODER_ALIGNMENT_TEST,
            desc="Test sensor alignment (10 second test)",
        )
        self.gcode.register_command(
            "_ENCODER_DIAGNOSTICS_LIVE",
            self.cmd_ENCODER_DIAGNOSTICS_LIVE,
            desc="[SSH ONLY] Toggle live diagnostics monitor",
        )
        
        # Add LUT calibration command (with underscore like Auto_Offset pattern)
        self.gcode.register_command(
            "_START_ENCODER_CALIBRATION",
            self.cmd__START_ENCODER_CALIBRATION,
            desc="Start automatic encoder LUT calibration (2 full rotations)",
        )
        
        # Add wheel diameter calibration command
        self.gcode.register_command(
            "CALIBRATE_ENCODER_WHEEL",
            self.cmd_CALIBRATE_ENCODER_WHEEL,
            desc="Calibrate encoder wheel diameter by measuring known filament length",
        )
        
        # Add save wheel diameter command
        self.gcode.register_command(
            "SAVE_ENCODER_WHEEL_DIAMETER",
            self.cmd_SAVE_ENCODER_WHEEL_DIAMETER,
            desc="Save the calibrated wheel diameter to config",
        )
        
        # Initialize pending diameter
        self._pending_wheel_diameter = None
        
        # Start background worker
        self.bg.start()
        
        logging.info(f"Encoder calibration initialized: {self.name}")
    
    # ========================================================================
    # Calibration Logic
    # ========================================================================
    
    def run_calibration(self, tolerance, max_iter, length):
        """Run automatic calibration (blocking)"""
        self._respond(f"{'═' * 40}")
        self._respond("🎯  ENCODER AUTO-KALIBRIERUNG")
        self._respond(f"{'═' * 40}")
        
        if not self.bg.is_connected():
            self._respond_error("Encoder nicht verbunden! Warte...")
            # Wait for connection
            timeout = time.time() + 30
            while not self.bg.is_connected() and time.time() < timeout:
                time.sleep(1)
            
            if not self.bg.is_connected():
                self._respond_error("Verbindung fehlgeschlagen!")
                return
        
        # Get extruder
        toolhead = self.printer.lookup_object("toolhead")
        extruder = toolhead.get_extruder()
        initial_rd = extruder.get_status(0)["rotation_distance"]
        
        self._respond(f"Start rotation_distance: {initial_rd:.6f}")
        
        for iteration in range(1, max_iter + 1):
            self._respond("")
            self._respond(f"{'─' * 40}")
            self._respond(f"Iteration {iteration}/{max_iter}")
            self._respond(f"{'─' * 40}")
            
            # Reset encoder
            try:
                self.bg.run_async(self.bg.reset_position())
            except Exception as e:
                self._respond_error(f"Encoder reset failed: {e}")
                return
            
            time.sleep(0.5)
            
            # Extrudiere
            self._respond(f"Extrudiere {length}mm...")
            self.gcode.run_script_from_command(
                f"EXTRUDER_CALIBRATION DIST={length} SPEED={self.extrude_speed}"
            )
            
            # Warte
            estimated_time = length / self.extrude_speed + 2.0
            time.sleep(estimated_time)
            
            # Lese IST-Wert
            try:
                steps, ist, speed = self.bg.run_async(self.bg.read_position())
                self._respond(f"📊 Current: {ist:.3f}mm ({steps} steps) | Speed: {speed:.2f}mm/s")
            except Exception as e:
                self._respond_error(f"Encoder read failed: {e}")
                return
            
            soll = length
            
            # Plausibilitätsprüfung
            if ist < self.min_sane_value or ist > self.max_sane_value:
                self._respond_error(
                    f"Unsinniger Messwert: {ist:.3f}mm "
                    f"(erwartet {self.min_sane_value}-{self.max_sane_value}mm)"
                )
                return
            
            # Berechne Abweichung
            deviation_percent = abs((ist - soll) / soll * 100)
            
            self._respond(f"• SOLL: {soll:.3f}mm | IST: {ist:.3f}mm")
            self._respond(f"• Abweichung: {deviation_percent:.2f}%")
            
            # Prüfe Toleranz
            if deviation_percent <= tolerance:
                self._respond("")
                self._respond(f"{'═' * 40}")
                self._respond("✅ KALIBRIERUNG ERFOLGREICH!")
                self._respond(f"{'═' * 40}")
                self._respond(f"Iterationen: {iteration}")
                current_rd = extruder.get_status(0)["rotation_distance"]
                self._respond(f"Finale rotation_distance: {current_rd:.6f}")
                self._respond(f"{'═' * 40}")
                return
            
            # Update rotation_distance
            self._respond("Aktualisiere rotation_distance...")
            self.gcode.run_script_from_command(
                f"AUTO_CALC_ROTATION_DISTANCE_DIRECT SOLL={soll} IST={ist}"
            )
            
            # Pause
            if iteration < max_iter:
                self._respond(f"Warte {self.iteration_delay}s...")
                time.sleep(self.iteration_delay)
        
        # Max Iterationen erreicht
        self._respond("")
        self._respond(f"{'═' * 40}")
        self._respond("❌ KALIBRIERUNG FEHLGESCHLAGEN")
        self._respond(f"{'═' * 40}")
        self._respond(f"Max. Iterationen ({max_iter}) erreicht")
        self._respond(f"Letzte Abweichung: {deviation_percent:.2f}%")
        self._respond(f"{'═' * 40}")
    
    # ========================================================================
    # G-Code Commands
    # ========================================================================
    
    def cmd_ENCODER_AUTO_CALIBRATE(self, gcmd):
        tolerance = gcmd.get_float("TOLERANCE", self.tolerance_percent)
        max_iter = gcmd.get_int("MAX_ITERATIONS", self.max_iterations)
        length = gcmd.get_float("LENGTH", self.extrude_length)
        
        self.run_calibration(tolerance, max_iter, length)
    
    def cmd_ENCODER_CONNECTION_TEST(self, gcmd):
        self._respond("Testing encoder connection...")
        
        if self.bg.is_connected():
            self._respond("✅ Connected!")
            try:
                steps, mm, speed = self.bg.run_async(self.bg.read_position())
                self._respond(f"Position: {mm:.3f}mm ({steps} steps) | Speed: {speed:.2f}mm/s")
            except Exception as e:
                self._respond_error(f"Read failed: {e}")
        else:
            self._respond_error("❌ Not connected")
    
    def cmd_ENCODER_RESET_POSITION(self, gcmd):
        if not self.bg.is_connected():
            self._respond_error("Not connected")
            return
        
        try:
            self.bg.run_async(self.bg.reset_position())
            self._respond("Encoder position reset to 0")
        except Exception as e:
            self._respond_error(f"Reset failed: {e}")
    
    def cmd_ENCODER_GET_POSITION(self, gcmd):
        if not self.bg.is_connected():
            self._respond_error("Not connected")
            return
        
        try:
            steps, mm, speed = self.bg.run_async(self.bg.read_position())
            self._respond(f"Encoder Position: {mm:.3f}mm ({steps} steps) | Speed: {speed:.2f}mm/s")
        except Exception as e:
            self._respond_error(f"Read failed: {e}")
    
    def cmd_ENCODER_PRINT_STATUS(self, gcmd):
        self._respond(f"{'═' * 40}")
        self._respond("📊  ENCODER STATUS")
        self._respond(f"{'═' * 40}")
        self._respond(f"BLE Address: {self.ble_address or 'Auto-detect'}")
        self._respond(f"Connected: {'✅ Yes' if self.bg.is_connected() else '❌ No'}")
        self._respond(f"Wheel Diameter: {self.wheel_diameter:.3f}mm")
        self._respond(f"Tolerance: ±{self.tolerance_percent}%")
        self._respond(f"Max Iterations: {self.max_iterations}")
        self._respond(f"{'═' * 40}")
    
    cmd_ENCODER_ALIGNMENT_TEST_help = "Test sensor alignment (10 second test)"
    def cmd_ENCODER_ALIGNMENT_TEST(self, gcmd):
        """10-second alignment test - measures magnitude variation while rotating"""
        if not self.bg.is_connected():
            self._respond_error("Not connected")
            return
        
        self._respond("🔧 SENSOR ALIGNMENT TEST")
        self._respond(f"{'━' * 40}")
        self._respond("Drehe das Encoder-Rad langsam für 10 Sekunden...")
        self._respond("")
        
        samples = []
        start_time = time.time()
        
        try:
            while time.time() - start_time < 10.0:
                magnitude, agc, diag_flags = self.bg.run_async(self.bg.read_diagnostics())
                samples.append(magnitude)
                
                # Live output
                status = "✅" if 3000 <= magnitude <= 5000 else "⚠️"
                self._respond(f"Mag: {magnitude:4d} | AGC: {agc:3d} | {status}")
                
                time.sleep(0.2)  # 5Hz sampling
            
            # Analyze results
            min_mag = min(samples)
            max_mag = max(samples)
            avg_mag = sum(samples) / len(samples)
            variation = max_mag - min_mag
            
            # Calculate standard deviation
            import math
            variance = sum((x - avg_mag) ** 2 for x in samples) / len(samples)
            std_dev = math.sqrt(variance)
            
            self._respond("")
            self._respond(f"{'━' * 40}")
            self._respond("📊 ALIGNMENT ERGEBNIS:")
            self._respond(f"{'━' * 40}")
            self._respond(f"Magnitude:")
            self._respond(f"  Min: {min_mag}  Max: {max_mag}  Variation: {variation}")
            self._respond(f"  Durchschnitt: {avg_mag:.1f}  StdDev: {std_dev:.1f}")
            
            # Rating
            if variation < 50:
                rating = "✅ PERFEKT ZENTRIERT!"
                advice = "Sensor ist optimal ausgerichtet."
            elif variation < 200:
                rating = "✅ GUT"
                advice = "Ausrichtung ist akzeptabel."
            elif variation < 500:
                rating = "⚠️ MITTEL"
                advice = "Sensor könnte besser zentriert werden."
            else:
                rating = "❌ SCHLECHT"
                advice = "Sensor ist exzentrisch! Bitte neu ausrichten."
            
            self._respond(f"  AGC Durchschnitt: {sum(s[1] for s in [(self.bg.run_async(self.bg.read_diagnostics())) for _ in range(5)]) / 5:.0f}")
            self._respond("")
            self._respond(rating)
            self._respond(advice)
            self._respond(f"{'━' * 40}")
            
        except Exception as e:
            self._respond_error(f"Test failed: {e}")
    
    cmd_ENCODER_DIAGNOSTICS_LIVE_help = "Toggle live diagnostics monitor"
    def cmd_ENCODER_DIAGNOSTICS_LIVE(self, gcmd):
        """Toggle live diagnostics on Pico - outputs to Pico Serial Monitor!"""
        if not self.bg.is_connected():
            self._respond_error("Not connected")
            return
        
        # Check if already running
        if hasattr(self, '_diagnostics_running') and self._diagnostics_running:
            try:
                self.bg.run_async(self.bg.stop_diagnostics_serial_mode())
                self._diagnostics_running = False
                self._respond("✅ Live diagnostics monitor GESTOPPT")
                self._respond("📺 Pico Serial Monitor gestoppt")
            except Exception as e:
                self._respond_error(f"Stop failed: {e}")
            return
        
        # Start diagnostics mode on Pico
        try:
            self.bg.run_async(self.bg.start_diagnostics_serial_mode())
            self._diagnostics_running = True
            self._respond("✅ Live diagnostics monitor GESTARTET")
            self._respond("📺 Öffne Pico Serial Monitor (115200 baud)!")
            self._respond("📊 Live Mag/AGC Werte erscheinen dort")
            self._respond("⏸️  Zum Stoppen: ENCODER_DIAGNOSTICS_LIVE nochmal ausführen")
        except Exception as e:
            self._respond_error(f"Start failed: {e}")
    
    def cmd_CALIBRATE_ENCODER_WHEEL(self, gcmd):
        """Calibrate encoder wheel diameter by measuring known filament length"""
        if not self.bg.is_connected():
            self._respond_error("❌ Encoder nicht verbunden!")
            return
        
        # Get filament length (required parameter)
        filament_length = gcmd.get_float("LENGTH", None)
        if filament_length is None:
            self._respond_error("❌ Fehler: LENGTH parameter erforderlich!")
            self._respond("   Beispiel: CALIBRATE_ENCODER_WHEEL LENGTH=100")
            return
        
        if filament_length <= 0 or filament_length > 500:
            self._respond_error("❌ Fehler: LENGTH muss zwischen 0 und 500mm sein!")
            return
        
        self._respond("📏 Starte Rad-Durchmesser Kalibrierung...")
        self._respond(f"• Filament-Länge: {filament_length}mm")
        self._respond("")
        self._respond("🔄 ANLEITUNG:")
        self._respond("   1. Markiere Filament bei 0mm")
        self._respond(f"   2. Schiebe {filament_length}mm Filament durch (von Hand oder Extruder)")
        self._respond("   3. Warte 3 Sekunden ohne Bewegung")
        self._respond("   4. System erkennt automatisch das Ende!")
        self._respond("")
        
        try:
            # Reset encoder position
            self._respond("🔄 Setze Encoder auf Null...")
            self.bg.run_async(self.bg.reset_position())
            time.sleep(0.5)
            
            # Wait for user to push filament
            self._respond("⏸️  Warte auf Filament-Durchschub...")
            self._respond(f"   Schiebe jetzt {filament_length}mm Filament durch!")
            self._respond("   (System wartet auf 3 Sekunden stabile Position)")
            
            # Read initial position
            initial_steps, _, _ = self.bg.run_async(self.bg.read_position())
            
            # Wait for user confirmation (we'll check position changes)
            start_time = time.time()
            last_steps = initial_steps
            stable_count = 0
            
            self._respond("")
            self._respond("📊 Warte auf stabile Position...")
            
            while True:
                time.sleep(0.5)
                current_steps, _, _ = self.bg.run_async(self.bg.read_position())
                
                # Check if position is stable (no movement for 3 seconds)
                if abs(current_steps - last_steps) < 10:
                    stable_count += 1
                    if stable_count >= 6:  # 3 seconds stable
                        break
                else:
                    stable_count = 0
                    self._respond(f"   Steps: {current_steps}")
                
                last_steps = current_steps
                
                # Timeout after 2 minutes
                if time.time() - start_time > 120:
                    self._respond_error("⏱️  Timeout! Kalibrierung abgebrochen.")
                    return
            
            # Read final position
            final_steps, _, _ = self.bg.run_async(self.bg.read_position())
            total_steps = abs(final_steps - initial_steps)
            
            if total_steps < 100:
                self._respond_error("❌ Fehler: Zu wenig Bewegung erkannt!")
                self._respond(f"   Gemessen: {total_steps} steps")
                self._respond("   Mindestens 100 steps erforderlich")
                return
            
            # Calculate wheel diameter
            # Formula: diameter = (length * resolution) / (steps * π)
            calculated_diameter = (filament_length * self.encoder_resolution) / (total_steps * 3.14159265359)
            
            self._respond("")
            self._respond("✅ Messung abgeschlossen!")
            self._respond(f"   Gemessene Steps: {total_steps}")
            self._respond(f"   Filament-Länge: {filament_length}mm")
            self._respond(f"   Berechneter Durchmesser: {calculated_diameter:.3f}mm")
            self._respond("")
            
            # Ask for confirmation
            self._respond("💾 Möchtest du diesen Wert speichern?")
            self._respond(f"   Aktuell: {self.wheel_diameter:.3f}mm")
            self._respond(f"   Neu: {calculated_diameter:.3f}mm")
            self._respond("")
            self._respond("   Zum Speichern: SAVE_ENCODER_WHEEL_DIAMETER")
            
            # Store for later save
            self._pending_wheel_diameter = calculated_diameter
            
        except Exception as e:
            self._respond_error(f"❌ Fehler: {str(e)}")
            import traceback
            self._respond_error(traceback.format_exc())
    
    def cmd_SAVE_ENCODER_WHEEL_DIAMETER(self, gcmd):
        """Save the calibrated wheel diameter"""
        if self._pending_wheel_diameter is None:
            self._respond_error("❌ Keine Kalibrierung vorhanden!")
            self._respond("   Führe zuerst CALIBRATE_ENCODER_WHEEL aus")
            return
        
        try:
            # Update local values
            old_diameter = self.wheel_diameter
            self.wheel_diameter = self._pending_wheel_diameter
            self.wheel_circumference = 3.14159265359 * self.wheel_diameter
            
            # Send to Pico
            self._respond(f"📤 Sende neuen Durchmesser an Pico...")
            self.bg.run_async(self.bg.write_config(self.wheel_diameter))
            
            self._respond("")
            self._respond("✅ Raddurchmesser gespeichert!")
            self._respond(f"   Alt: {old_diameter:.3f}mm")
            self._respond(f"   Neu: {self.wheel_diameter:.3f}mm")
            self._respond(f"   Umfang: {self.wheel_circumference:.3f}mm")
            self._respond("")
            self._respond("💡 WICHTIG: Aktualisiere encoder_calibration.cfg:")
            self._respond(f"   wheel_diameter: {self.wheel_diameter:.3f}")
            
            # Clear pending
            self._pending_wheel_diameter = None
            
        except Exception as e:
            self._respond_error(f"❌ Fehler beim Speichern: {str(e)}")
            import traceback
            self._respond_error(traceback.format_exc())
    
    def cmd__START_ENCODER_CALIBRATION(self, gcmd):
        """Start automatic encoder LUT calibration with 2 full rotations"""
        if not self.bg.is_connected():
            self._respond_error("❌ Encoder nicht verbunden!")
            return
            
        # Get parameters with defaults
        speed = gcmd.get_float("SPEED", 5.0)  # mm/s
        distance = gcmd.get_float("DISTANCE", self.wheel_circumference * 2)  # 2 full rotations
        
        self._respond("🔧 Starte automatische Encoder-Kalibrierung...")
        self._respond(f"• Umdrehungen: {distance/self.wheel_circumference:.1f}")
        self._respond(f"• Strecke: {distance:.2f}mm")
        self._respond(f"• Geschwindigkeit: {speed}mm/s")
        self._respond(" ")
        self._respond("🔄 Bewege das Filament nicht während der Kalibrierung!")
        
        try:
            # Start calibration mode on Pico
            self._respond("\n🚀 Starte Kalibrierungsmodus...")
            self.bg.start_calibration()
            
            # Move extruder
            self._respond(f"\n🔄 Bewege Extruder um {distance:.2f}mm...")
            gcmd.respond_info("Bewege Extruder für Kalibrierung...")
            
            # Save current position
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.wait_moves()
            
            # Move in relative mode
            self.gcode.run_script(f"""
                G91                   ; Relativmodus
                G1 E{distance:.2f} F{speed*60:.0f}  ; Extrudieren mit {speed}mm/s
                G90                   ; Zurück zum Absolutmodus
            """)
            
            # Wait for movement to complete
            while toolhead.is_moving():
                time.sleep(0.1)
            
            # End calibration and save LUT
            self._respond("\n💾 Speichere Kalibrierung...")
            self.bg.end_calibration()
            
            self._respond("\n✅ Encoder-Kalibrierung erfolgreich abgeschlossen!")
            self._respond("   Die neuen Kalibrierungsdaten wurden gespeichert.")
            
        except Exception as e:
            self._respond_error(f"\n❌ Fehler bei der Kalibrierung: {str(e)}")
            import traceback
            self._respond_error(traceback.format_exc())
    
    # ========================================================================
    # Helpers
    # ========================================================================
    
    def _respond(self, msg: str):
        """Send message to console"""
        self.gcode.respond_info(msg)
    
    def _respond_error(self, msg: str):
        self.gcode.respond_raw(f"!! {msg}\n")


# ============================================================================
# Virtual ADC Pins for Hall Sensors
# ============================================================================

class EncoderADCPin:
    """
    Virtual ADC pin that reads values from Pico via BLE
    Compatible with Klipper's hall_filament_width_sensor
    """
    def __init__(self, encoder_bg, adc_channel):
        """
        Args:
            encoder_bg: EncoderBackground instance
            adc_channel: 1 or 2 (ADC1 or ADC2)
        """
        self.encoder_bg = encoder_bg
        self.adc_channel = adc_channel
        self._last_value = 0.0
        self._mcu = None
        self._callback = None
        self._sample_time = 0.001  # 1ms default
        self._sample_count = 1
        self._report_time = 0.5  # 500ms default
        self._timer = None
    
    def get_mcu(self):
        """Return virtual MCU (encoder)"""
        return self._mcu
    
    def setup_adc_callback(self, report_time, callback):
        """Setup callback for ADC readings"""
        self._report_time = report_time
        self._callback = callback
    
    def setup_adc_sample(self, sample_time, sample_count):
        """Setup ADC sampling parameters"""
        self._sample_time = sample_time
        self._sample_count = sample_count
    
    def setup_minmax(self, sample_time, minval, maxval, range_check_count=0):
        """Setup min/max range (not used for virtual ADC)"""
        pass
    
    def _read_adc_value(self):
        """Read ADC value from Pico via BLE"""
        try:
            if not self.encoder_bg.is_connected():
                return self._last_value
            
            # Read ADC values from Pico (run async in background loop)
            adc1, adc2 = self.encoder_bg.run_async(self.encoder_bg.read_adc())
            
            # Select correct channel
            raw_value = adc1 if self.adc_channel == 1 else adc2
            
            # Convert 12-bit ADC (0-4095) to normalized value (0.0-1.0)
            # Klipper expects values in range 0.0-1.0
            normalized = raw_value / 4095.0
            
            self._last_value = normalized
            return normalized
            
        except Exception as e:
            logging.error(f"EncoderADCPin: Error reading ADC{self.adc_channel}: {e}")
            return self._last_value
    
    def _timer_callback(self, eventtime):
        """Timer callback to read ADC and call user callback"""
        try:
            value = self._read_adc_value()
            if self._callback:
                self._callback(eventtime, value)
        except Exception as e:
            logging.error(f"EncoderADCPin: Timer callback error: {e}")
        
        return eventtime + self._report_time


class EncoderPins:
    """
    Pin manager for virtual encoder ADC pins
    Registers with Klipper's pin system
    """
    def __init__(self, encoder_bg, reactor):
        self.encoder_bg = encoder_bg
        self.reactor = reactor
        self.pins = {}
    
    def setup_pin(self, pin_type, pin_params):
        """Setup a virtual pin"""
        if pin_type != 'adc':
            raise Exception(f"Encoder pins only support 'adc' type, got '{pin_type}'")
        
        # Parse pin name: "encoder:adc1" or "encoder:adc2"
        pin_name = pin_params['pin']
        if not pin_name.startswith('encoder:'):
            raise Exception(f"Invalid encoder pin name: {pin_name}")
        
        adc_name = pin_name.split(':', 1)[1]
        if adc_name not in ['adc1', 'adc2']:
            raise Exception(f"Invalid ADC channel: {adc_name}. Use 'adc1' or 'adc2'")
        
        adc_channel = 1 if adc_name == 'adc1' else 2
        
        # Create or return existing pin
        if pin_name not in self.pins:
            pin = EncoderADCPin(self.encoder_bg, adc_channel)
            pin._mcu = self  # Virtual MCU
            self.pins[pin_name] = pin
            
            # Start timer for this pin
            pin._timer = self.reactor.register_timer(pin._timer_callback)
            self.reactor.update_timer(pin._timer, self.reactor.NOW)
        
        return self.pins[pin_name]
    
    def get_printer(self):
        """Return printer object (for MCU interface)"""
        return self.encoder_bg._printer if hasattr(self.encoder_bg, '_printer') else None


def load_config(config):
    return EncoderCalibration(config)
