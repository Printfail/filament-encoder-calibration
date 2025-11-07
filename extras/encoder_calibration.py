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
        
        if not self.char_position:
            raise Exception("Position characteristic not found")
        if not self.char_reset:
            raise Exception("Reset characteristic not found")
        if not self.char_config:
            raise Exception("Config characteristic not found")
        
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
        
        await self.client.write_gatt_char(self.char_reset, b"\x01")
    
    async def write_config(self, wheel_diameter: float):
        """Write wheel diameter configuration"""
        if not self.is_connected() or not self.char_config:
            raise Exception("Not connected")
        
        import struct
        # Schreibe float (4 bytes, little-endian) wie Pico es erwartet
        data = struct.pack('<f', wheel_diameter)
        await self.client.write_gatt_char(self.char_config, data)
        self.log.info(f"Config sent to Pico: wheel_diameter={wheel_diameter}mm")
    
    def run_async(self, coro):
        """Run coroutine in background loop"""
        if not self._loop:
            raise Exception("Background loop not running")
        
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=10.0)


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
    
    # ========================================================================
    # Helpers
    # ========================================================================
    
    def _respond(self, msg: str):
        """Send message to console"""
        self.gcode.respond_info(msg)
    
    def _respond_error(self, msg: str):
        """Send error message to console"""
        self.gcode.respond_info(f"!! {msg}")


def load_config(config):
    return EncoderCalibration(config)
