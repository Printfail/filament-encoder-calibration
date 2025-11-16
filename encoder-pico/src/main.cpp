#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "btstack.h"
#include "encoder/as5048a.hpp"
#include "encoder/filament_tracker.hpp"
#include "adc/hall_sensor.hpp"
#include "gatt.hpp"
#include "gatt/encoder_service.hpp"
#include <cstdio>

using namespace encoder;
using namespace adc;

// ========================================================================
// Configuration
// ========================================================================

// SPI Configuration
#define SPI_PORT spi0
#define SPI_MISO_PIN 16    // GPIO 16 (Pin 21)
#define SPI_CS_PIN 17      // GPIO 17 (Pin 22)
#define SPI_CLK_PIN 18     // GPIO 18 (Pin 24)
#define SPI_MOSI_PIN 19    // GPIO 19 (Pin 25)
#define SPI_FREQ_HZ 1000000  // 1 MHz (AS5048A supports up to 10 MHz)

// Encoder Configuration
#define WHEEL_DIAMETER_MM 10.0f  // Default, kann via BLE geändert werden
#define ENCODER_DIRECTION 1      // 1 oder -1

// Update Rate
#define ENCODER_UPDATE_INTERVAL_MS 50  // 20 Hz


// ========================================================================
// Global Objects
// ========================================================================

AS5048A* g_encoder = nullptr;
FilamentTracker* g_tracker = nullptr;
HallSensor* g_hall_sensor = nullptr;


// ========================================================================
// Callbacks
// ========================================================================

void on_encoder_reset() {
    printf("RESET: Resetting encoder position\n");
    if (g_tracker) {
        g_tracker->reset();
    }
}

void on_config_update(float wheel_diameter) {
    printf("CONFIG: Updating wheel diameter to %.3fmm\n", wheel_diameter);
    if (g_tracker) {
        g_tracker->set_wheel_diameter(wheel_diameter);
    }
}

void on_lut_calibration_start() {
    printf("LUT_CAL: Starting calibration...\n");
    if (g_tracker) {
        g_tracker->start_lut_calibration();
    }
}

void on_lut_calibration_end() {
    printf("LUT_CAL: Ending calibration...\n");
    if (g_tracker) {
        g_tracker->end_lut_calibration();
    }
}


// ========================================================================
// Hardware Setup
// ========================================================================

void setup_spi() {
    printf("Initializing SPI...\n");
    
    // Initialize SPI
    spi_init(SPI_PORT, SPI_FREQ_HZ);
    
    // Setup GPIO pins for SPI
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    // CS pin is configured in AS5048A constructor
    
    // SPI Mode 1: CPOL=0, CPHA=1 (AS5048A requirement)
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
    
    printf("SPI initialized: MISO=%d, CLK=%d, MOSI=%d, CS=%d\n", 
           SPI_MISO_PIN, SPI_CLK_PIN, SPI_MOSI_PIN, SPI_CS_PIN);
}

bool setup_encoder() {
    printf("Initializing AS5048A encoder...\n");
    
    g_encoder = new AS5048A(SPI_PORT, SPI_CS_PIN);
    
    if (!g_encoder->init()) {
        printf("ERROR: Failed to initialize encoder!\n");
        return false;
    }
    
    // Test read
    auto angle = g_encoder->read_angle();
    if (!angle.has_value()) {
        printf("ERROR: Failed to read encoder angle!\n");
        return false;
    }
    
    printf("Encoder initialized successfully. Current angle: %u\n", angle.value());
    
    // Initialize tracker
    g_tracker = new FilamentTracker(*g_encoder, WHEEL_DIAMETER_MM, ENCODER_DIRECTION);
    g_tracker->reset();
    
    return true;
}

bool setup_bluetooth() {
    printf("Initializing Bluetooth...\n");
    
    // Initialize CYW43 (WiFi/BT chip) - MUSS VOR gatt::init()!
    if (cyw43_arch_init()) {
        printf("ERROR: Failed to initialize CYW43!\n");
        return false;
    }
    
    // Initialize GATT services (macht hci_power_control intern!)
    if (!gatt::init()) {
        printf("ERROR: Failed to initialize GATT!\n");
        return false;
    }
    
    // Register callbacks
    gatt::EncoderService::set_reset_callback(on_encoder_reset);
    gatt::EncoderService::set_config_callback(on_config_update);
    gatt::EncoderService::set_lut_calibration_start_callback(on_lut_calibration_start);
    gatt::EncoderService::set_lut_calibration_end_callback(on_lut_calibration_end);
    
    printf("Bluetooth initialized\n");
    return true;
}


// ========================================================================
// Main Loop
// ========================================================================

void encoder_update_task() {
    static absolute_time_t last_update = get_absolute_time();
    static uint32_t update_counter = 0;
    static bool magnet_warning_shown = false;
    
    absolute_time_t now = get_absolute_time();
    int64_t elapsed_ms = absolute_time_diff_us(last_update, now) / 1000;
    
    if (elapsed_ms >= ENCODER_UPDATE_INTERVAL_MS) {
        last_update = now;
        update_counter++;
        
        // Magnet-Warnung NUR zur Info - NICHT blockieren!
        // Die Kalibrierung zeigt uns bereits ob der Sensor stabil ist
        if (g_encoder && !g_encoder->is_magnet_detected() && !magnet_warning_shown) {
            printf("INFO: Magnet detection marginal (AGC) - but sensor is stable!\n");
            magnet_warning_shown = true;
        }
        
        // Update tracker (nur mit Magnet!)
        if (g_tracker && g_tracker->update()) {
            // Update GATT service mit neuen Werten
            int32_t steps = g_tracker->get_position_steps();
            float distance = g_tracker->get_distance_mm();
            float speed = g_tracker->get_speed_mm_per_sec();
            
            gatt::EncoderService::update_position(steps, distance, speed);
            
            // Update diagnostics (für Alignment-Tool)
            auto diag = g_tracker->get_diagnostics();
            if (diag.has_value()) {
                gatt::EncoderService::update_diagnostics(
                    diag->magnitude, 
                    diag->agc, 
                    diag->diagnostics
                );
                
                // Live serial output wenn Diagnostics Mode aktiv
                if (gatt::EncoderService::is_diagnostics_serial_mode()) {
                    // Decode flags (diagnostics enthält jetzt Bits [13:8] des 0x3FFD Registers)
                    // Bit 2: Magnetic field strength high warning (too strong)
                    // Bit 3: Magnetic field strength low warning (too weak)
                    bool mag_too_weak = (diag->diagnostics & (1 << 3)) != 0;
                    bool mag_too_strong = (diag->diagnostics & (1 << 2)) != 0;
                    
                    const char* status = "OK";
                    if (mag_too_weak) status = "ZU SCHWACH";
                    else if (mag_too_strong) status = "ZU STARK";
                    
                    printf("DIAG: Mag: %4d | AGC: %3d | %s\n", 
                           diag->magnitude, diag->agc, status);
                }
            }
            
            // Update ADC values (Hall Sensors)
            if (g_hall_sensor) {
                uint16_t adc1, adc2;
                g_hall_sensor->read_both(&adc1, &adc2);
                gatt::EncoderService::update_adc(adc1, adc2);
            }
            
            // Debug output bei Bewegung (> 20 steps = 0.04mm)
            // Filter glättet Rauschen, daher niedrigerer Schwellwert möglich
            // ABER: Unterdrücke normale Ausgabe wenn Diagnostics Mode aktiv!
            static int32_t last_logged_steps = 0;
            int32_t step_delta = abs(steps - last_logged_steps);
            if (step_delta >= 20 && !gatt::EncoderService::is_diagnostics_serial_mode()) {
                printf("Encoder: %.3fmm (%d steps, %d rev) | Speed: %.2fmm/s\n",
                       distance, steps, g_tracker->get_revolutions(), speed);
                last_logged_steps = steps;
            }
        }
        
        // Blink LED (zeigt Aktivität)
        static bool led_state = false;
        if (update_counter % 10 == 0) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            led_state = !led_state;
        }
    }
}


// ========================================================================
// Main
// ========================================================================

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(2000);  // Warte auf USB Serial
    
    printf("\n");
    printf("========================================\n");
    printf("  Encoder Calibration - Pico W\n");
    printf("  AS5048A Filament Tracker\n");
    printf("========================================\n");
    printf("\n");
    
    // Setup Hardware
    setup_spi();
    
    if (!setup_encoder()) {
        printf("FATAL: Encoder initialization failed!\n");
        printf("Check SPI connections and encoder power.\n");
        while (true) {
            sleep_ms(1000);
        }
    }
    
    // Setup Hall Sensors (ADC)
    printf("\nInitializing Hall Sensors (ADC)...\n");
    g_hall_sensor = new HallSensor(26, 27);  // GPIO 26 = ADC0, GPIO 27 = ADC1
    if (!g_hall_sensor->init()) {
        printf("WARNING: Hall Sensor initialization failed!\n");
        printf("ADC values will not be available.\n");
        delete g_hall_sensor;
        g_hall_sensor = nullptr;
    } else {
        printf("Hall Sensors initialized successfully\n");
    }
    
    if (!setup_bluetooth()) {
        printf("FATAL: Bluetooth initialization failed!\n");
        while (true) {
            sleep_ms(1000);
        }
    }
    
    printf("\n");
    printf("========================================\n");
    printf("  System Ready!\n");
    printf("  BLE Name: Encoder-PicoW\n");
    printf("  Wheel Diameter: %.3fmm\n", WHEEL_DIAMETER_MM);
    printf("========================================\n");
    printf("\n");
    
    // Main Loop - BLE läuft automatisch im Background!
    while (true) {
        // Update encoder
        encoder_update_task();
        
        // Mit pico_cyw43_arch_threadsafe_background läuft BLE automatisch
        // im Hintergrund - kein manuelles Polling nötig!
        
        // Kurze Pause (1ms = 1000 Hz Loop)
        sleep_ms(1);
    }
    
    return 0;
}
