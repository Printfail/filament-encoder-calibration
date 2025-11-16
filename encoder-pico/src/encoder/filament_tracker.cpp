#include "filament_tracker.hpp"
#include "pico/time.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <cmath>
#include <cstdio>
#include <cstring>

namespace encoder {

FilamentTracker::FilamentTracker(AS5048A& encoder, float wheel_diameter_mm, int8_t direction)
    : encoder_(encoder),
      wheel_diameter_mm_(wheel_diameter_mm),
      wheel_circumference_mm_(M_PI * wheel_diameter_mm),
      direction_(direction),
      last_angle_(0),
      revolutions_(0),
      total_steps_(0),
      distance_mm_(0.0f),
      speed_mm_per_sec_(0.0f),
      initialized_(false),
      last_distance_mm_(0.0f),
      last_update_time_us_(0),
      filter_index_(0),
      filter_count_(0),
      calibration_count_(0),
      noise_threshold_(5.0f),  // Default 5 steps
      is_calibrated_(false),
      lut_(nullptr),
      lut_calibrating_(false),
      lut_sample_count_(0),
      lut_start_angle_(0),
      lut_expected_steps_(0),
      lut_last_angle_(0) {
    
    // Initialisiere Filter-Array
    for (int i = 0; i < FILTER_SIZE; i++) {
        angle_history_[i] = 0;
    }
    
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        calibration_samples_[i] = 0;
    }
    
    printf("FilamentTracker: Initialized with diameter=%.3fmm, circumference=%.3fmm\n",
           wheel_diameter_mm_, wheel_circumference_mm_);
    printf("FilamentTracker: Starting noise calibration (keep still!)...\n");
    
    // Lade LUT aus Flash (falls vorhanden)
    load_lut_from_flash();
}

void FilamentTracker::reset() {
    // Lese aktuellen Encoder-Wert als neuen Nullpunkt
    auto angle = encoder_.read_angle();
    if (angle.has_value()) {
        last_angle_ = angle.value();
        initialized_ = true;
    } else {
        printf("FilamentTracker: WARNING - Failed to read encoder during reset\n");
        last_angle_ = 0;
        initialized_ = false;
    }
    
    revolutions_ = 0;
    total_steps_ = 0;
    distance_mm_ = 0.0f;
    
    // Reset Filter
    filter_index_ = 0;
    filter_count_ = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        angle_history_[i] = last_angle_;
    }
    
    printf("FilamentTracker: Reset to 0 (encoder at %u)\n", last_angle_);
}

bool FilamentTracker::update() {
    auto angle_opt = encoder_.read_angle();
    if (!angle_opt.has_value()) {
        printf("FilamentTracker: Failed to read encoder\n");
        return false;
    }
    
    uint16_t current_angle = angle_opt.value();
    
    // Während LUT-Kalibrierung: Sammle Samples und berechne Korrekturen
    if (lut_calibrating_ && lut_) {
        // Berechne Delta seit letztem Sample
        int32_t delta = static_cast<int32_t>(current_angle) - static_cast<int32_t>(lut_last_angle_);
        
        // Handle overflow
        if (delta > 8192) {
            delta -= AS5048A::RESOLUTION;
        } else if (delta < -8192) {
            delta += AS5048A::RESOLUTION;
        }
        
        // Akkumuliere erwartete Steps (gleichmäßige Bewegung angenommen)
        lut_expected_steps_ += delta;
        
        // Berechne erwarteten Winkel (linear interpoliert)
        int32_t expected_angle = (lut_start_angle_ + lut_expected_steps_) % AS5048A::RESOLUTION;
        if (expected_angle < 0) expected_angle += AS5048A::RESOLUTION;
        
        // Berechne Fehler: Differenz zwischen erwartet und gemessen
        int16_t error = static_cast<int16_t>(expected_angle - current_angle);
        
        // Speichere Korrektur in LUT (gleitender Durchschnitt für Rauschunterdrückung)
        if (lut_[current_angle] == 0) {
            lut_[current_angle] = error;
        } else {
            // Mittelwert mit vorherigem Wert
            lut_[current_angle] = (lut_[current_angle] + error) / 2;
        }
        
        lut_sample_count_++;
        lut_last_angle_ = current_angle;
        
        // Zeige Fortschritt alle 1000 Samples
        if (lut_sample_count_ % 1000 == 0) {
            printf("LUT_CAL: %d samples, current error: %d steps\n", lut_sample_count_, error);
        }
    }
    
    // Kalibrierungs-Phase: Sammle Messungen im Stillstand
    if (!is_calibrated_) {
        if (calibration_count_ < CALIBRATION_SAMPLES) {
            calibration_samples_[calibration_count_] = current_angle;
            calibration_count_++;
            
            if (!initialized_) {
                last_angle_ = current_angle;
                initialized_ = true;
            }
            
            return true;  // Noch in Kalibrierung
        } else {
            // Kalibrierung abgeschlossen
            calibrate_noise_threshold();
            is_calibrated_ = true;
        }
    }
    
    if (!initialized_) {
        // Erste Messung - setze als Referenz
        last_angle_ = current_angle;
        initialized_ = true;
        return true;
    }
    
    // Berechne Delta mit RAW Wert (nicht gefiltert!)
    // Filter würde bei Overflow falsche Werte produzieren
    int32_t delta = static_cast<int32_t>(current_angle) - static_cast<int32_t>(last_angle_);
    
    // Erkenne Überläufe (0→16383 oder 16383→0) und korrigiere Delta
    // WICHTIG: Threshold muss HÄLFTE sein (8192), nicht Viertel!
    // Sonst werden schnelle Bewegungen falsch als Overflow erkannt
    const uint16_t THRESHOLD = AS5048A::RESOLUTION / 2;  // 8192 steps
    
    // Overflow-Erkennung basierend auf Roadrunner's Logik:
    // Prüfe ob Delta zu groß ist UND schaue auf VORHERIGEN Winkel
    if (abs(delta) > static_cast<int32_t>(THRESHOLD)) {
        // Delta ist größer als halbe Drehung → Overflow!
        if (last_angle_ > THRESHOLD) {
            // Vorheriger Winkel war in oberer Hälfte (8192-16383)
            // → Overflow vorwärts über Nullpunkt
            revolutions_++;
            delta += AS5048A::RESOLUTION;
            printf("FilamentTracker: Forward overflow detected (rev=%d)\n", revolutions_);
        }
        else {
            // Vorheriger Winkel war in unterer Hälfte (0-8191)
            // → Overflow rückwärts über Nullpunkt
            revolutions_--;
            delta -= AS5048A::RESOLUTION;
            printf("FilamentTracker: Backward overflow detected (rev=%d)\n", revolutions_);
        }
    }
    
    // ADAPTIVE SCHWELLE: Ignoriere Rauschen basierend auf Kalibrierung
    if (abs(delta) >= static_cast<int32_t>(noise_threshold_)) {
        // Nur echte Bewegung zählen (> Rausch-Schwelle)
        total_steps_ += (delta * direction_);
        
        // Aktualisiere letzten Winkel nur bei echter Bewegung
        last_angle_ = current_angle;
    }
    // Sonst: Rauschen ignorieren, last_angle_ bleibt gleich
    
    // Berechne Distanz
    calculate_distance();
    
    // Berechne Geschwindigkeit (mm/s)
    calculate_speed();
    
    return true;
}

void FilamentTracker::set_wheel_diameter(float diameter_mm) {
    wheel_diameter_mm_ = diameter_mm;
    wheel_circumference_mm_ = M_PI * diameter_mm;
    
    // Neuberechnung der Distanz mit neuem Durchmesser
    calculate_distance();
    
    printf("FilamentTracker: Diameter updated to %.3fmm (circumference=%.3fmm)\n",
           wheel_diameter_mm_, wheel_circumference_mm_);
}


void FilamentTracker::calculate_distance() {
    // Distanz = (Steps / Resolution) × Radumfang
    // Steps können positiv oder negativ sein (Richtung)
    float total_revolutions = static_cast<float>(total_steps_) / AS5048A::RESOLUTION;
    distance_mm_ = total_revolutions * wheel_circumference_mm_;
}

uint16_t FilamentTracker::get_filtered_angle(uint16_t new_angle) {
    // Füge neue Messung zum Filter-Array hinzu
    angle_history_[filter_index_] = new_angle;
    filter_index_ = (filter_index_ + 1) % FILTER_SIZE;
    
    if (filter_count_ < FILTER_SIZE) {
        filter_count_++;
    }
    
    // Berechne gleitenden Durchschnitt
    // Bei weniger als FILTER_SIZE Messungen, nur über verfügbare Messungen mitteln
    uint32_t sum = 0;
    for (int i = 0; i < filter_count_; i++) {
        sum += angle_history_[i];
    }
    
    return static_cast<uint16_t>(sum / filter_count_);
}

void FilamentTracker::calibrate_noise_threshold() {
    // Berechne Durchschnitt der Kalibrierungsmessungen
    uint32_t sum = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum += calibration_samples_[i];
    }
    float mean = static_cast<float>(sum) / CALIBRATION_SAMPLES;
    
    // Berechne Standardabweichung (Rauschen)
    float variance_sum = 0.0f;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        float diff = static_cast<float>(calibration_samples_[i]) - mean;
        variance_sum += diff * diff;
    }
    float std_dev = sqrtf(variance_sum / CALIBRATION_SAMPLES);
    
    // Schwellwert = 3× Standardabweichung (99.7% der Messungen)
    // Minimum 3 steps um echte kleine Bewegungen nicht zu verlieren
    noise_threshold_ = fmaxf(3.0f * std_dev, 3.0f);
    
    printf("FilamentTracker: Noise calibration complete!\n");
    printf("  Mean: %.1f, StdDev: %.2f, Threshold: %.1f steps (%.3fmm)\n",
           mean, std_dev, noise_threshold_,
           (noise_threshold_ / AS5048A::RESOLUTION) * wheel_circumference_mm_);
}

void FilamentTracker::calculate_speed() {
    // Zeit in Mikrosekunden vom Pico SDK (pico/time.h)
    uint64_t current_time_us = time_us_64();
    
    // Beim ersten Update initialisieren
    if (last_update_time_us_ == 0) {
        last_update_time_us_ = current_time_us;
        last_distance_mm_ = distance_mm_;
        speed_mm_per_sec_ = 0.0f;
        return;
    }
    
    // Berechne Zeit-Delta in Sekunden
    float delta_time_sec = (current_time_us - last_update_time_us_) / 1000000.0f;
    
    // Vermeide Division durch 0
    if (delta_time_sec < 0.001f) {  // < 1ms
        return;
    }
    
    // Berechne Distanz-Delta
    float delta_distance_mm = distance_mm_ - last_distance_mm_;
    
    // Geschwindigkeit = Delta Distance / Delta Time
    speed_mm_per_sec_ = delta_distance_mm / delta_time_sec;
    
    // Update für nächste Berechnung
    last_update_time_us_ = current_time_us;
    last_distance_mm_ = distance_mm_;
}

std::optional<EncoderDiagnostics> FilamentTracker::get_diagnostics() {
    auto magnitude = encoder_.read_magnitude();
    auto agc = encoder_.read_agc();
    auto diag = encoder_.read_diagnostics();
    
    if (!magnitude.has_value() || !agc.has_value() || !diag.has_value()) {
        return std::nullopt;
    }
    
    EncoderDiagnostics result;
    result.magnitude = magnitude.value();
    result.agc = agc.value();
    result.diagnostics = diag.value();
    
    return result;
}

void FilamentTracker::start_lut_calibration() {
    printf("FilamentTracker: Starting LUT calibration...\n");
    
    // Allocate LUT if not already done
    if (!lut_) {
        lut_ = new int16_t[LUT_SIZE];
    }
    
    // Initialisiere LUT mit 0 (keine Korrektur)
    for (int i = 0; i < LUT_SIZE; i++) {
        lut_[i] = 0;
    }
    
    // Read start angle
    auto angle = encoder_.read_angle();
    if (angle.has_value()) {
        lut_start_angle_ = angle.value();
        lut_last_angle_ = angle.value();
    }
    
    lut_calibrating_ = true;
    lut_sample_count_ = 0;
    lut_expected_steps_ = 0;
    
    printf("FilamentTracker: LUT calibration started at angle %u\n", lut_start_angle_);
    printf("FilamentTracker: Rotate slowly for 2 full revolutions...\n");
}

void FilamentTracker::end_lut_calibration() {
    if (!lut_calibrating_) {
        printf("FilamentTracker: No calibration in progress\n");
        return;
    }
    
    lut_calibrating_ = false;
    
    printf("FilamentTracker: LUT calibration completed with %d samples\n", lut_sample_count_);
    
    // Interpoliere fehlende Werte (Winkel die nicht gemessen wurden)
    printf("FilamentTracker: Interpolating missing values...\n");
    for (int i = 0; i < LUT_SIZE; i++) {
        if (lut_[i] == 0) {
            // Finde nächsten nicht-null Wert vorwärts und rückwärts
            int prev = i - 1;
            int next = i + 1;
            
            // Suche vorherigen Wert
            while (prev >= 0 && lut_[prev] == 0) prev--;
            if (prev < 0) prev = LUT_SIZE - 1;
            
            // Suche nächsten Wert
            while (next < LUT_SIZE && lut_[next] == 0) next++;
            if (next >= LUT_SIZE) next = 0;
            
            // Interpoliere
            if (lut_[prev] != 0 || lut_[next] != 0) {
                lut_[i] = (lut_[prev] + lut_[next]) / 2;
            }
        }
    }
    
    printf("FilamentTracker: Saving LUT to flash...\n");
    save_lut_to_flash();
    
    printf("FilamentTracker: LUT calibration finished!\n");
}

uint16_t FilamentTracker::apply_lut_correction(uint16_t raw_angle) {
    if (!lut_) {
        return raw_angle;  // No LUT loaded
    }
    
    // Apply correction from LUT
    int16_t correction = lut_[raw_angle];
    int32_t corrected = static_cast<int32_t>(raw_angle) + correction;
    
    // Wrap around if necessary
    if (corrected < 0) {
        corrected += AS5048A::RESOLUTION;
    } else if (corrected >= AS5048A::RESOLUTION) {
        corrected -= AS5048A::RESOLUTION;
    }
    
    return static_cast<uint16_t>(corrected);
}

void FilamentTracker::save_lut_to_flash() {
    if (!lut_) {
        printf("FilamentTracker: No LUT to save\n");
        return;
    }
    
    printf("FilamentTracker: Saving LUT to flash...\n");
    
    // Flash-Adresse: Letzter Sektor (2MB - 4KB = 0x1FF000)
    // WICHTIG: Muss FLASH_SECTOR_SIZE (4096 bytes) aligned sein!
    const uint32_t FLASH_TARGET_OFFSET = (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE);
    
    // LUT Größe: 16384 * 2 bytes = 32768 bytes = 8 Sektoren
    const uint32_t LUT_SIZE_BYTES = LUT_SIZE * sizeof(int16_t);
    const uint32_t SECTORS_NEEDED = (LUT_SIZE_BYTES + FLASH_SECTOR_SIZE - 1) / FLASH_SECTOR_SIZE;
    
    printf("  Flash offset: 0x%X\n", FLASH_TARGET_OFFSET);
    printf("  LUT size: %d bytes (%d sectors)\n", LUT_SIZE_BYTES, SECTORS_NEEDED);
    
    // Interrupts deaktivieren während Flash-Schreiben
    uint32_t ints = save_and_disable_interrupts();
    
    // Lösche Flash-Sektoren
    flash_range_erase(FLASH_TARGET_OFFSET, SECTORS_NEEDED * FLASH_SECTOR_SIZE);
    
    // Schreibe LUT in Flash (muss in 256-byte Pages geschrieben werden)
    flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t*)lut_, LUT_SIZE_BYTES);
    
    // Interrupts wieder aktivieren
    restore_interrupts(ints);
    
    printf("FilamentTracker: LUT saved to flash successfully!\n");
}

void FilamentTracker::load_lut_from_flash() {
    printf("FilamentTracker: Loading LUT from flash...\n");
    
    // Flash-Adresse (XIP_BASE = 0x10000000 für Pico)
    const uint32_t FLASH_TARGET_OFFSET = (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE);
    const uint8_t* flash_ptr = (const uint8_t*)(XIP_BASE + FLASH_TARGET_OFFSET);
    
    // Allocate LUT if not already done
    if (!lut_) {
        lut_ = new int16_t[LUT_SIZE];
    }
    
    // Kopiere aus Flash in RAM
    memcpy(lut_, flash_ptr, LUT_SIZE * sizeof(int16_t));
    
    // Prüfe ob LUT gültig ist (erste Werte sollten nicht alle 0xFF sein)
    bool is_valid = false;
    for (int i = 0; i < 100; i++) {
        if (lut_[i] != -1) {  // 0xFFFF = -1 in int16_t
            is_valid = true;
            break;
        }
    }
    
    if (is_valid) {
        printf("FilamentTracker: LUT loaded from flash successfully!\n");
    } else {
        printf("FilamentTracker: No valid LUT found in flash (using defaults)\n");
        // Initialisiere mit 0
        for (int i = 0; i < LUT_SIZE; i++) {
            lut_[i] = 0;
        }
    }
}

}  // namespace encoder
