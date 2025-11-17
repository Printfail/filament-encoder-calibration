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
      is_calibrated_(false) {
    
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
    
    // Overflow-Erkennung:
    // Wenn Delta > halbe Drehung, ist ein Overflow passiert
    if (abs(delta) > static_cast<int32_t>(THRESHOLD)) {
        // Delta ist größer als halbe Drehung → Overflow!
        if (delta > 0) {
            // Positives Delta zu groß → Backward overflow (16000→100)
            // Tatsächliche Bewegung war rückwärts über Nullpunkt
            revolutions_--;
            delta -= AS5048A::RESOLUTION;
            printf("FilamentTracker: Backward overflow detected (rev=%d)\n", revolutions_);
        }
        else {
            // Negatives Delta zu groß → Forward overflow (100→16000)
            // Tatsächliche Bewegung war vorwärts über Nullpunkt
            revolutions_++;
            delta += AS5048A::RESOLUTION;
            printf("FilamentTracker: Forward overflow detected (rev=%d)\n", revolutions_);
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

}  // namespace encoder
