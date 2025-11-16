#pragma once

#include "as5048a.hpp"
#include <cstdint>

namespace encoder {

/**
 * Diagnostics data from encoder
 */
struct EncoderDiagnostics {
    uint16_t magnitude;    // Magnetic field strength (0-16383)
    uint8_t agc;           // Automatic Gain Control (0-255)
    uint8_t diagnostics;   // Diagnostics flags
};

/**
 * Filament Tracker
 * Konvertiert Encoder-Umdrehungen in Filament-Distanz (mm)
 */
class FilamentTracker {
public:
    /**
     * Constructor
     * @param encoder AS5048A Encoder Instanz
     * @param wheel_diameter_mm Durchmesser der Encoder-Welle in mm
     * @param direction Richtung: 1 = normal, -1 = invertiert
     */
    FilamentTracker(AS5048A& encoder, float wheel_diameter_mm, int8_t direction = 1);
    
    /**
     * Reset Position auf 0
     */
    void reset();
    
    /**
     * Update - liest aktuellen Encoder-Wert und berechnet Distanz
     * Sollte regelmäßig aufgerufen werden um Überläufe zu erkennen
     * @return true wenn erfolgreich
     */
    bool update();
    
    /**
     * Hole zurückgelegte Distanz in mm
     * @return Distanz seit letztem Reset
     */
    float get_distance_mm() const { return distance_mm_; }
    
    /**
     * Hole rohe Position in Encoder-Steps
     * @return Steps seit letztem Reset
     */
    int32_t get_position_steps() const { return total_steps_; }
    
    /**
     * Hole Anzahl voller Umdrehungen
     * @return Umdrehungen seit Reset
     */
    int32_t get_revolutions() const { return revolutions_; }
    
    /**
     * Setze Wellendurchmesser (für Kalibrierung)
     * @param diameter_mm Neuer Durchmesser in mm
     */
    void set_wheel_diameter(float diameter_mm);
    
    /**
     * Hole aktuellen Wellendurchmesser
     */
    float get_wheel_diameter() const { return wheel_diameter_mm_; }
    
    /**
     * Hole Radumfang
     */
    float get_wheel_circumference() const { return wheel_circumference_mm_; }
    
    /**
     * Prüfe ob Kalibrierung abgeschlossen ist
     */
    bool is_calibrated() const { return is_calibrated_; }
    
    /**
     * Hole aktuelle Geschwindigkeit in mm/s
     * @return Filament-Geschwindigkeit (kann negativ sein bei Rückzug)
     */
    float get_speed_mm_per_sec() const { return speed_mm_per_sec_; }
    
    /**
     * Hole Encoder Diagnostics (Magnitude, AGC, Flags)
     * @return Diagnostics data or nullopt if read failed
     */
    std::optional<EncoderDiagnostics> get_diagnostics();
    
    /**
     * Starte LUT-Kalibrierung
     */
    void start_lut_calibration();
    
    /**
     * Beende LUT-Kalibrierung und speichere im Flash
     */
    void end_lut_calibration();
    
    /**
     * Prüfe ob LUT-Kalibrierung aktiv ist
     */
    bool is_lut_calibrating() const { return lut_calibrating_; }
    
private:
    AS5048A& encoder_;
    float wheel_diameter_mm_;
    float wheel_circumference_mm_;
    int8_t direction_;
    
    // State
    uint16_t last_angle_;       // Letzter Encoder-Winkel (0-16383)
    int32_t revolutions_;       // Anzahl voller Umdrehungen
    int32_t total_steps_;       // Gesamt-Steps seit Reset
    float distance_mm_;         // Berechnete Distanz in mm
    float speed_mm_per_sec_;    // Aktuelle Geschwindigkeit in mm/s
    
    bool initialized_;
    
    // Für Geschwindigkeits-Berechnung
    float last_distance_mm_;
    uint64_t last_update_time_us_;
    
    // Moving Average Filter für Rausch-Unterdrückung
    static constexpr int FILTER_SIZE = 5;
    uint16_t angle_history_[FILTER_SIZE];
    int filter_index_;
    int filter_count_;
    
    // Auto-Kalibrierung für Rausch-Schwellwert
    static constexpr int CALIBRATION_SAMPLES = 50;
    int calibration_count_;
    uint16_t calibration_samples_[CALIBRATION_SAMPLES];
    float noise_threshold_;  // Adaptive Schwelle basierend auf gemessenem Rauschen
    bool is_calibrated_;
    
    // LUT Kalibrierung
    static constexpr int LUT_SIZE = 16384;  // Für AS5048A (14-bit)
    int16_t* lut_;  // Look-Up Table für Winkelkorrektur
    bool lut_calibrating_;
    int lut_sample_count_;
    uint16_t lut_start_angle_;
    int32_t lut_expected_steps_;  // Erwartete Steps seit Kalibrierungsstart
    uint16_t lut_last_angle_;     // Letzter Winkel für Delta-Berechnung
    
    /**
     * Berechne Distanz aus Steps
     */
    void calculate_distance();
    
    /**
     * Berechne Geschwindigkeit aus Delta Distance und Delta Time
     */
    void calculate_speed();
    
    /**
     * Gleitender Durchschnitt über die letzten FILTER_SIZE Messungen
     */
    uint16_t get_filtered_angle(uint16_t new_angle);
    
    /**
     * Auto-Kalibrierung: Misst Rauschen im Stillstand
     */
    void calibrate_noise_threshold();
    
    /**
     * Wende LUT-Korrektur auf Winkel an
     */
    uint16_t apply_lut_correction(uint16_t raw_angle);
    
    /**
     * Speichere LUT im Flash
     */
    void save_lut_to_flash();
    
    /**
     * Lade LUT aus Flash
     */
    void load_lut_from_flash();
};

}  // namespace encoder
