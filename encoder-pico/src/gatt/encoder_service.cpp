#include "encoder_service.hpp"
#include <cstdio>
#include <cstring>
#include <algorithm>  // für std::min

namespace gatt {

// Static members
uint16_t EncoderService::handle_position_ = 0;
uint16_t EncoderService::handle_reset_ = 0;
uint16_t EncoderService::handle_config_ = 0;
uint16_t EncoderService::handle_diagnostics_ = 0;
uint16_t EncoderService::handle_adc_ = 0;

int32_t EncoderService::current_position_steps_ = 0;
float EncoderService::current_distance_mm_ = 0.0f;
float EncoderService::current_speed_mm_per_sec_ = 0.0f;
float EncoderService::current_wheel_diameter_ = 10.0f;

uint16_t EncoderService::current_magnitude_ = 0;
uint8_t EncoderService::current_agc_ = 0;
uint8_t EncoderService::current_diagnostics_ = 0;

uint16_t EncoderService::current_adc1_ = 0;
uint16_t EncoderService::current_adc2_ = 0;

// Diagnostics static members
bool EncoderService::diagnostics_serial_mode_ = false;

// Callbacks
void (*EncoderService::reset_callback_)() = nullptr;
void (*EncoderService::config_callback_)(float) = nullptr;
void (*EncoderService::lut_calibration_start_callback_)() = nullptr;
void (*EncoderService::lut_calibration_end_callback_)() = nullptr;

void EncoderService::init() {
    printf("EncoderService: Initialized\n");
    // ATT Handles werden später von BTstack gesetzt
    // Siehe gatt.cpp für Service-Registrierung
}

void EncoderService::update_position(int32_t position_steps, float distance_mm, float speed_mm_per_sec) {
    current_position_steps_ = position_steps;
    current_distance_mm_ = distance_mm;
    current_speed_mm_per_sec_ = speed_mm_per_sec;
    
    // Optional: Sende BLE Notification wenn sich Wert signifikant ändert
    // TODO: Implementiere Notifications für Live-Updates
}

void EncoderService::update_diagnostics(uint16_t magnitude, uint8_t agc, uint8_t diagnostics) {
    current_magnitude_ = magnitude;
    current_agc_ = agc;
    current_diagnostics_ = diagnostics;
}

void EncoderService::update_adc(uint16_t adc1, uint16_t adc2) {
    current_adc1_ = adc1;
    current_adc2_ = adc2;
}

uint16_t EncoderService::att_read_callback(
    hci_con_handle_t con_handle,
    uint16_t att_handle,
    uint16_t offset,
    uint8_t* buffer,
    uint16_t buffer_size
) {
    // Position Read (int32_t steps + float mm + float speed = 12 bytes)
    if (att_handle == handle_position_) {
        if (offset > 12) return 0;
        
        uint8_t data[12];
        // Bytes 0-3: position_steps (little-endian)
        memcpy(&data[0], &current_position_steps_, 4);
        // Bytes 4-7: distance_mm (little-endian float)
        memcpy(&data[4], &current_distance_mm_, 4);
        // Bytes 8-11: speed_mm_per_sec (little-endian float)
        memcpy(&data[8], &current_speed_mm_per_sec_, 4);
        
        uint16_t bytes_to_copy = std::min(buffer_size, static_cast<uint16_t>(12 - offset));
        memcpy(buffer, &data[offset], bytes_to_copy);
        
        return bytes_to_copy;
    }
    
    // Config Read (float diameter = 4 bytes)
    if (att_handle == handle_config_) {
        if (offset > 4) return 0;
        
        uint8_t data[4];
        memcpy(data, &current_wheel_diameter_, 4);
        
        uint16_t bytes_to_copy = std::min(buffer_size, static_cast<uint16_t>(4 - offset));
        memcpy(buffer, &data[offset], bytes_to_copy);
        
        return bytes_to_copy;
    }
    
    // Diagnostics Read (uint16_t magnitude + uint8_t agc + uint8_t diag = 4 bytes)
    if (att_handle == handle_diagnostics_) {
        if (offset > 4) return 0;
        
        uint8_t data[4];
        // Bytes 0-1: magnitude (little-endian)
        memcpy(&data[0], &current_magnitude_, 2);
        // Byte 2: agc
        data[2] = current_agc_;
        // Byte 3: diagnostics flags
        data[3] = current_diagnostics_;
        
        uint16_t bytes_to_copy = std::min(buffer_size, static_cast<uint16_t>(4 - offset));
        memcpy(buffer, &data[offset], bytes_to_copy);
        
        return bytes_to_copy;
    }
    
    // ADC Read (uint16_t adc1 + uint16_t adc2 = 4 bytes)
    if (att_handle == handle_adc_) {
        if (offset > 4) return 0;
        
        uint8_t data[4];
        // Bytes 0-1: adc1 (little-endian, 12-bit value 0-4095)
        memcpy(&data[0], &current_adc1_, 2);
        // Bytes 2-3: adc2 (little-endian, 12-bit value 0-4095)
        memcpy(&data[2], &current_adc2_, 2);
        
        uint16_t bytes_to_copy = std::min(buffer_size, static_cast<uint16_t>(4 - offset));
        memcpy(buffer, &data[offset], bytes_to_copy);
        
        return bytes_to_copy;
    }
    
    return 0;  // Unknown handle
}

int EncoderService::att_write_callback(
    hci_con_handle_t con_handle,
    uint16_t att_handle,
    uint16_t transaction_mode,
    uint16_t offset,
    uint8_t* buffer,
    uint16_t buffer_size
) {
    // Reset/Command Characteristic (0x01=Reset, 0x02=StartDiag, 0x03=StopDiag)
    if (att_handle == handle_reset_) {
        if (buffer_size >= 1) {
            if (buffer[0] == 0x01) {
                printf("EncoderService: Reset command received\n");
                if (reset_callback_) {
                    reset_callback_();
                }
                return 0;  // Success
            }
            else if (buffer[0] == 0x02) {
                printf("EncoderService: Start diagnostics serial mode\n");
                diagnostics_serial_mode_ = true;
                return 0;  // Success
            }
            else if (buffer[0] == 0x03) {
                printf("EncoderService: Stop diagnostics serial mode\n");
                diagnostics_serial_mode_ = false;
                return 0;  // Success
            }
        }
    }
    
    // Config Write - Handle both diameter and calibration commands
    if (att_handle == handle_config_) {
        // Check if it's a string command (START_CAL or END_CAL)
        if (buffer_size >= 9 && memcmp(buffer, "START_CAL", 9) == 0) {
            printf("EncoderService: START_CAL command received\n");
            if (lut_calibration_start_callback_) {
                lut_calibration_start_callback_();
            }
            return 0;  // Success
        }
        else if (buffer_size >= 7 && memcmp(buffer, "END_CAL", 7) == 0) {
            printf("EncoderService: END_CAL command received\n");
            if (lut_calibration_end_callback_) {
                lut_calibration_end_callback_();
            }
            return 0;  // Success
        }
        // Otherwise it's a float diameter
        else if (buffer_size >= 4) {
            float new_diameter;
            memcpy(&new_diameter, buffer, 4);
            
            printf("EncoderService: Config update - diameter=%.3fmm\n", new_diameter);
            current_wheel_diameter_ = new_diameter;
            
            if (config_callback_) {
                config_callback_(new_diameter);
            }
            return 0;  // Success
        }
    }
    
    return ATT_ERROR_INVALID_ATTRIBUTE_VALUE_LENGTH;
}

void EncoderService::set_reset_callback(void (*callback)()) {
    reset_callback_ = callback;
}

void EncoderService::set_config_callback(void (*callback)(float)) {
    config_callback_ = callback;
}

void EncoderService::set_lut_calibration_start_callback(void (*callback)()) {
    lut_calibration_start_callback_ = callback;
}

void EncoderService::set_lut_calibration_end_callback(void (*callback)()) {
    lut_calibration_end_callback_ = callback;
}

}  // namespace gatt
