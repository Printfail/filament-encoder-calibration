#pragma once

#include "ble/att_server.h"
#include "btstack.h"
#include <cstdint>
#include <span>

namespace gatt {

/**
 * BLE GATT Service für Encoder
 * Ermöglicht Klipper Zugriff auf Encoder-Daten
 */
class EncoderService {
public:
    // UUID: 12345678-1234-1234-1234-123456789abc (Encoder Service)
    static constexpr const char* SERVICE_UUID = "12345678-1234-1234-1234-123456789abc";
    
    // Characteristics UUIDs
    static constexpr const char* CHAR_UUID_POSITION = "12345678-1234-1234-1234-123456789001";
    static constexpr const char* CHAR_UUID_RESET = "12345678-1234-1234-1234-123456789002";
    static constexpr const char* CHAR_UUID_CONFIG = "12345678-1234-1234-1234-123456789003";
    
    /**
     * Initialize service
     */
    static void init();
    
    /**
     * Update encoder position value (wird von main loop aufgerufen)
     * @param position_steps Aktuelle Position in Steps
     * @param distance_mm Aktuelle Distanz in mm
     * @param speed_mm_per_sec Aktuelle Geschwindigkeit in mm/s
     */
    static void update_position(int32_t position_steps, float distance_mm, float speed_mm_per_sec);
    
    /**
     * ATT Read Handler
     */
    static uint16_t att_read_callback(
        hci_con_handle_t con_handle,
        uint16_t att_handle,
        uint16_t offset,
        uint8_t* buffer,
        uint16_t buffer_size
    );
    
    /**
     * ATT Write Handler
     */
    static int att_write_callback(
        hci_con_handle_t con_handle,
        uint16_t att_handle,
        uint16_t transaction_mode,
        uint16_t offset,
        uint8_t* buffer,
        uint16_t buffer_size
    );
    
    /**
     * Setze Reset-Callback (wird aufgerufen wenn Reset-Command empfangen)
     */
    static void set_reset_callback(void (*callback)());
    
    /**
     * Setze Config-Callback (wird aufgerufen wenn Konfiguration geändert)
     */
    static void set_config_callback(void (*callback)(float wheel_diameter));

    // ATT Handles (werden von gatt.cpp gesetzt)
    static uint16_t handle_position_;
    static uint16_t handle_reset_;
    static uint16_t handle_config_;

private:
    
    // Aktuelle Werte
    static int32_t current_position_steps_;
    static float current_distance_mm_;
    static float current_speed_mm_per_sec_;
    static float current_wheel_diameter_;
    
    // Callbacks
    static void (*reset_callback_)();
    static void (*config_callback_)(float);
};

}  // namespace gatt
