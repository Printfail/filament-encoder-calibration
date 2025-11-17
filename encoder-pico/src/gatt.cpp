#include "gatt.hpp"
#include "gatt/encoder_service.hpp"
#include "ble/att_server.h"
#include "ble/sm.h"
#include "btstack_event.h"
#include "btstack.h"
#include "encoder.h"  // Generiert von encoder.gatt!
#include <cstdio>
#include <cstring>

namespace gatt {

// GATT Database wird aus encoder.gatt generiert (bluetooth_gatt.h)
// ATT Handles werden automatisch definiert

// ========================================================================
// ATT Callbacks
// ========================================================================

static uint16_t att_read_callback(hci_con_handle_t con_handle, uint16_t att_handle, 
                                   uint16_t offset, uint8_t* buffer, uint16_t buffer_size) {
    // Delegiere an EncoderService
    return EncoderService::att_read_callback(con_handle, att_handle, offset, buffer, buffer_size);
}

static int att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, 
                               uint16_t transaction_mode, uint16_t offset, 
                               uint8_t* buffer, uint16_t buffer_size) {
    // Delegiere an EncoderService
    return EncoderService::att_write_callback(con_handle, att_handle, transaction_mode, 
                                               offset, buffer, buffer_size);
}

// ========================================================================
// Event Handler
// ========================================================================

static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t* packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    
    if (packet_type != HCI_EVENT_PACKET) return;
    
    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("GATT: Bluetooth ready\n");
                
                // Print BLE MAC Address
                bd_addr_t local_addr;
                gap_local_bd_addr(local_addr);
                printf("BLE MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                       local_addr[0], local_addr[1], local_addr[2],
                       local_addr[3], local_addr[4], local_addr[5]);
                
                // Advertising Data - EXAKT wie Nevermore (constexpr/static!)
                static const uint8_t adv_data[] = {
                    // Flags: LE General Discoverable (0x02) | BR/EDR Not Supported (0x04) = 0x06
                    0x02, 0x01, 0x06,
                    // Shortened Local Name (type 0x08): "Encoder-PicoW" (13 chars)
                    0x0E, 0x08, 'E', 'n', 'c', 'o', 'd', 'e', 'r', '-', 'P', 'i', 'c', 'o', 'W'
                };
                printf("GATT: Advertisement size = %d bytes\n", (int)sizeof(adv_data));
                
                // Setup advertisements - ALLES ZUSAMMEN wie Nevermore!
                // 300ms = 480 ticks, 500ms = 800 ticks (BLE tick = 0.625ms)
                bd_addr_t null_addr = {0};
                gap_advertisements_set_params(480, 800, 0, 0, null_addr, 0x07, 0x00);
                gap_advertisements_set_data((uint16_t)sizeof(adv_data), (uint8_t*)adv_data);
                
                // TX Power über CYW43 direkt setzen (Pico W spezifisch)
                // cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
                // Für BLE: Default ist bereits optimiert
                gap_advertisements_enable(1);
                printf("GATT: Advertising enabled\n");
                
                printf("GATT: Advertising as 'Encoder-PicoW'\n");
            }
            break;
            
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("GATT: Client disconnected\n");
            // Re-enable advertising
            gap_advertisements_enable(1);
            break;
            
        case ATT_EVENT_CONNECTED:
            printf("GATT: Client connected\n");
            break;
            
        case HCI_EVENT_LE_META:
            switch (hci_event_le_meta_get_subevent_code(packet)) {
                case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
                    printf("GATT: LE Connection complete\n");
                    break;
            }
            break;
    }
}

// ========================================================================
// Initialization
// ========================================================================

bool init() {
    printf("GATT: Initializing...\n");
    
    // Enable BTstack debug output
    hci_dump_enable_packet_log(true);
    
    // Initialize BTstack L2CAP
    l2cap_init();
    
    // Initialize Security Manager (required for BLE)
    sm_init();
    
    // Register packet handler ERST - muss static sein!
    static btstack_packet_callback_registration_t hci_event_callback_registration;
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    
    // ATT Server mit generierter GATT Database (aus encoder.gatt)
    att_server_init(profile_data, att_read_callback, att_write_callback);
    att_server_register_packet_handler(hci_packet_handler);
    
    // Set max connections - KRITISCH!
    gap_set_max_number_peripheral_connections(1);
    
    // Turn on Bluetooth - INNERHALB gatt::init() wie Nevermore!
    if (hci_power_control(HCI_POWER_ON)) {
        printf("ERROR: hci_power_control failed!\n");
        return false;
    }
    
    // ATT Handles in EncoderService setzen (aus generierter encoder.h)
    // Die generierten Handle-Namen folgen dem Schema: ATT_CHARACTERISTIC_[UUID]_01_VALUE_HANDLE
    // Wir müssen die handles manuell setzen basierend auf der GATT-Struktur:
    // Handle 0x0001: GAP_SERVICE
    // Handle 0x0002: GAP_DEVICE_NAME Characteristic Declaration
    // Handle 0x0003: GAP_DEVICE_NAME Value
    // Handle 0x0004: Encoder Service
    // Handle 0x0005: Position Characteristic Declaration
    // Handle 0x0006: Position Value
    // Handle 0x0007: Position CCC
    // Handle 0x0008: Reset Characteristic Declaration  
    // Handle 0x0009: Reset Value
    // Handle 0x000A: Config Characteristic Declaration
    // Handle 0x000B: Config Value
    // Handle 0x000C: Diagnostics Characteristic Declaration
    // Handle 0x000D: Diagnostics Value
    // Handle 0x000E: Diagnostics CCC Descriptor
    // Handle 0x000F: ADC Characteristic Declaration
    // Handle 0x0010: ADC Value
    // Handle 0x0011: ADC CCC Descriptor
    EncoderService::handle_position_ = 0x0006;
    EncoderService::handle_reset_ = 0x0009;
    EncoderService::handle_config_ = 0x000B;
    EncoderService::handle_diagnostics_ = 0x000D;
    EncoderService::handle_adc_ = 0x0010;
    
    // Initialize service
    EncoderService::init();
    
    printf("GATT: Initialized successfully\n");
    return true;
}

}  // namespace gatt
