#include "as5048a.hpp"
#include <cmath>
#include <cstdio>
#include "pico/stdlib.h"

namespace encoder {

AS5048A::AS5048A(spi_inst_t* spi_inst, uint cs_pin)
    : spi_(spi_inst), cs_pin_(cs_pin) {
    // Configure CS pin as output, initially high (inactive)
    gpio_init(cs_pin_);
    gpio_set_dir(cs_pin_, GPIO_OUT);
    gpio_put(cs_pin_, 1);  // CS high = inactive
}

bool AS5048A::init() {
    // Test communication by reading angle
    auto angle = read_angle();
    if (!angle.has_value()) {
        printf("AS5048A: Failed to communicate via SPI\n");
        return false;
    }
    
    // Check if magnet is detected
    if (!is_magnet_detected()) {
        printf("AS5048A: WARNING - No magnet detected or magnet too weak/strong\n");
        // Don't fail init, just warn
    }
    
    printf("AS5048A: Initialized successfully via SPI (angle=%u)\n", angle.value());
    return true;
}

std::optional<uint16_t> AS5048A::read_angle() {
    // Read angle register (0x3FFF = NOP, returns angle data)
    auto angle_raw = read_register(REG_ANGLE);
    if (!angle_raw.has_value()) {
        return std::nullopt;
    }
    
    // Data is already 14-bit
    return angle_raw.value();
}

std::optional<float> AS5048A::read_angle_degrees() {
    auto angle = read_angle();
    if (!angle.has_value()) {
        return std::nullopt;
    }
    
    // Convert to degrees: angle * 360.0 / 16384
    float degrees = (angle.value() * 360.0f) / RESOLUTION;
    return degrees;
}

std::optional<uint16_t> AS5048A::read_magnitude() {
    return read_register(REG_MAGNITUDE);
}

std::optional<uint8_t> AS5048A::read_diagnostics() {
    auto diag16 = read_register(REG_DIAGNOSTICS);
    if (!diag16.has_value()) {
        return std::nullopt;
    }
    // Return lower 8 bits
    return static_cast<uint8_t>(diag16.value() & 0xFF);
}

bool AS5048A::is_magnet_detected() {
    auto diag = read_diagnostics();
    if (!diag.has_value()) {
        return false;
    }
    
    uint8_t diagnostics = diag.value();
    
    // Bit 3: COF (CORDIC overflow) - magnet too strong
    // Bit 4: OCF (Offset compensation finished)
    // Bit 5: Magnet too weak
    bool magnet_too_weak = (diagnostics & (1 << 5)) != 0;
    bool magnet_too_strong = (diagnostics & (1 << 3)) != 0;
    
    return !magnet_too_weak && !magnet_too_strong;
}

uint16_t AS5048A::spi_transfer(uint16_t command) {
    // CS low (active)
    gpio_put(cs_pin_, 0);
    sleep_us(1);  // CS setup time
    
    // Send 16-bit command and receive 16-bit response
    uint8_t tx_buf[2] = {
        static_cast<uint8_t>((command >> 8) & 0xFF),  // MSB first
        static_cast<uint8_t>(command & 0xFF)
    };
    uint8_t rx_buf[2];
    
    spi_write_read_blocking(spi_, tx_buf, rx_buf, 2);
    
    // CS high (inactive)
    sleep_us(1);  // CS hold time
    gpio_put(cs_pin_, 1);
    sleep_us(10);  // Inter-frame delay
    
    // Combine received bytes
    uint16_t response = (rx_buf[0] << 8) | rx_buf[1];
    return response;
}

bool AS5048A::calculate_parity(uint16_t value) {
    // Calculate even parity for lower 15 bits
    uint16_t masked = value & 0x7FFF;  // Mask to 15 bits
    bool parity = false;
    
    while (masked) {
        parity = !parity;
        masked = masked & (masked - 1);  // Clear lowest set bit
    }
    
    return parity;
}

std::optional<uint16_t> AS5048A::read_register(uint16_t reg) {
    // Build command: bit 14 = Read (1), bits [13:0] = address
    uint16_t command = CMD_READ | (reg & 0x3FFF);
    
    // Add parity bit (bit 15)
    if (calculate_parity(command)) {
        command |= CMD_PARITY;
    }
    
    // First transfer: Send read command (response is previous data, ignore)
    spi_transfer(command);
    
    // Second transfer: Send NOP to receive the data
    uint16_t nop_cmd = CMD_READ | 0x0000;  // NOP command
    if (calculate_parity(nop_cmd)) {
        nop_cmd |= CMD_PARITY;
    }
    
    uint16_t response = spi_transfer(nop_cmd);
    
    // Extract 14-bit data (bits [13:0])
    uint16_t data = response & 0x3FFF;
    
    // Check parity
    bool received_parity = (response & CMD_PARITY) != 0;
    bool calculated_parity = calculate_parity(response & 0x7FFF);
    
    if (received_parity != calculated_parity) {
        printf("AS5048A: Parity error!\n");
        return std::nullopt;
    }
    
    return data;
}

}  // namespace encoder
