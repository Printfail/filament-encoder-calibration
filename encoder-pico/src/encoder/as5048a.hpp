#pragma once

#include "hardware/spi.h"
#include "hardware/gpio.h"
#include <cstdint>
#include <optional>

namespace encoder {

/**
 * AS5048A Magnetic Encoder Driver
 * 14-bit absolute position encoder
 * Communication: SPI
 */
class AS5048A {
public:
    static constexpr uint16_t RESOLUTION = 16384;  // 2^14 = 16384 steps/revolution
    
    // SPI Register Addresses (14-bit addresses)
    static constexpr uint16_t REG_ANGLE = 0x3FFF;      // Read angle (all 1s = NOP/Read)
    static constexpr uint16_t REG_MAGNITUDE = 0x3FFE;   // Magnitude register
    static constexpr uint16_t REG_DIAGNOSTICS_AGC = 0x3FFD; // Diagnostics + AGC (bits[7:0]=AGC, bits[13:8]=flags)
    // Deprecated - use REG_DIAGNOSTICS_AGC instead
    static constexpr uint16_t REG_AGC = 0x3FFD;         // Same as DIAGNOSTICS_AGC
    static constexpr uint16_t REG_DIAGNOSTICS = 0x3FFD; // Same as DIAGNOSTICS_AGC
    
    // SPI Command bits
    static constexpr uint16_t CMD_READ = 0x4000;       // Read bit (bit 14)
    static constexpr uint16_t CMD_PARITY = 0x8000;     // Parity bit (bit 15)
    
    /**
     * Constructor
     * @param spi_inst SPI instance (spi0 or spi1)
     * @param cs_pin Chip select GPIO pin
     */
    AS5048A(spi_inst_t* spi_inst, uint cs_pin);
    
    /**
     * Initialize encoder (test communication)
     * @return true if successful
     */
    bool init();
    
    /**
     * Read raw angle position (0-16383)
     * @return Position or nullopt if read failed
     */
    std::optional<uint16_t> read_angle();
    
    /**
     * Read angle in degrees (0.0 - 359.99°)
     * @return Angle in degrees or nullopt if read failed
     */
    std::optional<float> read_angle_degrees();
    
    /**
     * Read magnetic field strength
     * @return Magnitude or nullopt if read failed
     */
    std::optional<uint16_t> read_magnitude();
    
    /**
     * Read diagnostics register
     * @return Diagnostics byte or nullopt if read failed
     */
    std::optional<uint8_t> read_diagnostics();
    
    /**
     * Read AGC (Automatic Gain Control) value
     * @return AGC value (0-255) or nullopt if read failed
     */
    std::optional<uint8_t> read_agc();
    
    /**
     * Check if magnet is detected properly
     * @return true if magnet OK
     */
    bool is_magnet_detected();
    
private:
    spi_inst_t* spi_;
    uint cs_pin_;
    
    /**
     * SPI transfer with CS control
     */
    uint16_t spi_transfer(uint16_t command);
    
    /**
     * Calculate even parity for 15-bit value
     */
    static bool calculate_parity(uint16_t value);
    
    /**
     * Read 16-bit register via SPI
     */
    std::optional<uint16_t> read_register(uint16_t reg);
};

}  // namespace encoder
