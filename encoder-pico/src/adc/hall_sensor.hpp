#pragma once

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include <cstdint>

namespace adc {

/**
 * Hall-Sensor ADC Reader
 * Liest 2 analoge Hall-Sensoren (SS49E) für Filament-Durchmesser Messung
 */
class HallSensor {
public:
    /**
     * Constructor
     * @param adc1_pin GPIO Pin für ersten Hall-Sensor (z.B. GPIO 26 = ADC0)
     * @param adc2_pin GPIO Pin für zweiten Hall-Sensor (z.B. GPIO 27 = ADC1)
     */
    HallSensor(uint8_t adc1_pin = 26, uint8_t adc2_pin = 27);
    
    /**
     * Initialisiere ADC
     */
    bool init();
    
    /**
     * Lese ADC1 Wert (0-4095, 12-bit)
     */
    uint16_t read_adc1();
    
    /**
     * Lese ADC2 Wert (0-4095, 12-bit)
     */
    uint16_t read_adc2();
    
    /**
     * Lese beide ADC-Werte
     * @param adc1_out Pointer für ADC1 Wert
     * @param adc2_out Pointer für ADC2 Wert
     */
    void read_both(uint16_t* adc1_out, uint16_t* adc2_out);
    
    /**
     * Konvertiere ADC-Wert zu Spannung (0.0 - 3.3V)
     */
    float adc_to_voltage(uint16_t adc_value);

private:
    uint8_t adc1_pin_;
    uint8_t adc2_pin_;
    uint8_t adc1_channel_;  // ADC Kanal (0-3)
    uint8_t adc2_channel_;
    bool initialized_;
    
    /**
     * Konvertiere GPIO Pin zu ADC Kanal
     */
    uint8_t gpio_to_adc_channel(uint8_t gpio_pin);
};

} // namespace adc
