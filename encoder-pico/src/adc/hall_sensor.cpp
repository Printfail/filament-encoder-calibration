#include "hall_sensor.hpp"
#include "pico/stdlib.h"
#include <cstdio>

namespace adc {

HallSensor::HallSensor(uint8_t adc1_pin, uint8_t adc2_pin)
    : adc1_pin_(adc1_pin),
      adc2_pin_(adc2_pin),
      adc1_channel_(0),
      adc2_channel_(0),
      initialized_(false) {
}

bool HallSensor::init() {
    printf("HallSensor: Initializing ADC...\n");
    
    // ADC initialisieren
    adc_init();
    
    // GPIO Pins zu ADC Kanälen konvertieren
    adc1_channel_ = gpio_to_adc_channel(adc1_pin_);
    adc2_channel_ = gpio_to_adc_channel(adc2_pin_);
    
    if (adc1_channel_ > 3 || adc2_channel_ > 3) {
        printf("HallSensor: ERROR - Invalid ADC pins!\n");
        printf("  ADC1 Pin: %d (Channel: %d)\n", adc1_pin_, adc1_channel_);
        printf("  ADC2 Pin: %d (Channel: %d)\n", adc2_pin_, adc2_channel_);
        return false;
    }
    
    // GPIO Pins als ADC konfigurieren
    adc_gpio_init(adc1_pin_);
    adc_gpio_init(adc2_pin_);
    
    initialized_ = true;
    
    printf("HallSensor: Initialized successfully\n");
    printf("  ADC1: GPIO %d (Channel %d)\n", adc1_pin_, adc1_channel_);
    printf("  ADC2: GPIO %d (Channel %d)\n", adc2_pin_, adc2_channel_);
    
    return true;
}

uint16_t HallSensor::read_adc1() {
    if (!initialized_) {
        return 0;
    }
    
    // Wähle ADC Kanal
    adc_select_input(adc1_channel_);
    
    // Lese ADC Wert (12-bit: 0-4095)
    return adc_read();
}

uint16_t HallSensor::read_adc2() {
    if (!initialized_) {
        return 0;
    }
    
    // Wähle ADC Kanal
    adc_select_input(adc2_channel_);
    
    // Lese ADC Wert (12-bit: 0-4095)
    return adc_read();
}

void HallSensor::read_both(uint16_t* adc1_out, uint16_t* adc2_out) {
    if (!initialized_) {
        *adc1_out = 0;
        *adc2_out = 0;
        return;
    }
    
    // Lese ADC1
    adc_select_input(adc1_channel_);
    *adc1_out = adc_read();
    
    // Kleine Pause zwischen Messungen
    sleep_us(10);
    
    // Lese ADC2
    adc_select_input(adc2_channel_);
    *adc2_out = adc_read();
}

float HallSensor::adc_to_voltage(uint16_t adc_value) {
    // 12-bit ADC: 0-4095 entspricht 0-3.3V
    return (adc_value * 3.3f) / 4095.0f;
}

uint8_t HallSensor::gpio_to_adc_channel(uint8_t gpio_pin) {
    // GPIO 26 = ADC0
    // GPIO 27 = ADC1
    // GPIO 28 = ADC2
    // GPIO 29 = ADC3
    if (gpio_pin >= 26 && gpio_pin <= 29) {
        return gpio_pin - 26;
    }
    return 255; // Ungültiger Kanal
}

} // namespace adc
