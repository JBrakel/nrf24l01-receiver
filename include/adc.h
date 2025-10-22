#ifndef ADC_H
#define ADC_H

#include <stdio.h>
#include <avr/io.h>  // Needed for AVR register definitions

// =========================
// ADC FUNCTION PROTOTYPES
// =========================

/**
 * @brief Initialize the ADC peripheral
 * 
 * Configures reference voltage, prescaler, and enables the ADC module.
 * Call this once at the start of your program before using adc_read().
 */
void adc_init(void);

/**
 * @brief Read a value from the specified ADC channel
 * 
 * @param channel ADC channel number (0-7 for ATmega328P)
 * @return 10-bit ADC value (0-1023)
 */
uint16_t adc_read(uint8_t channel);

#endif // ADC_H
