#include "pwm.h"

void led_init() {
    DDRD |= (1 << LED_PIN_LEFT) | (1 << LED_PIN_UP) | (1 << LED_PIN_DOWN); // PD3, PD5, PD6
    DDRB |= (1 << LED_PIN_RIGHT); // PB2
}

void pwm_init(){
    // timer 0 -> PD5 PD6
    // TCCR0A – Timer/Counter Control Register A
    TCCR0A |= (1 << WGM01) | (1 << WGM00);    // Waveform Generation Mode: Fast PWM
    TCCR0A |= (1 << COM0A1);                  // Clear OC0A on compare match
    TCCR0A |= (1 << COM0B1);                  // Non-inverting mode for OC0B (PD5)
    OCR0A = 0;
    TCCR0B |= (1 << CS01) | (1 << CS00); // Prescaler = 64, starts PWM
    // TIMSK0 |= (1 << TOIE0);

    // timer 1 -> PD4
    TCCR1A |= (1 << WGM10) | (1 << WGM12); // Fast PWM mode, 8-bit
    TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler = 64, starts PWM
    TCCR1A |= (1 << COM1B1); // Non-inverting mode for OC1B (PD4)

    // timer 2 -> PD3
    TCCR2A |= (1 << WGM20) | (1 << WGM21); // Fast PWM mode
    TCCR2A |= (1 << COM2B1); // Non-inverting mode for OC2B (PD3)
    TCCR2B |= (1 << CS22); // Prescaler = 64, starts PWM

}

void pwm_set(uint8_t led_pin, uint8_t value) {
    switch (led_pin) {
        case LED_PIN_LEFT:
            OCR0A = value; // PD6
            break;
        case LED_PIN_UP:
            OCR0B = value; // PD5
            break;
        case LED_PIN_RIGHT:
            OCR1B = value; // PB2
            break;
        case LED_PIN_DOWN:
            OCR2B = value; // PD3
            break;
        default:
            break;
    }
}

void pwm_control_axis(uint8_t pin1, uint8_t pin2, int16_t value, uint8_t pwm_value, uint8_t threshold, uint8_t *is_active){
  if(value > threshold){
    pwm_set(pin1, pwm_value);
    pwm_set(pin2, 0);
    *is_active = 1;
  }
  else if(value < -threshold){
    pwm_set(pin1, 0);
    pwm_set(pin2, pwm_value);
    *is_active = 1;
  }
  else{
    pwm_set(pin1, 0);
    pwm_set(pin2, 0);
    *is_active = *is_active ? 1 : 0;
  }
}

int16_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_max, int32_t out_min)
{
    int16_t value = ((x - in_min) * (out_min - out_max) / (in_max - in_min) + out_max);
    if(value < out_min) return (int16_t)out_min;
    if(value > out_max) return (int16_t)out_max;
    return value;
}