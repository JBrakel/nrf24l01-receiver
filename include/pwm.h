#ifndef PWM_H
#define PWM_H

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "config.h"

// =========================
// PWM FUNCTION PROTOTYPES
// =========================

/**
 * @brief Initialize LED pins as outputs
 * 
 * Configures the data direction registers for all LEDs
 * used for PWM control.
 */
void led_init(void);

/**
 * @brief Initialize the timers for PWM operation
 * 
 * Sets up Timer0, Timer1, Timer2 (or used timers) in PWM mode,
 * with appropriate prescalers and output compare settings.
 */
void pwm_init(void);

/**
 * @brief Set the PWM duty cycle for a specific LED
 * 
 * @param led   Identifier for the LED (use predefined macros or enums)
 * @param value Duty cycle value (0-255)
 * 
 * Example usage:
 * pwm_set(LED_LEFT, 128);  // 50% duty cycle on left LED
 */
void pwm_set(uint8_t led, uint8_t value);

/**
 * @brief Control two LEDs along a single axis based on an input value.
 * 
 * This function sets the PWM duty cycle of two LEDs corresponding to
 * the positive and negative directions of an axis (e.g., X or Y joystick).
 * If the input value exceeds the threshold, the corresponding LED is activated,
 * otherwise both LEDs are turned off. It also updates the active status flag.
 * 
 * @param pin1       Pin number for the positive direction LED
 * @param pin2       Pin number for the negative direction LED
 * @param value      Input value for the axis (can be negative)
 * @param pwm_value  PWM duty cycle to set when active
 * @param threshold  Minimum absolute value required to activate an LED
 * @param is_active  Pointer to a flag that tracks if this axis is currently active
 * 
 * Example usage:
 * pwm_control_axis(LED_PIN_RIGHT, LED_PIN_LEFT, x_left, 128, 10, &IS_ACTIVE_LEFT);
 */
void pwm_control_axis(uint8_t pin1, uint8_t pin2, int16_t value, uint8_t pwm_value, uint8_t threshold, uint8_t *is_active);

/**
 * @brief Linearly maps a value from one range to another.
 * 
 * This function takes an input value `x` that lies within the range 
 * [`in_min`, `in_max`] and maps it proportionally to the range 
 * [`out_min`, `out_max`]. Useful for scaling sensor or joystick values
 * to PWM duty cycles or other output ranges.
 * 
 * @param x        Input value to map
 * @param in_min   Minimum value of the input range
 * @param in_max   Maximum value of the input range
 * @param out_max  Maximum value of the output range
 * @param out_min  Minimum value of the output range
 * 
 * @return int16_t Mapped value within the output range
 * 
 * Example usage:
 * int16_t pwm_val = map(512, 0, 1023, 0, 255);
 */
int16_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_max, int32_t out_min);


#endif // PWM_H
