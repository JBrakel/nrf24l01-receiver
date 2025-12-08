#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "config.h"
#include "spi.h"
#include "usart.h"
#include "joystick.h"
#include "adc.h"
#include "pwm.h"
#include "nrf.h"
#include "nrf24l01.h"
#include "shared.h"

#define MOTOR_FL LED_PIN_RIGHT
#define MOTOR_FR LED_PIN_DOWN
#define MOTOR_RL LED_PIN_UP
#define MOTOR_RR LED_PIN_LEFT

// TODO rename LED_PIN to MOTOR positions

volatile uint8_t IS_NEW_DATA = 0;
volatile DualJoystickPacket joysticks;

ISR(INT0_vect){
  // Clear interrupt in NRF module
  nrf_write_register(NRF_REG_STATUS, (1 << NRF_RX_DR));

  // Receive Data
  nrf_read_payload((uint8_t*)&joysticks, NRF_PAYLOAD_LENGTH);

  // Toggle new data flag
  IS_NEW_DATA = 1;
}

int main(){
  // Disable global interrupts during setup
  cli();

  // Init
  usart_init();
  switch_init();
  adc_init();
  led_init();
  pwm_init();
  spi_init();
  nrf_init();
  nrf_config_rx();

  // Enable global interrupts
  sei();
  
  int16_t x_left, y_left, x_right, y_right;
  uint8_t sw_left, sw_right, pressed_left, pressed_right;
  char buffer[120];
  uint8_t threshold = 5;

  int16_t throttle = 0;
  uint8_t step_throttle = 10;

  uint8_t IS_DEBUGGING = 0;
  
  while(1){
    if (IS_NEW_DATA) {

      // Extract joystick data
      x_left = joysticks.left.x;
      y_left = joysticks.left.y;
      sw_left = joysticks.left.button;
      pressed_left = (sw_left == 0) ? 1 : 0;

      x_right = joysticks.right.x;
      y_right = joysticks.right.y;
      sw_right = joysticks.right.button;
      pressed_right = (sw_right == 0) ? 1 : 0;

      // --------------------------------------------------------

      // Convert for PWM
      uint16_t x_left_pwm = abs(x_left) * 2;
      uint16_t y_left_pwm = abs(y_left) * 2;
      uint16_t x_right_pwm = abs(x_right) * 2;
      uint16_t y_right_pwm = abs(y_right) * 2;
      
      // Clamp values to 255
      if(x_left_pwm > 255)  x_left_pwm = 255;
      if(y_left_pwm > 255)  y_left_pwm = 255;
      if(x_right_pwm > 255) x_right_pwm = 255;
      if(y_right_pwm > 255) y_right_pwm = 255;

      // --------------------------------------------------------

      if(IS_DEBUGGING){
        x_right = -x_right;
        y_right = -y_right;
        x_left = -x_left;
        y_left = -y_left;

        // Use highest value 
        uint16_t up_value = 0;
        if (y_left > threshold || y_right > threshold) {
            up_value = (y_left > y_right) ? y_left_pwm : y_right_pwm;
        }

        uint16_t down_value = 0;
        if (y_left < -threshold || y_right < -threshold) {
            down_value = (abs(y_left) > abs(y_right)) ? y_left_pwm : y_right_pwm;
        }

        uint16_t left_value = 0;
        if (x_left < -threshold || x_right < -threshold) {
            left_value = (abs(x_left) > abs(x_right)) ? x_left_pwm : x_right_pwm;
        }

        uint16_t right_value = 0;
        if (x_left > threshold || x_right > threshold) {
            right_value = (x_left > x_right) ? x_left_pwm : x_right_pwm;
        }

        // --------------------------------------------------------

        // Evaluate buttons
        if (pressed_left || pressed_right) {
            pwm_set(LED_PIN_LEFT, 255);
            pwm_set(LED_PIN_UP, 255);
            pwm_set(LED_PIN_RIGHT, 255);
            pwm_set(LED_PIN_DOWN, 255);
        } 

        // Idle case
        else if (x_left == 0 && y_left == 0 && x_right == 0 && y_right == 0) {
            pwm_set(LED_PIN_LEFT, 0);
            pwm_set(LED_PIN_RIGHT, 0);
            pwm_set(LED_PIN_UP, 0);
            pwm_set(LED_PIN_DOWN, 0);
        } 
        
        // Set LEDs 
        else {
            pwm_set(LED_PIN_UP, up_value);
            pwm_set(LED_PIN_DOWN, down_value);
            pwm_set(LED_PIN_LEFT, left_value);
            pwm_set(LED_PIN_RIGHT, right_value);
        }
      }

      // --------------------------------------------------------

      else{
        
        if(y_left > threshold) throttle += step_throttle;
        else if(y_left < -threshold) throttle -= step_throttle;

        if(throttle > 255) throttle = 255;
        if(throttle < 0) throttle = 0;

        pwm_set(LED_PIN_UP, throttle);
        pwm_set(LED_PIN_DOWN, throttle);
        pwm_set(LED_PIN_LEFT, throttle);
        pwm_set(LED_PIN_RIGHT, throttle);
      }
        
      // --------------------------------------------------------

      // Printing
      sprintf(buffer, "(%4d, %4d, %4d), (%4d, %4d, %4d)", x_left, y_left, pressed_left, x_right, y_right, pressed_right);
      usart_tx(buffer);  
      usart_tx("\n");
    }
  }
} 
