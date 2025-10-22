#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>

#include "config.h"
#include "spi.h"
#include "usart.h"
#include "joystick.h"
#include "adc.h"
#include "pwm.h"
#include "nrf.h"
#include "nrf24l01.h"
#include "shared.h"

int main(){
  // Init
  usart_init();
  switch_init();
  adc_init();
  led_init();
  pwm_init();
  spi_init();
  nrf_init();
  nrf_config_rx();
  
  int16_t x_left, y_left, x_right, y_right;
  uint8_t sw_left, sw_right, pressed_left, pressed_right;
  char buffer[120];

  uint8_t IS_ACTIVE_LEFT = 0;
  uint8_t IS_ACTIVE_RIGHT = 0;

  DualJoystickPacket joysticks;
  uint8_t threshold = 5;
  while(1){
    uint8_t status = nrf_read_register(NRF_REG_STATUS);
    if (status & (1 << NRF_RX_DR)) {
        
        // --------------------------------------------------------

        // Receive Data
        nrf_read_payload((uint8_t*)&joysticks, NRF_PAYLOAD_LENGTH);

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
        uint16_t x_left_pwm_value = abs(x_left) * 2;
        uint16_t y_left_pwm_value = abs(y_left) * 2;
        uint16_t x_right_pwm_value = abs(x_right) * 2;
        uint16_t y_right_pwm_value = abs(y_right) * 2;
        
        if(x_left_pwm_value > 255)  x_left_pwm_value = 255;
        if(y_left_pwm_value > 255)  y_left_pwm_value = 255;
        if(x_right_pwm_value > 255) x_right_pwm_value = 255;
        if(y_right_pwm_value > 255) y_right_pwm_value = 255;

        // --------------------------------------------------------

        // Reset LEDs in idle case
        if (!pressed_left && !pressed_right && 
            abs(x_left) <= threshold && abs(y_left) <= threshold 
            && abs(x_right) <= threshold && abs(y_right) <= threshold){
              pwm_set(LED_PIN_LEFT, 0);
              pwm_set(LED_PIN_UP, 0);
              pwm_set(LED_PIN_RIGHT, 0);
              pwm_set(LED_PIN_DOWN, 0);

              IS_ACTIVE_LEFT = 0;
              IS_ACTIVE_RIGHT = 0;
          }

        // Evaluate buttons
        else if (pressed_left || pressed_right) {
            pwm_set(LED_PIN_LEFT, 255);
            pwm_set(LED_PIN_UP, 255);
            pwm_set(LED_PIN_RIGHT, 255);
            pwm_set(LED_PIN_DOWN, 255);
            IS_ACTIVE_LEFT = 0;  
            IS_ACTIVE_RIGHT = 0;  
        } 

        // Evaluate analog joysticks
        else{
            // --------------------------------------------------------

            // Left Joystick
            if(!IS_ACTIVE_RIGHT){
              // Horizontal control
              pwm_control_axis(LED_PIN_RIGHT, LED_PIN_LEFT, x_left, x_left_pwm_value, threshold, &IS_ACTIVE_LEFT);
              // Vertical control 
              pwm_control_axis(LED_PIN_UP, LED_PIN_DOWN, y_left, y_left_pwm_value, threshold, &IS_ACTIVE_LEFT);
            }
            

            // --------------------------------------------------------

            // Right Joystick
            if(!IS_ACTIVE_LEFT){
            // Horizontal control 
            pwm_control_axis(LED_PIN_RIGHT, LED_PIN_LEFT, x_right, x_right_pwm_value, threshold, &IS_ACTIVE_RIGHT);
            // Vertical control 
            pwm_control_axis(LED_PIN_UP, LED_PIN_DOWN, y_right, y_right_pwm_value, threshold, &IS_ACTIVE_RIGHT);
          }
        }
        
        // --------------------------------------------------------

        // Printing
        sprintf(buffer, "(%4d, %4d, %4d), (%4d, %4d, %4d)", x_left, y_left, pressed_left, x_right, y_right, pressed_right);
        usart_tx(buffer);  
        usart_tx("\n");
    }
  }
} 
