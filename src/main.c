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

#define MOTOR_FL LED_PIN_RIGHT // CW
#define MOTOR_FR LED_PIN_DOWN  // CCW
#define MOTOR_RL LED_PIN_UP    // CCW
#define MOTOR_RR LED_PIN_LEFT  // CW

typedef struct {
  float kp;
  float ki;
  float kd;

  float error_prev;
  float integral;
} PID;


// TODO rename LED_PIN to MOTOR positions


// Global variables
volatile uint8_t IS_NEW_DATA = 0;
volatile DualJoystickPacket joysticks;

PID pid_roll  = { .kp = 1.2, .ki = 0.0, .kd = 0.04 };
PID pid_pitch = { .kp = 1.2, .ki = 0.0, .kd = 0.04 };
PID pid_yaw   = { .kp = 2.0, .ki = 0.0, .kd = 0.0 };

// PID controller
float pid_compute(PID *pid, float desired, float measured, float dt){
  float error = desired - measured;
  pid->integral += error * dt;
  
  float derivative = (error - pid->error_prev) / dt;
  float output = pid->kp * error + 
                 pid->ki * pid->integral + 
                 pid->kd * derivative;

  pid->error_prev = error;
  return output;
}

// Interrupts
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
  int16_t yaw = 0;
  int16_t pitch = 0;
  int16_t roll = 0;
  int16_t pwm_fl = 0;
  int16_t pwm_fr = 0;
  int16_t pwm_rl = 0;
  int16_t pwm_rr = 0;

  uint8_t step_throttle = 10;
  uint8_t step_yaw = 10;
  uint8_t step_pitch = 10;
  uint8_t step_roll = 10;

  uint8_t IS_DEBUGGING = 0;
  uint8_t IS_PID = 0;
  
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

      if(!IS_DEBUGGING){

        // Deadzone for joystick
        if(abs(y_left) < threshold) y_left = 0;
        if(abs(x_left) < threshold) x_left = 0;
        if(abs(y_right) < threshold) y_right = 0;
        if(abs(x_right) < threshold) x_right = 0;

        // --------------------------------------------------------

        // Joysticks - Desired behavior
        // Left joystick (y-axis): Throttle 
        throttle += (y_left > 0) ? step_throttle : (y_left < 0) ? -step_throttle : 0; 

        if(throttle > 255) throttle = 255;
        if(throttle < 0) throttle = 0;

        // Left joystick (x-axis): Yaw
        yaw = (x_left > 0) ? step_yaw : (x_left < 0) ? -step_yaw : 0;

        // Righ joystick (y-axis):
        pitch = (y_right > 0) ? step_pitch : (y_right < 0) ? -step_pitch : 0;

        // Righ joystick (x-axis):
        roll = (x_right > 0) ? step_roll : (x_right < 0) ? -step_roll : 0;

        // --------------------------------------------------------

        // // IMU Sensor - Measured behavior
        float sensor_roll  = 0.0f;
        float sensor_pitch = 0.0f;
        float sensor_yaw   = 0.0f;

        // Example: simulate small drift
        sensor_roll += 0.5;
        sensor_pitch -= 0.2;

        // --------------------------------------------------------

        // PID controller
        if(IS_PID){
          float dt = 0.01;
          float pid_roll_out = pid_compute(&pid_roll, roll, sensor_roll, dt);
          float pid_pitch_out = pid_compute(&pid_pitch, pitch, sensor_pitch, dt);
          float pid_yaw_out = pid_compute(&pid_yaw, yaw, sensor_yaw, dt);

          pwm_fl = throttle + pid_roll_out - pid_pitch_out + pid_yaw_out;
          pwm_fr = throttle - pid_roll_out - pid_pitch_out - pid_yaw_out;
          pwm_rr = throttle - pid_roll_out + pid_pitch_out + pid_yaw_out;
          pwm_rl = throttle + pid_roll_out + pid_pitch_out - pid_yaw_out;
        }
        
        // --------------------------------------------------------

        // Motor PWM calculation
        else{
          pwm_fl = throttle + roll - pitch + yaw;
          pwm_fr = throttle - roll - pitch - yaw;
          pwm_rr = throttle - roll + pitch + yaw;
          pwm_rl = throttle + roll + pitch - yaw;
        }
        
        // Clamping
        if(pwm_fl > 255) pwm_fl = 255; if(pwm_fl < 0) pwm_fl = 0;
        if(pwm_fr > 255) pwm_fr = 255; if(pwm_fr < 0) pwm_fr = 0;
        if(pwm_rl > 255) pwm_rl = 255; if(pwm_rl < 0) pwm_rl = 0;
        if(pwm_rr > 255) pwm_rr = 255; if(pwm_rr < 0) pwm_rr = 0;

        // Apply PWM values to motors
        pwm_set(MOTOR_FL, pwm_fl);
        pwm_set(MOTOR_FR, pwm_fr);
        pwm_set(MOTOR_RL, pwm_rl);
        pwm_set(MOTOR_RR, pwm_rr);

      }

      // --------------------------------------------------------

      else{
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

      // Printing
      sprintf(buffer, "(%4d, %4d, %4d), (%4d, %4d, %4d)", x_left, y_left, pressed_left, x_right, y_right, pressed_right);
      usart_tx(buffer);  
      usart_tx("\n");
    }
  }
} 
