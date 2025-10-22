#include "joystick.h"

void switch_init(){
  DDRD &= ~PIN_SW_LEFT;
  DDRD &= ~PIN_SW_RIGHT;
  PORTD |= PIN_SW_LEFT;
  PORTD |= PIN_SW_RIGHT;
}