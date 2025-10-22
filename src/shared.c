#include "shared.h"

void debug_print_bits(uint8_t value) {
  char hex[10];
  sprintf(hex, " | 0x%02X", value);
  for (int i = 7; i >= 0; i--) {
      usart_tx(((value >> i) & 1) ? "1" : "0");
      if(i==4) usart_tx(" ");
  }
  usart_tx(hex);
}