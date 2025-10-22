#include "usart.h"

void usart_init(){
  // Set baud rate
  // UBRRnL and UBRRnH – USART Baud Rate Registers
  uint16_t prescale = ((CPU_F)/(16UL*BAUDRATE) - 1);  
  UBRR0H = (unsigned char)(prescale >> 8);
  UBRR0L = (unsigned char)prescale;
  
  // Enable receiver and transmitter
  // UCSRnB – USART Control and Status Register n B
  UCSR0B = (1 << TXEN0) | (1 << RXEN0);

  // Frame format: 8N1 - 8 data bit, no parity bit,  1 stop bit
  // UCSRnC – USART Control and Status Register n C
  UCSR0C = (0 << UCSZ02) | (1 << UCSZ01) | (1 << UCSZ00);
  UCSR0C |= (0 << UPM00) | (0 << UPM01);
  UCSR0C |= (0 << USBS0);
}

void usart_tx(char *data){
  uint16_t i = 0;
  while(data[i] != 0){
    // Wait for empty transmit buffer
    // UCSRnA – USART Control and Status Register n A
    while(!(UCSR0A & (1 << UDRE0))){};
    // Put data into buffer, sends the data
    // UDRn – USART I/O Data Register n
    UDR0 = data[i];
    i++;
  }
}