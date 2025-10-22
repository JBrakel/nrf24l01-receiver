#include "adc.h"

void adc_init(){
  // ADMUX – ADC Multiplexer Selection Register
  ADMUX = (0 << REFS1) | (1 << REFS0);
  // ADCSRA – ADC Control and Status Register A
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read(uint8_t channel){
  // select channel
  ADMUX &= ~((1 << MUX3) | (1 << MUX2)| (1 << MUX1) | (1 << MUX0));
  ADMUX |= (channel & 0x0F);
  // start dummy conversion
  // ADCSRA – ADC Control and Status Register A
  ADCSRA |= (1 << ADSC);
  while(ADCSRA & (1 << ADSC)){};
  // ADCL and ADCH – The ADC Data Register
  return ADC;
}