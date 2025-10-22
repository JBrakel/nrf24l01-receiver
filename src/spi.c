#include "spi.h"


// SPI CONFIGURATION
void spi_init(){
  // Set SS as output to keep SPI master mode stable
  DDRB  |= PIN_SS;
  // Set SS high to deselect any SPI slave initially
  PORTB |= PIN_SS;
  // Outputs
  DDRB |= PIN_CE | PIN_CSN | PIN_MOSI | PIN_SCK;
  // Inputs
  DDRB &= ~PIN_MISO;
  // Enable SPI, Master & fosc/16
  // SPCR – SPI Control Register
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
  // Deselect chip & keep chip idle after boot
  CSN_HIGH();
  CE_LOW();
}

uint8_t spi_tx(uint8_t data){
  // Start transmission
  // SPDR – SPI Data Register
  SPDR = data;
  // Wait for transmission complete
  // SPSR – SPI Status Register
  while(!(SPSR & (1 << SPIF)));
  return SPDR;
}

void spi_test(){
  char buffer2[80];
  for (int i = 0; i < 10; i++) {
    CSN_LOW();
    uint8_t status = spi_tx(0xFF);
    uint8_t value = spi_tx(0xFF);
    CSN_HIGH();
    sprintf(buffer2, "status=%02X value=%02X\n", status, value);
    usart_tx(buffer2);
  }
}