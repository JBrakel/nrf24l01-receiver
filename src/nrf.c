#include "nrf.h"

void nrf_write_register(uint8_t reg, uint8_t value){
  CSN_LOW();
  spi_tx(NRF_W_REGISTER | (reg & 0x1F));
  spi_tx(value);
  CSN_HIGH();
}

void nrf_write_register_multi(uint8_t reg, uint8_t *data, uint8_t len){
  CSN_LOW();
  spi_tx(NRF_W_REGISTER | (reg & 0x1F));
  for(uint8_t i=0; i<len; i++){
    spi_tx(data[i]);
  }
  CSN_HIGH();
}

uint8_t nrf_read_register(uint8_t reg){
  CSN_LOW();
  spi_tx(NRF_R_REGISTER | (reg & 0x1F));
  uint8_t result = spi_tx(NRF_DUMMY_BYTE);
  CSN_HIGH();
  return result;
}

void nrf_read_register_multi(uint8_t reg, uint8_t *data, uint8_t len){
  CSN_LOW();
  spi_tx(NRF_R_REGISTER | reg);
  for(uint8_t i=0; i<len; i++){
    data[i] = spi_tx(NRF_DUMMY_BYTE);
  }
  CSN_HIGH();
}

void nrf_test(){
  nrf_write_register(NRF_REG_CONFIG, 0x01);
  _delay_ms(1);

  uint8_t value = nrf_read_register(NRF_REG_CONFIG);
  while(1){
    usart_tx("CONFIG REGISTER: ");
    debug_print_bits(value);
    usart_tx("\n");
  }
}

void nrf_test_multi() {
    uint8_t tx_address[5] = NRF_ADDRESS;
    nrf_write_register_multi(NRF_REG_TX_ADDR, tx_address, 5);
    _delay_ms(1);

    uint8_t read_back[5];
    nrf_read_register_multi(NRF_REG_TX_ADDR, read_back, 5);

    usart_tx("TX_ADDR: ");
    for (uint8_t i = 0; i < 5; i++) {
        debug_print_bits(read_back[i]);
        usart_tx(" ");
    }
    usart_tx("\n");

    while(1);
}

void nrf_flush_tx(){
  CSN_LOW();
  spi_tx(NRF_FLUSH_TX);
  CSN_HIGH();
}

void nrf_flush_rx(){
  CSN_LOW();
  spi_tx(NRF_FLUSH_RX);
  CSN_HIGH();
}

void nrf_write_payload(uint8_t *data, uint8_t len){

  nrf_flush_tx();

  CSN_LOW();
  spi_tx(NRF_W_TX_PAYLOAD);
  for(uint8_t i=0; i<len; i++){
    spi_tx(data[i]);
  }
  CSN_HIGH();

  CE_HIGH();
  _delay_us(15);
  CE_LOW();
}

void nrf_init(){
  _delay_ms(200);

  nrf_write_register(NRF_REG_EN_AA, (1 << NRF_ENAA_P0));
  nrf_write_register(NRF_REG_RF_CH, NRF_RF_CH_VAL(CHANNEL));
  nrf_write_register(NRF_REG_RF_SETUP, (1 << RF_PWR1) | (1 << RF_PWR0));
  nrf_write_register(NRF_REG_RX_PW_P0, NRF_PAYLOAD_LENGTH);
  nrf_write_register(NRF_REG_SETUP_AW, (1 << AW0) | (1 << AW1));

  nrf_write_register_multi(NRF_REG_TX_ADDR, (uint8_t[])NRF_ADDRESS, 5);
  nrf_write_register_multi(NRF_REG_RX_ADDR_P0, (uint8_t[])NRF_ADDRESS, 5);

  nrf_flush_rx();
  CE_HIGH();
  _delay_ms(200);
}

void nrf_config_rx(){
    nrf_write_register(NRF_REG_CONFIG, (NRF_CONFIG_RX) | (1 << NRF_EN_CRC) | (0 << NRF_CRC0));
     _delay_ms(5);
}

void nrf_config_tx(){
    nrf_write_register(NRF_REG_CONFIG, (NRF_CONFIG_TX) | (1 << NRF_EN_CRC) | (0 << NRF_CRC0));
     _delay_ms(5);
}

void debug_register(const char* name, uint8_t reg){
  char buffer[32];
  sprintf(buffer, "%-10s", name);
  usart_tx(buffer);
  usart_tx(": ");
  uint8_t reg_rx = nrf_read_register(reg);
  debug_print_bits(reg_rx);
  usart_tx("\n");
}

void debug_register_multi(char* name, uint8_t reg, uint8_t len) {
    uint8_t buffer[5];
    nrf_read_register_multi(reg, buffer, len);
    char buffer_name[32];
    sprintf(buffer_name, "%-10s", name);
    usart_tx(buffer_name);
    usart_tx(": ");
    for (uint8_t i = 0; i < len; i++) {
        char hex[6];
        sprintf(hex, "0x%02X ", buffer[i]);
        usart_tx(hex);
    }
    usart_tx("\n");
}

void nrf_read_payload(uint8_t *data, uint8_t len)
{
  CSN_LOW();
  spi_tx(NRF_R_RX_PAYLOAD);
  for (uint8_t i = 0; i < len; i++) {
    data[i] = spi_tx(NRF_DUMMY_BYTE);
  }
  CSN_HIGH();
  nrf_write_register(NRF_REG_STATUS, (1 << NRF_RX_DR));
}