#ifndef SPI_H
#define SPI_H

#include <stdio.h>
#include <avr/io.h>
#include "usart.h"
#include "config.h"

// =========================
// SPI Function Prototypes
// =========================
/**
 * @brief Initialize the SPI peripheral and configure pins
 */
void spi_init(void);

/**
 * @brief Transmit a byte over SPI and receive the response
 * @param data Byte to send
 * @return Byte received from SPI slave
 */
uint8_t spi_tx(uint8_t data);

/**
 * @brief Simple test function for SPI (e.g., prints test results)
 */
void spi_test(void);


#endif // SPI_H