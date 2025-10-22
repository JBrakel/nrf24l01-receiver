#ifndef NRF_H
#define NRF_H

#include "nrf24l01.h"
#include "config.h"
#include "spi.h"
#include "shared.h"
#include <stdint.h>
#include <util/delay.h>

/**
 * @file nrf.h
 * @brief High-level interface for nRF24L01+ transceiver functions.
 * Provides initialization, configuration, and SPI register operations.
 */

// ============================================================
// Initialization and Configuration
// ============================================================

/** @brief Initialize the nRF24L01 module and basic SPI settings. */
void nrf_init(void);

/** @brief Configure the nRF24L01 as a transmitter (TX mode). */
void nrf_config_tx(void);

/** @brief Configure the nRF24L01 as a receiver (RX mode). */
void nrf_config_rx(void);

// ============================================================
// Register Access
// ============================================================

/** @brief Write a single byte to an nRF24L01 register. */
void nrf_write_register(uint8_t reg, uint8_t value);

/** @brief Write multiple bytes to an nRF24L01 register. */
void nrf_write_register_multi(uint8_t reg, uint8_t *data, uint8_t len);

/** @brief Read a single byte from an nRF24L01 register. */
uint8_t nrf_read_register(uint8_t reg);

/** @brief Read multiple bytes from an nRF24L01 register. */
void nrf_read_register_multi(uint8_t reg, uint8_t *data, uint8_t len);

// ============================================================
// Payload & FIFO Operations
// ============================================================

/** @brief Flush the TX FIFO buffer. */
void nrf_flush_tx(void);

/** @brief Flush the RX FIFO buffer. */
void nrf_flush_rx(void);

/** @brief Write a payload to the TX FIFO and transmit it. */
void nrf_write_payload(uint8_t *data, uint8_t len);

/** @brief Read the received payload from the RX FIFO. */
void nrf_read_payload(uint8_t *data, uint8_t len);

// ============================================================
// Debug & Test Utilities
// ============================================================

/** @brief Simple test to verify communication with the nRF24L01. */
void nrf_test(void);

/** @brief Test function to verify multi-byte register read/write. */
void nrf_test_multi(void);

/** @brief Print the value of a single nRF24L01 register for debugging. */
void debug_register(const char* name, uint8_t reg);

/** @brief Print the contents of a multi-byte nRF24L01 register for debugging. */
void debug_register_multi(char* name, uint8_t reg, uint8_t len);

#endif // NRF_H
