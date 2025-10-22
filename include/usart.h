#ifndef USART_H
#define USART_H

#include <stdio.h>
#include <avr/io.h>
#include "config.h"

// =========================
// USART Function Prototypes
// =========================
/**
 * @brief Initialize USART0 with baudrate and 8N1 frame format
 */
void usart_init(void);

/**
 * @brief Transmit a null-terminated string via USART0
 * @param data Pointer to null-terminated string
 */
void usart_tx(char *data);

#endif // USART_H