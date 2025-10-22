#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdio.h>
#include <avr/io.h>
#include "config.h"

// =========================
// FUNCTION PROTOTYPES
// =========================

/**
 * @brief Initialize the joystick switches as input pins
 */
void switch_init(void);

#endif // JOYSTICK_H
