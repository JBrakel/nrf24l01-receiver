#ifndef CONFIG_H
#define CONFIG_H

#include <avr/io.h>

// --------------------------------------------------------

// SHARED CONFIG
#define CPU_F 16000000UL
#define BAUDRATE 9600
#define NRF_ADDRESS {0x00, 0x00, 0x00, 0x00, 0x01}
#define CHANNEL 76

// --------------------------------------------------------

// SETTING & CLEARING BITS
#define SET_BIT(byte, bit)    ((byte) |=  (1 << bit))
#define CLEAR_BIT(byte, bit)  ((byte) &= ~(1 << bit))

// --------------------------------------------------------

// LEDS CONFIG
#define LED_PIN_LEFT   PD6
#define LED_PIN_RIGHT  PB2
#define LED_PIN_UP     PD5
#define LED_PIN_DOWN   PD3

// --------------------------------------------------------

// JOYSTICK CONFIG
#define PIN_SW_LEFT    (1 << PD2)
#define PIN_SW_RIGHT   (1 << PD7)
#define PIN_X_LEFT     1 
#define PIN_Y_LEFT     0
#define PIN_X_RIGHT    2
#define PIN_Y_RIGHT    3


typedef struct {
    int16_t x;
    int16_t y;
    uint8_t button;
} JoystickPacket;


typedef struct {
    JoystickPacket left;
    JoystickPacket right;
} DualJoystickPacket;

#define NRF_PAYLOAD_LENGTH sizeof(DualJoystickPacket)

// --------------------------------------------------------

// SPI CONFIG
#define PIN_CE      (1 << PB1)
#define PIN_CSN     (1 << PB0)
#define PIN_SCK     (1 << PB5)
#define PIN_MOSI    (1 << PB3)
#define PIN_MISO    (1 << PB4)
#define PIN_SS      (1 << PB2)

#define CE_HIGH()   (PORTB |= PIN_CE)
#define CE_LOW()    (PORTB &= ~PIN_CE)
#define CSN_HIGH()  (PORTB |= PIN_CSN)
#define CSN_LOW()   (PORTB &= ~PIN_CSN)

// --------------------------------------------------------


#endif