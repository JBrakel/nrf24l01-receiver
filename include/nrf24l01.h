#ifndef NRF24L01_H
#define NRF24L01_H

#include <stdint.h>

// ============================================================
// nRF24L01+ SPI COMMANDS
// ============================================================

#define NRF_R_REGISTER             0x00  // Read register: 0x00–0x1F
#define NRF_W_REGISTER             0x20  // Write register: 0x20–0x3F
#define NRF_R_RX_PAYLOAD           0x61  // Read RX payload
#define NRF_W_TX_PAYLOAD           0xA0  // Write TX payload
#define NRF_FLUSH_TX               0xE1  // Flush TX FIFO
#define NRF_FLUSH_RX               0xE2  // Flush RX FIFO
#define NRF_REUSE_TX_PL            0xE3  // Reuse last transmitted payload
#define NRF_R_RX_PL_WID            0x60  // Read RX payload width
#define NRF_W_ACK_PAYLOAD          0xA8  // Write payload for ACK packet (A8–AF)
#define NRF_W_TX_PAYLOAD_NOACK     0xB0  // Write payload, no ACK requested
#define NRF_NOP                    0xFF  // No operation, returns STATUS register

/* ============================================================
 *  nRF24L01+ Register Map
 * ============================================================ */

#define NRF_REG_CONFIG       0x00
#define NRF_REG_EN_AA        0x01
#define NRF_REG_EN_RXADDR    0x02
#define NRF_REG_SETUP_AW     0x03
#define NRF_REG_SETUP_RETR   0x04
#define NRF_REG_RF_CH        0x05
#define NRF_REG_RF_SETUP     0x06
#define NRF_REG_STATUS       0x07
#define NRF_REG_OBSERVE_TX   0x08
#define NRF_REG_CD           0x09
#define NRF_REG_RX_ADDR_P0   0x0A
#define NRF_REG_RX_ADDR_P1   0x0B
#define NRF_REG_RX_ADDR_P2   0x0C
#define NRF_REG_RX_ADDR_P3   0x0D
#define NRF_REG_RX_ADDR_P4   0x0E
#define NRF_REG_RX_ADDR_P5   0x0F
#define NRF_REG_TX_ADDR      0x10
#define NRF_REG_RX_PW_P0     0x11
#define NRF_REG_RX_PW_P1     0x12
#define NRF_REG_RX_PW_P2     0x13
#define NRF_REG_RX_PW_P3     0x14
#define NRF_REG_RX_PW_P4     0x15
#define NRF_REG_RX_PW_P5     0x16
#define NRF_REG_FIFO_STATUS  0x17
#define NRF_REG_DYNPD        0x1C
#define NRF_REG_FEATURE      0x1D

/* ============================================================
 *  Bit Definitions per Register
 * ============================================================ */

/* CONFIG Register (0x00) */
#define NRF_MASK_RX_DR   6
#define NRF_MASK_TX_DS   5
#define NRF_MASK_MAX_RT  4
#define NRF_EN_CRC       3
#define NRF_CRC0         2
#define NRF_PWR_UP       1
#define NRF_PRIM_RX      0

/* EN_AA Register (0x01) */
#define NRF_ENAA_P5      5
#define NRF_ENAA_P4      4
#define NRF_ENAA_P3      3
#define NRF_ENAA_P2      2
#define NRF_ENAA_P1      1
#define NRF_ENAA_P0      0

/* EN_RXADDR Register (0x02) */
#define NRF_ERX_P5       5
#define NRF_ERX_P4       4
#define NRF_ERX_P3       3
#define NRF_ERX_P2       2
#define NRF_ERX_P1       1
#define NRF_ERX_P0       0

/* SETUP_AW Register (0x03) */
#define AW0              0
#define AW1              1
// Address widths: 01=3 bytes, 10=4 bytes, 11=5 bytes

/* SETUP_RETR Register (0x04) */
#define ARD0             0   // Auto retransmit delay bits
#define ARD1             1
#define ARD2             2
#define ARD3             3
#define ARC0             4   // Auto retransmit count bits
#define ARC1             5
#define ARC2             6
#define ARC3             7

/* RF_CH Register (0x05) */
#define RF_CH0           0
#define RF_CH1           1
#define RF_CH2           2
#define RF_CH3           3
#define RF_CH4           4
#define RF_CH5           5
#define RF_CH6           6
#define RF_CH7           7

/* RF_SETUP Register (0x06) */
#define RF_CONT_WAVE     7
#define RF_DR_LOW        5
#define RF_PLL_LOCK      4
#define RF_DR_HIGH       3
#define RF_PWR1          2
#define RF_PWR0          1
// Bits 2:1 control TX power
// Bits 5 and 3 control data rate

/* STATUS Register (0x07) */
#define NRF_RX_DR        6
#define NRF_TX_DS        5
#define NRF_MAX_RT       4
#define NRF_RX_P_NO2     3
#define NRF_RX_P_NO1     2
#define NRF_RX_P_NO0     1
#define NRF_TX_FULL      0

/* FIFO_STATUS Register (0x17) */
#define NRF_TX_REUSE     6
#define NRF_FIFO_TX_FULL 5
#define NRF_TX_EMPTY     4
#define NRF_RX_FULL      1
#define NRF_RX_EMPTY     0

/* DYNPD Register (0x1C) */
#define NRF_DPL_P5       5
#define NRF_DPL_P4       4
#define NRF_DPL_P3       3
#define NRF_DPL_P2       2
#define NRF_DPL_P1       1
#define NRF_DPL_P0       0

/* FEATURE Register (0x1D) */
#define NRF_EN_DPL       2
#define NRF_EN_ACK_PAY   1
#define NRF_EN_DYN_ACK   0

/* ============================================================
 *  RF Power and Data Rate Settings
 * ============================================================ */

// Power levels (bits 2:1)
#define NRF_PWR_NEG18DBM  0x00
#define NRF_PWR_NEG12DBM  (1 << RF_PWR0)
#define NRF_PWR_NEG6DBM   (1 << RF_PWR1)
#define NRF_PWR_0DBM      ((1 << RF_PWR0) | (1 << RF_PWR1))

// Data rate settings
#define NRF_DR_1MBPS      (0)
#define NRF_DR_2MBPS      (1 << RF_DR_HIGH)
#define NRF_DR_250KBPS    (1 << RF_DR_LOW)

/* ============================================================
 *  Helper Macros
 * ============================================================ */

#define NRF_RF_CH_VAL(ch)   ((ch) & 0x7F)     // 0–127 → 2400–2527 MHz
#define NRF_DUMMY_BYTE 0xFF
#define NRF_CONFIG_TX  ((1 << NRF_PWR_UP) | (0 << NRF_PRIM_RX))  
#define NRF_CONFIG_RX  ((1 << NRF_PWR_UP) | (1 << NRF_PRIM_RX)) 

#endif /* NRF24L01_H */
