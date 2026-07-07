/*
 * TUGRUL flight controller — Milestone M2-prep (SPI1 master driver)
 *
 * Register-level SPI1 master over PB3/PB4/PB5 (AF5) with a manual GPIO chip
 * select on PB6. Built for the MPU-9250 IMU, but the API is a plain byte-level
 * full-duplex transfer plus CS framing, so it stays a generic SPI master.
 *
 * Pin map (Nucleo-F446RE) — see spi.c header for the full rationale:
 *   PB3 = SPI1_SCK  (AF5)
 *   PB4 = SPI1_MISO (AF5)
 *   PB5 = SPI1_MOSI (AF5)
 *   PB6 = CS/NSS    (plain GPIO output, driven by hand per transaction)
 *
 * Two bus speeds are exposed on purpose (both derived from APB2 = 84 MHz):
 *   SPI_SPEED_CONFIG  656.25 kHz  (<= 1 MHz  — MPU-9250 register access limit)
 *   SPI_SPEED_DATA    10.5   MHz  (<= 20 MHz — MPU-9250 sensor-register read
 *                                  limit; DATA is plumbed for M3, unused at M2)
 *
 * spi1_init() MUST be called only after clock_init() succeeded — the prescaler
 * math assumes the fixed 84 MHz APB2 (PCLK2) that clock.c establishes.
 */

#ifndef SPI_H
#define SPI_H

#include <stdint.h>

/*
 * Bus-speed selector. The two enumerators map to concrete SPI_CR1 BR[2:0]
 * prescaler encodings inside spi.c (see SPI_BR_* there). Kept as an enum so the
 * caller states intent (config vs data) instead of a raw prescaler code.
 */
typedef enum {
    SPI_SPEED_CONFIG = 0,   /* /128 -> 656.25 kHz : register writes + WHO_AM_I */
    SPI_SPEED_DATA   = 1    /* /8   -> 10.5   MHz : bulk sensor-register reads  */
} spi_speed_t;

/* Bring SPI1 up: GPIO AF setup + CS GPIO + SPI1 at CONFIG speed, SPE on. */
void spi1_init(void);

/* Change the bus prescaler. Disables SPE, edits BR, re-enables (see spi.c). */
void spi1_set_speed(spi_speed_t s);

/* Blocking full-duplex 8-bit exchange: clock 'tx' out, return the byte clocked
 * in. Bounded timeout — returns 0xFF if a status flag never asserts. */
uint8_t spi1_transfer(uint8_t tx);

/* Drive the manual chip select. Low = slave selected, High = deselected. */
void spi1_cs_low(void);
void spi1_cs_high(void);

#endif /* SPI_H */
