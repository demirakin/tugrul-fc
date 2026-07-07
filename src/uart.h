/*
 * TUGRUL flight controller — USART2 console
 *   M1: polled bring-up.  M5-prep: interrupt-driven, non-blocking TX.
 *
 * USART2 TX/RX (PA2/PA3) are hard-wired to the Nucleo-F446RE on-board ST-LINK,
 * which bridges them to a USB CDC virtual serial port — a host terminal
 * (Tera Term @ 115200 8N1) sees this UART with no extra cabling.
 *
 * TX PATH (M5-prep): non-blocking. Bytes are pushed into a 512-byte software
 * ring buffer and drained one-per-interrupt by USART2_IRQHandler on TXE. The
 * old blocking spin-on-TXE stalled the 1 kHz control loop for milliseconds per
 * heartbeat — that jitter source is now removed. When the ring is full the byte
 * is DROPPED and counted (uart_get_dropped()); blocking is forbidden (it would
 * defeat the purpose) and silent loss is forbidden (honesty rule).
 *
 * RX PATH: still polled / non-blocking (uart_read_byte_nonblock); RXNEIE is
 * never enabled. The main loop is never stalled waiting for a key.
 *
 * uart_init() MUST be called only after clock_init() succeeded — the baud
 * divisor is computed from the fixed 42 MHz PCLK1 that clock.c establishes.
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>

/* Bring USART2 up at 115200 8N1 on PA2/PA3. Call after clock_init() == 0. */
void uart_init(void);

/* Enqueue one raw byte into the TX ring (never blocks). If the ring is full the
 * byte is dropped and the drop counter is incremented — see uart_get_dropped(). */
void uart_write_byte(uint8_t b);

/* Send a NUL-terminated string; a lone '\n' is expanded to "\r\n". */
void uart_write_str(const char *s);

/* Print one byte as two uppercase hex digits, no prefix (e.g. "71"). */
void uart_write_hex8(uint8_t v);

/* Print a 32-bit value as eight uppercase hex digits, no prefix. */
void uart_write_hex32(uint32_t v);

/* Print an unsigned value in decimal, no leading zeros. */
void uart_write_dec(uint32_t v);

/* Non-blocking receive: returns 0..255 if a byte is waiting, else -1. */
int uart_read_byte_nonblock(void);

/* Total bytes dropped because the TX ring was full since boot. The heartbeat
 * prints this once per second only when nonzero (honest loss reporting). */
uint32_t uart_get_dropped(void);

#endif /* UART_H */
