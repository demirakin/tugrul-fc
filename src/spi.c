/*
 * TUGRUL flight controller — Milestone M2-prep (SPI1 master driver)
 * Register-level SPI1, no HAL.
 *
 * Pin map (Nucleo-F446RE) and WHY these pins:
 *   PB3 = SPI1_SCK  (AF5)
 *   PB4 = SPI1_MISO (AF5)
 *   PB5 = SPI1_MOSI (AF5)
 *   PB6 = CS/NSS    (plain GPIO output, manual framing)
 *
 * SPI1 also has a default mapping on PA5/PA6/PA7. We do NOT use it: SPI1_SCK on
 * PA5 collides with LD2, the user LED we have been blinking since M0. The PB3/
 * PB4/PB5 AF5 alternate mapping (F446RE datasheet "Alternate function mapping")
 * keeps the LED free. PB3 doubles as SWO/JTDO on the debug port, but we debug
 * over SWD (SWCLK/SWDIO) without SWO trace, so PB3 is free to repurpose as SCK.
 *
 * CS is a manual GPIO output, NOT the SPI peripheral's hardware NSS. The
 * MPU-9250 requires CS to frame each transaction (fall before the first clock,
 * rise after the last), and hardware NSS output on the F4 in master mode is
 * awkward — it de-asserts based on SPE, not per byte/transaction, so getting
 * clean per-transaction framing out of it fights the peripheral. Toggling a
 * plain GPIO around each transaction is simpler and unambiguous, at the cost of
 * two register writes per frame (a non-issue at our rates).
 *
 * Clocking: SPI1 is on APB2. clock.c fixes PCLK2 at 84 MHz (APB2 /1). The two
 * prescaler encodings below are derived from that constant; if the clock tree
 * changes, revisit them.
 */

#include "spi.h"
#include "clock.h"          /* documents the 84 MHz APB2 this driver assumes */
#include "stm32f446xx.h"

/*
 * fPCLK2 feeding SPI1. MUST match clock.c: SYSCLK 84 MHz with APB2 /1 gives
 * PCLK2 = 84 MHz. Hardcoded on purpose — spi1_init() only runs after
 * clock_init() succeeds, so this is a known constant, not a guess.
 */
#define SPI_PCLK2_HZ    84000000u

/*
 * BR[2:0] prescaler encodings (RM0390 §27.5.1, SPI_CR1): 000=/2, 001=/4,
 * 010=/8, 011=/16, 100=/32, 101=/64, 110=/128, 111=/256.
 *
 *   CONFIG: /128 -> 84 MHz / 128 = 656.25 kHz  (<= 1 MHz MPU-9250 config limit)
 *   DATA  : /8   -> 84 MHz / 8   = 10.5   MHz   (<= 20 MHz MPU-9250 read limit)
 *
 * Why /8 and not /4 for DATA: /4 = 84/4 = 21 MHz, which is just OVER the
 * MPU-9250's 20 MHz sensor-register read ceiling — so the next-slower /8 is the
 * fastest legal step. M2 only ever uses CONFIG; DATA is plumbed now so M3's
 * bulk reads cannot regress into a single-speed driver.
 */
#define SPI_BR_CONFIG   (SPI_CR1_BR_2 | SPI_CR1_BR_1)   /* 0b110 = /128 */
#define SPI_BR_DATA     (SPI_CR1_BR_1)                  /* 0b010 = /8   */

/*
 * Base CR1 configuration shared by both speeds (BR filled in separately):
 *   CPOL=1, CPHA=1  -> SPI mode 3 (clock idles high, sample on 2nd edge). The
 *                      MPU-9250 supports modes 0 and 3; mode 3 (idle-high clock)
 *                      is the conventional choice for this part.
 *   MSTR=1          -> master.
 *   SSM=1, SSI=1    -> software slave management: NSS is driven internally high
 *                      so the peripheral never sees itself deselected. Without
 *                      SSI=1 in master+SSM mode, NSS reads low and the hardware
 *                      raises a mode fault (MODF) and drops MSTR. We manage the
 *                      real chip select by hand on PB6.
 *   LSBFIRST=0      -> MSB first (MPU-9250 is MSB-first).
 *   DFF=0           -> 8-bit frames.
 *   SPE set later   -> peripheral enabled only after the whole config lands.
 */
#define SPI_CR1_BASE    (SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_MSTR \
                        | SPI_CR1_SSM | SPI_CR1_SSI)

/*
 * Bounded spin for every SPI status wait. At the slowest bus speed (CONFIG,
 * 656.25 kHz) one 8-bit frame is ~12 us; TXE/RXNE/BSY transitions are all
 * sub-frame. This count is many milliseconds of head-room at 84 MHz CPU, so a
 * loop that never exits means a genuine bus fault (no clock, no slave, shorted
 * MISO) — we bail with a sentinel instead of hanging the flight loop forever.
 */
#define SPI_TIMEOUT     100000u

void spi1_init(void)
{
    /* 1) Peripheral clocks. GPIOB is not used elsewhere yet, but |= is
     *    idempotent so enabling it twice would be harmless anyway. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    /* RCC_AHB1ENR bit1:  GPIOB clock enable */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;     /* RCC_APB2ENR bit12: SPI1 clock enable  */

    /* Read-back barriers: on STM32F4 a clock-enable write can lag the bus by a
     * couple of cycles. Reading the register back stalls until the enable is
     * live, so the GPIO/SPI config below cannot land on unclocked silicon and
     * be dropped. Per the F4 "delay after RCC enable" erratum workaround, each
     * ENR that was written gets its own read-back — AHB1ENR guards the GPIOB
     * config, APB2ENR guards the SPI1 config. (Same rationale as the RCC
     * read-backs in main.c / uart.c.) */
    volatile uint32_t ahb1enr_readback = RCC->AHB1ENR;
    (void)ahb1enr_readback;
    volatile uint32_t apb2enr_readback = RCC->APB2ENR;
    (void)apb2enr_readback;

    /* 2) PB3/PB4/PB5 -> alternate-function mode for SPI1.
     *    MODER is 2 bits/pin: 00=input, 01=output, 10=alt-fn, 11=analog.
     *    Clear all three fields, then set bit1 of each -> 0b10 (alternate). */
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->MODER |=  (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);

    /* 2b) PB6 -> general-purpose output for the manual chip select (0b01). */
    GPIOB->MODER &= ~GPIO_MODER_MODER6;
    GPIOB->MODER |=  GPIO_MODER_MODER6_0;   /* PB6 = 01 output (CS) */

    /* 3) Select AF5 (SPI1/2) on PB3/PB4/PB5. AFRL holds 4 bits/pin for pins
     *    0..7. AF5 = 0b0101 = bits 0 and 2 of each 4-bit field. Clear then set. */
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3 | GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5);
    GPIOB->AFR[0] |=  (GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_2)   /* PB3 SCK  = AF5 */
                  |   (GPIO_AFRL_AFSEL4_0 | GPIO_AFRL_AFSEL4_2)   /* PB4 MISO = AF5 */
                  |   (GPIO_AFRL_AFSEL5_0 | GPIO_AFRL_AFSEL5_2);  /* PB5 MOSI = AF5 */

    /* 4) Output speed on the driven pins (SCK, MOSI, CS). At 10.5 MHz the SCK
     *    edges are fast enough to want a crisp slew, so use "high speed" (0b11)
     *    to keep edges clean and avoid rounding the clock at DATA speed. MISO is
     *    an input — OSPEEDR is irrelevant for it, so it is left untouched. */
    GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED5
                    |  GPIO_OSPEEDR_OSPEED6);   /* PB3/PB5/PB6 = 11 high speed */

    /* 5) Pull resistors.
     *    SCK (PB3): CPOL=1 idles the clock HIGH. Pull-up so the line parks high
     *      while SPE is off / between transactions, matching the idle level and
     *      preventing a spurious edge when the peripheral takes the pin.
     *    MISO (PB4): pull-UP. With no sensor wired (M2-prep reality) MISO floats;
     *      a defined pull keeps it from drifting mid-rail. See mpu9250.c for how
     *      a floating/stuck line reads back as 0x00 or 0xFF ("no response").
     *    MOSI (PB5): actively driven by SPI1, no pull needed -> 00.
     *    CS (PB6): driven by our GPIO, no pull needed -> 00 (we set its level
     *      explicitly high in step 6 before any transaction). */
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4
                    | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6);
    GPIOB->PUPDR |=  (GPIO_PUPDR_PUPD3_0 | GPIO_PUPDR_PUPD4_0);  /* PB3/PB4 = 01 pull-up */

    /* 6) Park CS high (deselected) BEFORE enabling the peripheral, so no slave
     *    is ever selected during bus bring-up. */
    spi1_cs_high();

    /* 7) SPI1 configuration. Program CR1 with SPE still 0 — several CR1 fields
     *    (BR, CPOL/CPHA, MSTR) must not change while the SPI is enabled
     *    (RM0390 §27.5.1). Start at CONFIG speed; M2 register access needs the
     *    slow bus, and DATA speed is switched in on demand for M3. */
    SPI1->CR1 = SPI_CR1_BASE | SPI_BR_CONFIG;   /* mode3, master, sw-NSS, MSB, 8-bit, /128 */
    SPI1->CR2 = 0u;                             /* no interrupts, no DMA, no hw SSOE */

    /* 8) Enable the peripheral last, config now complete. */
    SPI1->CR1 |= SPI_CR1_SPE;   /* CR1 bit6: SPI enable */
}

void spi1_set_speed(spi_speed_t s)
{
    /*
     * BR[2:0] must not be changed while the SPI is enabled and communicating
     * (RM0390 §27.5.1: the control bits are latched at SPE and a live change can
     * corrupt an in-flight frame). So: wait for the bus to go idle, drop SPE,
     * edit BR, then re-enable. We only ever call this between transactions (CS
     * high), so BSY should already be clear — the wait is a cheap guard.
     */
    volatile uint32_t timeout = SPI_TIMEOUT;
    while (((SPI1->SR & SPI_SR_BSY) != 0u) && (timeout != 0u)) {
        timeout--;              /* SR bit7: BSY — set while a transfer is on the wire */
    }

    SPI1->CR1 &= ~SPI_CR1_SPE;  /* disable before touching BR (control bits latch at SPE) */

    SPI1->CR1 &= ~SPI_CR1_BR_Msk;   /* clear current prescaler field */
    switch (s) {
        case SPI_SPEED_DATA:
            SPI1->CR1 |= SPI_BR_DATA;     /* /8   -> 10.5 MHz */
            break;
        case SPI_SPEED_CONFIG:
        default:
            SPI1->CR1 |= SPI_BR_CONFIG;   /* /128 -> 656.25 kHz */
            break;
    }

    SPI1->CR1 |= SPI_CR1_SPE;   /* re-enable with the new prescaler in effect */
}

uint8_t spi1_transfer(uint8_t tx)
{
    /*
     * Full-duplex single-byte exchange. On SPI every bit shifted OUT clocks a
     * bit IN, so one write produces exactly one received byte. The precise dance
     * (this is where SPI drivers rot):
     *   1) wait TXE=1  -> the TX FIFO/register can accept a byte,
     *   2) write DR    -> hardware starts clocking the 8 bits out (and in),
     *   3) wait RXNE=1 -> the incoming byte has fully shifted in,
     *   4) read DR     -> returns the received byte AND clears RXNE, keeping the
     *                     RX side clean so the NEXT transfer's RXNE is meaningful.
     * Skipping the RXNE read would leave RXNE stuck set and eventually overrun.
     * All hardware waits are bounded; on timeout we surface 0xFF, which is also
     * the "no MISO" pattern, so a dead bus and an absent slave both read as an
     * honest non-answer rather than a hang.
     */
    volatile uint32_t timeout;

    /* 1) Wait for the transmit buffer to be empty. */
    timeout = SPI_TIMEOUT;
    while (((SPI1->SR & SPI_SR_TXE) == 0u) && (timeout != 0u)) {
        timeout--;              /* SR bit1: TXE — TX buffer empty */
    }
    if (timeout == 0u) {
        return 0xFFu;           /* bus stuck: report non-answer, do not hang */
    }

    /* 2) Write the byte. DR is accessed 8-bit here so a single byte (not two)
     *    is loaded into the 8-bit-frame data register. */
    *(volatile uint8_t *)&SPI1->DR = tx;    /* DR: load TX byte -> starts the exchange */

    /* 3) Wait for the received byte to arrive. */
    timeout = SPI_TIMEOUT;
    while (((SPI1->SR & SPI_SR_RXNE) == 0u) && (timeout != 0u)) {
        timeout--;              /* SR bit0: RXNE — RX buffer not empty */
    }
    if (timeout == 0u) {
        return 0xFFu;           /* clocked out but nothing came back: non-answer */
    }

    /* 4) Read DR: returns the received byte and clears RXNE. 8-bit read to match
     *    the 8-bit frame. */
    return *(volatile uint8_t *)&SPI1->DR;  /* DR: received byte, clears RXNE */
}

void spi1_cs_low(void)
{
    /* Assert CS: drive PB6 low to select the slave. BSRR bit-reset is atomic —
     * no read-modify-write, so it cannot race the other PB pins. */
    GPIOB->BSRR = GPIO_BSRR_BR6;    /* BSRR BR6: reset PB6 -> CS low (selected) */
}

void spi1_cs_high(void)
{
    /* De-assert CS: drive PB6 high to deselect. Atomic bit-set via BSRR. */
    GPIOB->BSRR = GPIO_BSRR_BS6;    /* BSRR BS6: set PB6 -> CS high (deselected) */
}
