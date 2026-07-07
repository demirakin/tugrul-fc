/*
 * TUGRUL flight controller — USART2 console
 *   M1: polled bring-up.  M5-prep: interrupt-driven, non-blocking TX ring.
 * Register-level, no HAL.
 *
 * Pin map (Nucleo-F446RE, fixed by the board wiring):
 *   PA2 = USART2_TX  -> ST-LINK VCP RX
 *   PA3 = USART2_RX  <- ST-LINK VCP TX
 * Both pins use alternate function AF7 (USART1/2/3) per the F446RE datasheet
 * "Alternate function mapping" table.
 *
 * Clocking: USART2 sits on APB1. clock.c fixes PCLK1 at 42 MHz (SYSCLK 84 MHz,
 * APB1 prescaler /2). The baud divisor below is derived from that constant; if
 * the clock tree ever changes, this file must be revisited.
 */

#include "uart.h"
#include "clock.h"          /* documents the clock tree this driver depends on */
#include "stm32f446xx.h"

/*
 * PCLK1 feeding USART2. This MUST match clock.c: SYSCLK 84 MHz with APB1 /2
 * gives PCLK1 = 42 MHz. Hardcoded on purpose — uart_init() is only ever called
 * after clock_init() succeeds, so this is a known constant, not a guess.
 */
#define UART_PCLK1_HZ   42000000u

/* Target line rate for the ST-LINK VCP terminal. */
#define UART_BAUD       115200u

/*
 * BRR for 115200 baud from 42 MHz PCLK1 with OVER8=0 (oversampling by 16).
 *
 *   USARTDIV = PCLK1 / (16 * BAUD) = 42e6 / (16 * 115200) = 22.7864...
 *   mantissa = floor(22.7864)                = 22
 *   fraction = round(0.7864 * 16) = round(12.58) = 13   (fits 4 bits, no carry)
 *   BRR      = (mantissa << 4) | fraction = (22 << 4) | 13 = 0x160 | 0xD = 0x16D
 *
 * Actual divisor  = 22 + 13/16 = 22.8125
 * Actual baud     = 42e6 / (16 * 22.8125) = 115068.5 baud
 * Error           = (115068.5 - 115200) / 115200 = -0.114%
 * Well inside the ~±2-3% a UART receiver tolerates (RM0390 §24: with 16x
 * oversampling, OVER8=0, the matching window is comfortably wider than
 * 0.11%), so 8N1 framing is safe.
 */
#define UART_BRR_VALUE  0x16Du   /* = 365 decimal; see arithmetic above */

/* ==========================================================================
 * Non-blocking TX: single-producer / single-consumer (SPSC) ring buffer.
 *
 * Producer = thread-mode code (main loop, via uart_write_byte). Consumer =
 * USART2_IRQHandler on TXE. The two never run at the same time on the same
 * byte: the ISR preempts the producer, drains one byte, returns.
 *
 * Index discipline (lock-free, NO critical section for the indices):
 *   - s_tx_head is written ONLY by the producer; s_tx_tail ONLY by the ISR.
 *   - Both are single 16-bit words, naturally aligned -> a store is a single
 *     STRH that is atomic on Cortex-M4 (no tearing), and an exception is taken
 *     only on an instruction boundary, so the ISR always sees a fully-written
 *     head and the producer a fully-written tail. Hence no lock is needed.
 *   - They are FREE-RUNNING counters (0..65535), masked only when indexing the
 *     array. 65536 is an exact multiple of the 512 size, so head & MASK stays
 *     consistent across the 16-bit wrap. Fill level = (uint16_t)(head - tail)
 *     is wraparound-safe unsigned subtraction, range 0..512; empty when 0, full
 *     when 512. This uses all 512 bytes (no reserved slot).
 *
 * Buffer sizing: the 1 Hz heartbeat's worst-case burst is ~200 bytes (probe/raw/
 * scaled/attitude lines). At 115200 8N1 one byte = 10 bits / 115200 = 86.8 us,
 * so ~200 bytes drain in ~17 ms and a completely full 512-byte ring drains in
 * ~44 ms — both far inside the 1 s heartbeat period, so the ring never backs up
 * under normal operation. 512 is a power of two -> masking, never modulo.
 *
 * Full-buffer policy: DROP the byte and count it (s_tx_dropped). Blocking is
 * forbidden here — it would re-introduce exactly the control-loop stall this
 * change removes. Silent loss is forbidden — the heartbeat surfaces the count.
 */
#define UART_TXBUF_SIZE  512u
#define UART_TXBUF_MASK  (UART_TXBUF_SIZE - 1u)   /* 0x1FF; size is a power of two */

/* Data bytes AND head/tail are volatile so the compiler keeps program order
 * among them: the producer's byte store is emitted before the head publish, and
 * the ISR's byte load after the head/tail compare. That ordering is what makes
 * the barrier-free SPSC scheme safe on this single core. */
static volatile uint8_t  s_txbuf[UART_TXBUF_SIZE];
static volatile uint16_t s_tx_head = 0u;   /* next write slot (producer owns) */
static volatile uint16_t s_tx_tail = 0u;   /* next read slot  (ISR owns)      */

/* Dropped-byte counter. Written ONLY by the producer (uart_write_byte), read by
 * uart_get_dropped(); the ISR never touches it, so no sharing hazard. */
static volatile uint32_t s_tx_dropped = 0u;

void uart_init(void)
{
    /* 1) Peripheral clocks. GPIOA may already be enabled by main.c's LED setup;
     *    |= is idempotent — re-setting an already-set enable bit is harmless. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    /* RCC_AHB1ENR bit0:  GPIOA clock enable */
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;   /* RCC_APB1ENR bit17: USART2 clock enable */

    /* Read-back barrier: on STM32F4 a peripheral-clock enable can lag a couple
     * of cycles behind the write. Reading the register back stalls until the
     * enable is live, so the GPIO/USART config below cannot land on unclocked
     * silicon and get dropped. (Same rationale as the GPIOA read-back in main.c.) */
    volatile uint32_t apb1enr_readback = RCC->APB1ENR;
    (void)apb1enr_readback;

    /* 2) PA2/PA3 -> alternate function mode.
     *    MODER is 2 bits/pin: 00=input, 01=output, 10=alt-fn, 11=analog.
     *    Clear both fields first, then set bit1 of each -> 10 (alternate). */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |=  (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1); /* both = 0b10 alt-fn */

    /* 3) Select AF7 (USART2) on PA2/PA3. AFRL holds 4 bits/pin for pins 0..7.
     *    AF7 = 0b0111 = bits 0+1+2 of each 4-bit field. Clear then set. */
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
    GPIOA->AFR[0] |=  (GPIO_AFRL_AFSEL2_0 | GPIO_AFRL_AFSEL2_1 | GPIO_AFRL_AFSEL2_2)  /* PA2 = AF7 */
                  |   (GPIO_AFRL_AFSEL3_0 | GPIO_AFRL_AFSEL3_1 | GPIO_AFRL_AFSEL3_2); /* PA3 = AF7 */

    /* 4) Output speed on TX (PA2). At 115200 baud the edges are glacial; the
     *    default low-speed slew is plenty and produces the cleanest edges / least
     *    EMI. Leave PA3 (RX, an input) untouched — OSPEEDR is irrelevant for it. */
    GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEED2;   /* PA2 speed = 00 (low speed) */

    /* 5) Pull resistors.
     *    TX (PA2): actively driven by the USART, no pull needed -> 00. Also,
     *    before UE is set the pin floats briefly; but the ST-LINK VCP idles the
     *    line high itself, so no idle-glitch concern on TX.
     *    RX (PA3): enable a pull-UP -> 01. If the ST-LINK VCP ever tri-states
     *    its TX (unpowered/re-enumerating), a floating RX line could be read as
     *    random start bits and wedge the receiver in framing/overrun errors.
     *    A pull-up parks the idle level at logic '1' (UART idle = high), which is
     *    the safe, framing-correct rest state. */
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);
    GPIOA->PUPDR |=  GPIO_PUPDR_PUPD3_0;       /* PA3 (RX) = 01 pull-up; PA2 (TX) = 00 no pull */

    /* 6) USART2 configuration. Program everything with UE (enable) still 0, then
     *    turn the peripheral on last — several CR fields (e.g. baud, oversampling)
     *    must not change while the USART is enabled (RM0390 §24.6.4).
     *
     *    Start from a clean CR1: OVER8 must be 0 (oversampling by 16) for the BRR
     *    arithmetic above, and word length (M=0 -> 8 data bits) / parity (PCE=0 ->
     *    none) reset defaults give us 8N1. CR2 stop bits default to 1. */
    USART2->CR1 = 0u;                       /* OVER8=0 (x16), M=0 (8-bit), parity off -> 8N1 */
    USART2->CR2 = 0u;                       /* STOP=00 -> 1 stop bit */
    USART2->CR3 = 0u;                       /* no flow control, no DMA */
    USART2->BRR = UART_BRR_VALUE;           /* 115200 baud from 42 MHz PCLK1 (see #define) */

    /* Enable transmitter + receiver, then the USART as a whole. TE/RE set in the
     * same write that sets UE is fine on F4; UE is the master enable and takes
     * effect for the whole register state at once. */
    USART2->CR1 |= USART_CR1_TE            /* CR1 bit3:  transmitter enable */
                |  USART_CR1_RE            /* CR1 bit2:  receiver enable */
                |  USART_CR1_UE;           /* CR1 bit13: USART enable (config done -> turn on) */
    /* NOTE: TXEIE is left OFF here. It is turned on only when the first byte is
     * enqueued (uart_write_byte) and turned off by the ISR when the ring drains,
     * so an empty console generates no interrupt traffic. RXNEIE is never enabled
     * (RX stays polled). We deliberately never wait on TC anywhere — TXE alone
     * gates feeding the shift register (RM0390 §24.6.1). */

    /* ----------------------------------------------------------------------
     * Interrupt priorities — FIRST explicit priority decision in TUGRUL.
     * STM32F4 implements 4 priority bits => 16 levels, 0 = highest urgency. We
     * keep the reset default grouping (all 4 bits are pre-emption priority, no
     * sub-priority), so these numbers are pure pre-emption levels.
     *
     *   SysTick = 0 (highest): it IS the millisecond time base. A tick delayed
     *     behind a lower IRQ becomes millis() jitter and corrupts every cadence
     *     built on it (control tick, heartbeat, delay_ms). It is already 0 at
     *     reset; we set it explicitly to make the decision visible in code.
     *   USART2  = 2: strictly below the timebase (a late console byte is
     *     harmless), and level 1 is left free for a future flight-critical IRQ
     *     (e.g. an MPU data-ready line) that must sit above the console yet
     *     below the timebase.
     * SysTick is a core exception (negative IRQn); NVIC_SetPriority routes it to
     * the SCB SHP registers automatically. USART2 is external IRQ #38.
     * ---------------------------------------------------------------------- */
    NVIC_SetPriority(SysTick_IRQn, 0u);   /* timebase = most urgent (explicit, though 0 at reset) */
    NVIC_SetPriority(USART2_IRQn, 2u);    /* console below timebase; leaves level 1 for future flight-critical IRQ */
    NVIC_EnableIRQ(USART2_IRQn);          /* unmask USART2 in NVIC; TXEIE still off until first enqueue */
}

void uart_write_byte(uint8_t b)
{
    /* Single enqueue point for the whole driver (str/hex/dec all build on this).
     * Never blocks: push into the ring, or drop-and-count if it is full. */
    uint16_t head = s_tx_head;
    uint16_t fill = (uint16_t)(head - s_tx_tail);   /* 0..512, wraparound-safe */

    if (fill >= UART_TXBUF_SIZE) {
        /* Ring full: drop the byte and count it. Reading a slightly stale tail
         * here can only OVER-estimate fill (the ISR may have freed a slot since),
         * so this is conservative — it never overruns, at worst drops one byte
         * that would just have fit. Blocking is forbidden (see policy note). */
        s_tx_dropped++;
        return;
    }

    s_txbuf[head & UART_TXBUF_MASK] = b;   /* write data slot first ... */
    s_tx_head = (uint16_t)(head + 1u);     /* ... then publish it to the ISR */

    /* Kick the drain. TXEIE is a read-modify-write shared with the ISR (which
     * clears it when the ring empties); the enqueue-THEN-set ordering makes this
     * safe with no critical section — see the interleaving analysis on the ISR. */
    USART2->CR1 |= USART_CR1_TXEIE;        /* CR1 bit7: enable TXE interrupt */
}

/*
 * USART2 TX interrupt — the ring consumer. Kept constant-short: at most one DR
 * write or one CR1 clear per entry, no loops.
 *
 * TXEIE race analysis (why no critical section is needed)
 * -------------------------------------------------------
 * Producer (thread mode):  P1: store byte, publish head++   P2: CR1 |= TXEIE
 * ISR (preempts producer): A: if tail!=head -> send byte, tail++
 *                          B: else            -> CR1 &= ~TXEIE
 *
 * The ISR is atomic w.r.t. the producer (thread code cannot preempt the ISR), so
 * the only interleaving is "producer interrupted by ISR". Two hazards:
 *
 * 1) Lost byte?  Impossible. The producer ALWAYS makes the byte visible (P1)
 *    BEFORE enabling TXEIE (P2). So whenever TXEIE is (re)set, the byte is
 *    already in the ring. The ISR clears TXEIE only after it observes empty
 *    (path B), i.e. after every enqueued byte has been sent. Therefore a byte is
 *    either sent by a TXE interrupt already in flight, or picked up by the one
 *    that P2 arms — it can never be stranded.
 *
 * 2) Torn CR1 RMW?  Harmless. If the ISR fires between the LDR and STR of the
 *    producer's "CR1 |= TXEIE" and takes path B (clears TXEIE), the producer's
 *    STR writes back its snapshot with TXEIE SET — re-setting it. That is the
 *    correct outcome, because the producer just enqueued a byte (P1 done), so we
 *    WANT TXEIE on. No other CR1 bit (TE/RE/UE) is modified at run time, so none
 *    is lost either way. Conversely the ISR's own "CR1 &= ~TXEIE" is never torn,
 *    since the producer cannot preempt it.
 *
 * Worst case: the ISR drains the just-enqueued byte to empty (path A then, on a
 * later interrupt, would clear TXEIE) between P1 and P2, and P2 then re-sets
 * TXEIE on an already-empty ring. Result = ONE spurious TXE interrupt that finds
 * the ring empty and clears TXEIE (path B). One wasted interrupt, zero data loss.
 */
void USART2_IRQHandler(void)
{
    uint32_t sr  = USART2->SR;    /* SR bit7 = TXE (TX data register empty) */
    uint32_t cr1 = USART2->CR1;   /* CR1 bit7 = TXEIE                        */

    /* Handle ONLY TXE, and only while it is our enabled source. We never enable
     * RXNEIE/TCIE, so no other USART2 interrupt reason reaches this handler. */
    if (((sr & USART_SR_TXE) != 0u) && ((cr1 & USART_CR1_TXEIE) != 0u)) {
        uint16_t tail = s_tx_tail;
        if (tail != s_tx_head) {
            /* Ring non-empty: feed one byte. Writing DR clears TXE (RM0390
             * §24.6.1) — we do NOT wait on TC. One byte per interrupt: at
             * 115200 that is ~87 us of shift time per ~sub-us of ISR, so the
             * per-byte interrupt overhead is negligible and the ISR stays short. */
            USART2->DR = (uint32_t)s_txbuf[tail & UART_TXBUF_MASK];
            s_tx_tail  = (uint16_t)(tail + 1u);   /* publish consumption to producer */
        } else {
            /* Ring drained: disable TXE interrupts until the next enqueue re-arms
             * them, otherwise TXE (permanently set while DR is empty) would storm
             * the CPU with interrupts. */
            USART2->CR1 = cr1 & ~USART_CR1_TXEIE;   /* CR1 bit7: TXEIE = 0 */
        }
    }
}

uint32_t uart_get_dropped(void)
{
    return s_tx_dropped;   /* single aligned 32-bit read — atomic on Cortex-M4 */
}

void uart_write_str(const char *s)
{
    /* Terminal-friendly newline handling: a bare '\n' from C code becomes a
     * CR+LF pair so Tera Term (and most serial terminals in their default,
     * non-"implicit CR" mode) return the carriage to column 0 AND line-feed,
     * instead of the classic staircase output. Done here so callers can just
     * write "...\n" and not think about it. */
    while (*s != '\0') {
        if (*s == '\n') {
            uart_write_byte((uint8_t)'\r');
        }
        uart_write_byte((uint8_t)*s);
        s++;
    }
}

void uart_write_hex8(uint8_t v)
{
    static const char hexdigits[] = "0123456789ABCDEF";
    uart_write_byte((uint8_t)hexdigits[(v >> 4) & 0x0Fu]);  /* high nibble */
    uart_write_byte((uint8_t)hexdigits[v & 0x0Fu]);         /* low nibble  */
}

void uart_write_hex32(uint32_t v)
{
    /* Most-significant nibble first: shift down by 28,24,...,0 and emit each. */
    static const char hexdigits[] = "0123456789ABCDEF";
    for (int shift = 28; shift >= 0; shift -= 4) {
        uart_write_byte((uint8_t)hexdigits[(v >> shift) & 0x0Fu]);
    }
}

void uart_write_dec(uint32_t v)
{
    /* Build digits least-significant first into a small buffer, then emit in
     * reverse. Max uint32 = 4294967295 -> 10 digits, so 10 chars is enough. */
    char buf[10];
    int n = 0;

    if (v == 0u) {
        uart_write_byte((uint8_t)'0');
        return;
    }
    while (v != 0u) {
        buf[n] = (char)('0' + (v % 10u));   /* one decimal digit */
        v /= 10u;
        n++;
    }
    while (n > 0) {
        n--;
        uart_write_byte((uint8_t)buf[n]);   /* emit in correct (reversed) order */
    }
}

int uart_read_byte_nonblock(void)
{
    /* Read SR once, up front. This snapshot also matters for overrun recovery:
     * on the F4 USART, an overrun (ORE) is cleared by a read of SR followed by a
     * read of DR (RM0390 §24.6.1). If a byte arrived while we weren't looking and
     * ORE latched, RXNE stays asserted but the sticky ORE would block further
     * RX interrupts/state — our SR-then-DR sequence below clears it either way,
     * so an overrun can never wedge the receiver permanently. */
    uint32_t sr = USART2->SR;

    if ((sr & USART_SR_RXNE) == 0u) {
        return -1;              /* SR bit5: RXNE clear -> no byte waiting */
    }

    /* RXNE set: reading DR returns the byte AND clears RXNE (and, together with
     * the SR read above, clears any latched ORE). Mask to 8 bits (8N1). */
    return (int)(USART2->DR & 0xFFu);
}
