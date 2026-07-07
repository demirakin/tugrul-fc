/*
 * TUGRUL flight controller — Milestone M0 (PLL bring-up)
 * System clock + SysTick, register-level, no HAL.
 *
 * Clock tree we build here:
 *
 *   HSE (8 MHz, bypass) --> PLLM/4 --> 2 MHz (PFD) --> xPLLN(168) --> 336 MHz (VCO)
 *                                                        |
 *                                                        +-- /PLLP(4) --> 84 MHz SYSCLK
 *                                                        +-- /PLLQ(7) --> 48 MHz (USB, unused)
 *
 *   SYSCLK 84 MHz --> AHB /1  --> HCLK  84 MHz
 *                     APB1 /2 --> PCLK1 42 MHz   (APB1 timers run at x2 = 84 MHz)
 *                     APB2 /1 --> PCLK2 84 MHz   (APB2 timers run at x1 = 84 MHz)
 *
 * Clock-source tradeoff: we use HSE in BYPASS mode, fed by the on-board
 * ST-LINK's 8 MHz MCO on PH0 (Nucleo solder-bridge default — there is no
 * discrete crystal populated). This gives us a crystal-accurate reference
 * (the ST-LINK clock is derived from an HSE crystal on the ST-LINK side),
 * which we need for a stable PLL and correct UART baud/timer periods. The
 * alternative — staying on the internal HSI — is ~1% accurate and drifts
 * with temperature, which is unacceptable for a flight-control time base.
 * HSEBYP MUST be set while HSE is OFF, before HSEON (RM0390 §6.2.1).
 */

#include "clock.h"
#include "stm32f446xx.h"

/*
 * Bounded spin count for every hardware ready flag. At 16 MHz HSI (our state
 * on entry) this is a few milliseconds of head-room — far longer than any of
 * these transitions legitimately take (HSE startup, PLL lock are microseconds
 * to low-ms), so a loop that never exits means a genuine hardware fault, and
 * we return an error instead of hanging the flight controller forever.
 */
#define CLOCK_TIMEOUT 100000u

/* Distinct nonzero return codes so a caller/debugger can see WHERE it failed. */
#define CLK_ERR_HSE_TIMEOUT     1  /* HSERDY never asserted            */
#define CLK_ERR_LATENCY_READBACK 2 /* FLASH_ACR LATENCY read-back wrong */
#define CLK_ERR_PLL_TIMEOUT     3  /* PLLRDY never asserted            */
#define CLK_ERR_SWS_TIMEOUT     4  /* SYSCLK never switched to PLL     */

/* --- PLL field values (see clock tree above). ---------------------------- */
/* PLLM=4: PFD input = 8 MHz / 4 = 2 MHz. ST recommends PFD = 1..2 MHz for the
 * lowest PLL jitter, so 2 MHz is the sweet spot. */
#define PLLM_VALUE 4u
/* PLLN=168: VCO = 2 MHz * 168 = 336 MHz, inside the legal 100..432 MHz band. */
#define PLLN_VALUE 168u
/* PLLP=4: SYSCLK = 336 MHz / 4 = 84 MHz. PLLP field encodes /2,/4,/6,/8 as
 * 0,1,2,3 respectively — so /4 is the encoded value 1 (see below). */
#define PLLP_VALUE 4u
/* PLLQ=7: 336 MHz / 7 = 48 MHz. USB is unused at M0, but the field must still
 * hold a value that keeps the 48 MHz domain <= 48 MHz to stay legal. */
#define PLLQ_VALUE 7u

int clock_init(void)
{
    volatile uint32_t timeout;

    /* ---------------------------------------------------------------------
     * 1) Turn on HSE in bypass mode and wait for it to stabilize.
     *    HSEBYP first (only valid while HSE is off), then HSEON.
     * --------------------------------------------------------------------- */
    RCC->CR |= RCC_CR_HSEBYP;  /* RCC_CR bit18: HSE bypass — external clock, not a crystal oscillator */
    RCC->CR |= RCC_CR_HSEON;   /* RCC_CR bit16: enable HSE */

    timeout = CLOCK_TIMEOUT;
    while (((RCC->CR & RCC_CR_HSERDY) == 0u) && (timeout != 0u)) {
        timeout--;             /* RCC_CR bit17: HSERDY — set by hardware when HSE is stable */
    }
    if (timeout == 0u) {
        return CLK_ERR_HSE_TIMEOUT;
    }

    /* ---------------------------------------------------------------------
     * 2) FLASH latency BEFORE raising the clock. At 84 MHz HCLK the flash
     *    cannot be read at zero wait states. RM0390 rev 6, Table 6 (VDD
     *    2.7-3.6 V): 60 MHz < HCLK <= 90 MHz => 2 wait states. Program it,
     *    then READ BACK and verify — the RM requires confirming LATENCY took
     *    effect before running faster, because if the field did not update we
     *    would fetch corrupt instructions at 84 MHz.
     *    Also enable prefetch + instruction/data caches so the 2 WS penalty is
     *    hidden on sequential fetches (the CPU prefetches ahead / hits cache
     *    instead of stalling 2 cycles on every flash access).
     * --------------------------------------------------------------------- */
    FLASH->ACR = FLASH_ACR_LATENCY_2WS      /* FLASH_ACR LATENCY[3:0] = 2 wait states */
               | FLASH_ACR_PRFTEN           /* FLASH_ACR bit8:  prefetch enable */
               | FLASH_ACR_ICEN             /* FLASH_ACR bit9:  instruction cache enable */
               | FLASH_ACR_DCEN;            /* FLASH_ACR bit10: data cache enable */

    /* Mandatory read-back: confirm LATENCY == 2 before proceeding. */
    timeout = CLOCK_TIMEOUT;
    while (((FLASH->ACR & FLASH_ACR_LATENCY_Msk) != FLASH_ACR_LATENCY_2WS)
           && (timeout != 0u)) {
        timeout--;
    }
    if (timeout == 0u) {
        return CLK_ERR_LATENCY_READBACK;
    }

    /* ---------------------------------------------------------------------
     * 3) Configure the PLL from HSE. PLLCFGR must be written while the PLL is
     *    OFF (it is: reset default has PLLON clear and we have not set it).
     *    Build the whole register in one write rather than read-modify-write:
     *    at reset PLLCFGR holds a default we fully replace anyway, and a single
     *    write avoids a transient illegal field combination.
     *
     *    Field encodings (RM0390 §6.3.2):
     *      PLLM[5:0]   = divider value directly (4)
     *      PLLN[8:6]   = multiplier value directly (168)
     *      PLLP[17:16] = 0->/2, 1->/4, 2->/6, 3->/8  => for /4 write 1 = (4/2 - 1)
     *      PLLSRC[22]  = 1 -> HSE
     *      PLLQ[27:24] = divider value directly (7)
     * --------------------------------------------------------------------- */
    RCC->PLLCFGR =
          ((PLLM_VALUE) << RCC_PLLCFGR_PLLM_Pos)             /* PLLM = 4  */
        | ((PLLN_VALUE) << RCC_PLLCFGR_PLLN_Pos)             /* PLLN = 168 */
        | (((PLLP_VALUE / 2u) - 1u) << RCC_PLLCFGR_PLLP_Pos) /* PLLP /4 -> encoded 1 */
        | RCC_PLLCFGR_PLLSRC_HSE                             /* PLL source = HSE */
        | ((PLLQ_VALUE) << RCC_PLLCFGR_PLLQ_Pos);            /* PLLQ = 7  */

    /* Start the PLL and wait for lock. */
    RCC->CR |= RCC_CR_PLLON;   /* RCC_CR bit24: enable PLL */
    timeout = CLOCK_TIMEOUT;
    while (((RCC->CR & RCC_CR_PLLRDY) == 0u) && (timeout != 0u)) {
        timeout--;             /* RCC_CR bit25: PLLRDY — set by hardware on lock */
    }
    if (timeout == 0u) {
        return CLK_ERR_PLL_TIMEOUT;
    }

    /* ---------------------------------------------------------------------
     * 4) Program the bus prescalers BEFORE switching SYSCLK to the PLL.
     *    Ordering matters: the instant SW selects the PLL, HCLK jumps to
     *    84 MHz. If APB1 were still at /1 at that moment, PCLK1 would briefly
     *    be 84 MHz — nearly 2x its 45 MHz maximum — an out-of-spec transient.
     *    Setting APB1 /2 first guarantees PCLK1 never exceeds 42 MHz across
     *    the switch. AHB /1 and APB2 /1 are safe at 84 MHz (limits 180/90).
     * --------------------------------------------------------------------- */
    {
        uint32_t cfgr = RCC->CFGR;
        cfgr &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
        cfgr |= RCC_CFGR_HPRE_DIV1;   /* AHB  /1 -> HCLK  84 MHz */
        cfgr |= RCC_CFGR_PPRE1_DIV2;  /* APB1 /2 -> PCLK1 42 MHz (<= 45; timers x2 = 84 MHz) */
        cfgr |= RCC_CFGR_PPRE2_DIV1;  /* APB2 /1 -> PCLK2 84 MHz (<= 90; timers x1 = 84 MHz) */
        RCC->CFGR = cfgr;
    }

    /* ---------------------------------------------------------------------
     * 5) Switch SYSCLK to the PLL and confirm the switch actually happened.
     *    SW[1:0] requests the source; SWS[3:2] reports the active source. The
     *    hardware only updates SWS once the new clock is running, so we spin
     *    on SWS == PLL, not on the write completing.
     * --------------------------------------------------------------------- */
    {
        uint32_t cfgr = RCC->CFGR;
        cfgr &= ~RCC_CFGR_SW_Msk;
        cfgr |= RCC_CFGR_SW_PLL;   /* SW = 0b10: request PLL as system clock */
        RCC->CFGR = cfgr;
    }

    timeout = CLOCK_TIMEOUT;
    while (((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL)
           && (timeout != 0u)) {
        timeout--;                 /* SWS[3:2] == 0b10 once PLL is the live SYSCLK */
    }
    if (timeout == 0u) {
        return CLK_ERR_SWS_TIMEOUT;
    }

    /* Core now runs at SYSTEM_CORE_CLOCK_HZ (84 MHz). */
    return 0;
}

/* --------------------------------------------------------------------------
 * SysTick 1 ms time base.
 * -------------------------------------------------------------------------- */

/* Milliseconds since systick_init(). 'volatile' — written in ISR, read in
 * thread context; the compiler must not cache it. */
static volatile uint32_t s_tick_ms = 0u;

void systick_init(void)
{
    /* SysTick is a core (Cortex-M4) peripheral, driven here from the processor
     * clock (HCLK = 84 MHz). For a 1 ms tick we need 84000 counts; the timer
     * counts down from RELOAD to 0 inclusive, so LOAD = counts - 1. */
    SysTick->LOAD = (SYSTEM_CORE_CLOCK_HZ / 1000u) - 1u; /* 84000 - 1 = 83999 */
    SysTick->VAL  = 0u;   /* clear current value + the COUNTFLAG */

    /* CTRL: CLKSOURCE=1 (processor clock, not HCLK/8), TICKINT=1 (assert the
     * SysTick exception on underflow), ENABLE=1 (start counting). */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

/* SysTick exception: fires every 1 ms. Overrides the weak alias in the startup
 * file (which points at Default_Handler). */
void SysTick_Handler(void)
{
    s_tick_ms++;
}

uint32_t millis(void)
{
    return s_tick_ms;   /* single aligned 32-bit read — atomic on Cortex-M4 */
}

void delay_ms(uint32_t ms)
{
    uint32_t start = s_tick_ms;
    /* Unsigned subtraction handles the 32-bit wraparound correctly: even if
     * s_tick_ms rolls over past start mid-wait, (now - start) still yields the
     * true elapsed count modulo 2^32, which is what we want for any ms that
     * fits in 32 bits. */
    while ((s_tick_ms - start) < ms) {
        /* spin — the tick advances in the SysTick ISR */
    }
}
