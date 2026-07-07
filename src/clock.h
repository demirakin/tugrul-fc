/*
 * TUGRUL flight controller — Milestone M0 (PLL bring-up)
 * System clock + SysTick time base.
 *
 * clock_init()  brings the core from the reset-default 16 MHz HSI up to
 *               84 MHz off the HSE (bypass mode, ST-LINK MCO). It returns
 *               0 on success and a nonzero code per failure point so the
 *               caller can fall back to a visible panic pattern instead of
 *               silently running at the wrong frequency (or hanging).
 * systick_init() arms the SysTick as a 1 ms system tick once the core is at
 *               84 MHz. Must be called AFTER a successful clock_init().
 */

#ifndef CLOCK_H
#define CLOCK_H

#include <stdint.h>

/*
 * Target core/HCLK frequency after clock_init() succeeds.
 *
 * Deliberately a compile-time constant: at M0 we have exactly one clock tree
 * and it never changes at run time, so there is no reason to drag in the CMSIS
 * SystemCoreClock global + SystemCoreClockUpdate() machinery (which re-derives
 * the frequency from the RCC registers). If the clock tree ever becomes
 * dynamic, revisit this. For now it is the single source of truth used by
 * systick_init() to size the 1 ms reload.
 */
#define SYSTEM_CORE_CLOCK_HZ 84000000u

/* Configure HSE-bypass -> PLL -> 84 MHz SYSCLK. Returns 0 on success. */
int clock_init(void);

/* Arm SysTick for a 1 ms tick. Call only after clock_init() returns 0. */
void systick_init(void);

/* Busy-wait 'ms' milliseconds on top of the SysTick counter. */
void delay_ms(uint32_t ms);

/* Milliseconds elapsed since systick_init() (wraps after ~49.7 days). */
uint32_t millis(void);

#endif /* CLOCK_H */
