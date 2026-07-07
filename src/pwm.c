/*
 * TUGRUL flight controller — Milestone M5-prep (servo PWM skeleton)
 * Register-level TIM3 hardware PWM, no HAL, no interrupts.
 *
 * TIMER KERNEL CLOCK (the classic APB-prescaler trap):
 *   TIM3 sits on APB1. clock.c fixes PCLK1 = 42 MHz (APB1 prescaler /2). BUT the
 *   timer kernel clock is NOT PCLK1: RM0390 rev 6 §6.2 (clock tree) states that
 *   when the APB prescaler is different from 1, the timer clock fed to APBx
 *   timers is TWICE the corresponding PCLK. APB1 /2 (!= 1) => TIM3 kernel clock
 *   = 2 * 42 MHz = 84 MHz. (If APB1 were /1 the factor would be 1x = 42 MHz.)
 *   Every period/prescaler number below is derived from this 84 MHz.
 *
 * PWM NUMEROLOGY (all integer, no float):
 *   f_tim  = 84 MHz
 *   PSC    = 83  -> counter clock = 84 MHz / (83 + 1) = 1 MHz  (1 tick = 1 us)
 *   ARR    = 19999 -> period = (19999 + 1) ticks = 20000 us = 20 ms = 50 Hz
 *   Because 1 tick == 1 us, a CCR value IS the pulse width in microseconds:
 *   CCR = 1500 -> 1.5 ms high per 20 ms frame.
 *
 * OUTPUT MODE:
 *   PWM mode 1 (OCxM = 110): output is ACTIVE while CNT < CCRx, i.e. the pin is
 *   high for the first CCRx microseconds of each 20 ms frame — a standard servo
 *   pulse. Edge-aligned up-counting (CR1 DIR=0, CMS=00 — both reset defaults).
 *
 * PRELOAD (why it matters for servos):
 *   OCxPE (per-channel CCR preload) + ARPE (ARR preload) make CCR/ARR writes land
 *   in shadow registers that copy into the live registers only at the UPDATE
 *   event (counter wrap). So a pwm_set_us() mid-frame can never produce a runt or
 *   stretched pulse — the surface never twitches on an asynchronous write. UG is
 *   pulsed once after config to force the first shadow load before the counter
 *   starts, so frame 1 already carries the safe defaults.
 *
 * NO BDTR/MOE HERE:
 *   TIM3 is a general-purpose timer; its outputs are live as soon as CCER CCxE is
 *   set. The main-output-enable gate (BDTR.MOE) exists only on the ADVANCED
 *   timers TIM1/TIM8 — on those this code would ALSO need TIMx->BDTR |= MOE or the
 *   pins stay dark. Not applicable to TIM3.
 */

#include "pwm.h"
#include "clock.h"          /* documents the 84 MHz timer kernel clock assumed */
#include "stm32f446xx.h"

/* --- Timing constants (see numerology above). ---------------------------- */
#define PWM_PSC         83u      /* 84 MHz / (83+1) = 1 MHz counter (1 us/tick) */
#define PWM_ARR         19999u   /* (19999+1) us = 20 ms frame = 50 Hz          */

/* --- Servo pulse band + safe defaults (microseconds == CCR counts). -------
 * PWM_US_NEUTRAL / PWM_US_IDLE now live in pwm.h (exported for the M5 failsafe
 * unit — single source of truth). Only the clamp bounds stay private here. */
#define PWM_US_MIN      1000u    /* mechanical / ESC lower stop */
#define PWM_US_MAX      2000u    /* mechanical / ESC upper stop */

/*
 * Clamp a commanded pulse to the physical band. Out-of-range is a CALLER BUG
 * (the stabilization loop should never ask for it), but a firmware bug must not
 * be allowed to drive a control surface past its mechanical stop or an ESC below
 * idle — so we hard-limit here rather than trust the caller.
 */
static uint32_t pwm_clamp_us(uint32_t us)
{
    if (us < PWM_US_MIN) {
        return PWM_US_MIN;
    }
    if (us > PWM_US_MAX) {
        return PWM_US_MAX;
    }
    return us;
}

void pwm_init(void)
{
    /* 1) Peripheral clocks: TIM3 (APB1) plus the two GPIO ports carrying the
     *    outputs. GPIOA is already clocked by main.c and GPIOB by spi1_init(),
     *    but |= is idempotent, so enabling them here keeps this unit
     *    self-contained. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                     /* RCC_APB1ENR bit1: TIM3 clock enable */
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN); /* RCC_AHB1ENR bit0/bit1: GPIOA/GPIOB clock enable */

    /* Read-back barriers (project pattern, F4 "delay after RCC enable"): stall
     * until each clock-enable is live before configuring the peripheral, so the
     * config below cannot land on unclocked silicon and be dropped. */
    volatile uint32_t apb1enr_readback = RCC->APB1ENR;
    (void)apb1enr_readback;
    volatile uint32_t ahb1enr_readback = RCC->AHB1ENR;
    (void)ahb1enr_readback;

    /*
     * 2) Route the three pins to TIM3 alternate function.
     *    PIN MAP CHECK (Nucleo-F446RE, verified free vs the project pin map;
     *    occupied: PA2/PA3 USART2, PA5 LD2, PA8 jitter probe, PB3/4/5/6 SPI1+CS,
     *    PA13/PA14 SWD):
     *      TIM3_CH1 = PA6 (AF2) -> aileron
     *      TIM3_CH2 = PA7 (AF2) -> elevator
     *      TIM3_CH3 = PB0 (AF2) -> throttle
     *    All three are AF2 on the F446RE datasheet "Alternate function mapping".
     *    MODER is 2 bits/pin: 00=input, 01=output, 10=alt-fn, 11=analog. Clear
     *    each field, then set bit1 -> 0b10 (alternate function). RMW touches only
     *    the target pins. */
    GPIOA->MODER &= ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7);   /* clear PA6/PA7 mode */
    GPIOA->MODER |=  (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); /* PA6/PA7 = 10 alt-fn */
    GPIOB->MODER &= ~GPIO_MODER_MODER0;                          /* clear PB0 mode */
    GPIOB->MODER |=  GPIO_MODER_MODER0_1;                        /* PB0 = 10 alt-fn */

    /* 3) Select AF2 (TIM3) on each pin. AFRL holds 4 bits/pin for pins 0..7.
     *    AF2 = 0b0010 = bit1 of the field (AFSELx_1). Clear then set. */
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);    /* clear PA6/PA7 AF */
    GPIOA->AFR[0] |=  (GPIO_AFRL_AFSEL6_1 | GPIO_AFRL_AFSEL7_1); /* PA6/PA7 = AF2 (TIM3 CH1/CH2) */
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL0;                          /* clear PB0 AF */
    GPIOB->AFR[0] |=  GPIO_AFRL_AFSEL0_1;                        /* PB0 = AF2 (TIM3 CH3) */

    /* OSPEEDR is deliberately left at reset (low speed): 50 Hz PWM edges have no
     * slew-rate need, and low speed keeps EMI/ringing down on servo leads. */

    /* 4) Timer time base: 1 MHz counter, 20 ms frame. Written while the counter
     *    is stopped (CEN still 0 at reset). */
    TIM3->PSC = PWM_PSC;    /* TIM3_PSC: prescaler = 83 -> 1 MHz counter tick */
    TIM3->ARR = PWM_ARR;    /* TIM3_ARR: auto-reload = 19999 -> 20 ms / 50 Hz */

    /* 5) Channel output config. PWM mode 1 (OCxM = 110) + per-channel preload
     *    (OCxPE) on CH1/CH2/CH3. Reset value of CCMRx is 0, so a full assignment
     *    is clean and also leaves CH4 (CCMR2 high half) frozen = spare. */
    TIM3->CCMR1 =
          (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) | TIM_CCMR1_OC1PE   /* CH1: PWM mode 1 + preload */
        | (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1) | TIM_CCMR1_OC2PE;  /* CH2: PWM mode 1 + preload */
    TIM3->CCMR2 =
          (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) | TIM_CCMR2_OC3PE;  /* CH3: PWM mode 1 + preload */

    /* 6) SAFE DEFAULTS, loaded BEFORE the counter starts. Surfaces neutral,
     *    throttle idle. These are the failsafe-safe values the M5 failsafe unit
     *    will later re-command on sensor timeout (forward reference). 1 us/tick
     *    means the CCR value is the microsecond value directly. */
    TIM3->CCR1 = PWM_US_NEUTRAL;   /* TIM3_CCR1: PA6 aileron  = 1500 us neutral */
    TIM3->CCR2 = PWM_US_NEUTRAL;   /* TIM3_CCR2: PA7 elevator = 1500 us neutral */
    TIM3->CCR3 = PWM_US_IDLE;      /* TIM3_CCR3: PB0 throttle = 1000 us idle    */

    /* 7) Enable the three outputs. On a general-purpose timer the pin goes live
     *    as soon as CCxE is set — no BDTR/MOE gate (that is TIM1/TIM8 only). */
    TIM3->CCER = (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E); /* CH1/CH2/CH3 output enable */

    /* 8) Enable ARR preload. With ARPE + OCxPE, ARR/CCR writes take effect only
     *    at the update event -> glitch-free reloads (no runt pulse to a servo). */
    TIM3->CR1 |= TIM_CR1_ARPE;   /* TIM3_CR1 bit7: auto-reload preload enable */

    /* 9) Force an update event to copy PSC/ARR/CCR shadows into the live
     *    registers NOW, so the very first frame already carries the safe
     *    defaults instead of the reset (0) values. */
    TIM3->EGR = TIM_EGR_UG;      /* TIM3_EGR bit0: update generation (load shadows) */

    /* 10) Start the counter. Edge-aligned up-count (DIR=0, CMS=00 reset defaults)
     *     begins; the three pins now emit 50 Hz PWM at the safe defaults. */
    TIM3->CR1 |= TIM_CR1_CEN;    /* TIM3_CR1 bit0: counter enable */
}

/* Map a logical channel to its CCR register. Returns NULL for an unknown enum
 * value (defensive; the enum has no other members). */
static volatile uint32_t *pwm_ccr(pwm_ch_t ch)
{
    switch (ch) {
        case PWM_AILERON:  return &TIM3->CCR1;   /* PA6 */
        case PWM_ELEVATOR: return &TIM3->CCR2;   /* PA7 */
        case PWM_THROTTLE: return &TIM3->CCR3;   /* PB0 */
        default:           return (volatile uint32_t *)0;
    }
}

void pwm_set_us(pwm_ch_t ch, uint32_t us)
{
    volatile uint32_t *ccr = pwm_ccr(ch);
    if (ccr != (volatile uint32_t *)0) {
        /* CCR preload (OCxPE) means this write shadows and applies at the next
         * update event, not mid-pulse. 1 us/tick -> write the microsecond value. */
        *ccr = pwm_clamp_us(us);   /* TIM3_CCRx: pulse width in us (clamped 1000..2000) */
    }
}

uint32_t pwm_get_us(pwm_ch_t ch)
{
    volatile uint32_t *ccr = pwm_ccr(ch);
    if (ccr != (volatile uint32_t *)0) {
        return *ccr;   /* TIM3_CCRx read-back: current pulse width in us */
    }
    return 0u;
}
