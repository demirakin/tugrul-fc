/*
 * TUGRUL flight controller — Milestone M5-prep (servo PWM skeleton)
 * Register-level TIM3 hardware PWM, no HAL, no interrupts.
 *
 * Three 50 Hz servo/ESC PWM outputs on TIM3, one per control surface / throttle.
 * The counter runs at 1 MHz, so a CCR value is the pulse width in MICROSECONDS
 * directly (1000..2000 us standard servo band, 1 us resolution). Servos/ESC are
 * NOT attached yet — this unit is scope-verifiable PWM on the pins.
 *
 * This is a skeleton: it produces safe, steady pulses. There is deliberately NO
 * attitude->PWM mapping here; converting the attitude estimate into surface
 * commands is the stabilization-loop unit's job (scope guard).
 */
#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/*
 * Safe-state pulse widths (microseconds == CCR counts at the 1 MHz timer tick),
 * exported so the M5 failsafe unit commands the EXACT values pwm_init() starts
 * at — one source of truth, no duplicated literals. Surfaces sit at centre,
 * throttle at ESC idle. Kept in the physical band [1000, 2000] (pwm_set_us also
 * clamps). PWM_US_MIN/PWM_US_MAX stay private to pwm.c (clamp-only, not shared).
 */
#define PWM_US_NEUTRAL  1500u    /* aileron/elevator surface centre */
#define PWM_US_IDLE     1000u    /* throttle idle (ESC minimum)     */

/*
 * Logical channels. Order matches the TIM3 channel assignment:
 *   PWM_AILERON  = TIM3_CH1 = PA6 (surface, neutral at init)
 *   PWM_ELEVATOR = TIM3_CH2 = PA7 (surface, neutral at init)
 *   PWM_THROTTLE = TIM3_CH3 = PB0 (ESC,     idle    at init)
 * TIM3_CH4 (PB1) is left unconfigured as a spare.
 */
typedef enum {
    PWM_AILERON  = 0,
    PWM_ELEVATOR = 1,
    PWM_THROTTLE = 2
} pwm_ch_t;

/* Bring up TIM3 + the three GPIOs and start the PWM at safe defaults
 * (surfaces 1500 us neutral, throttle 1000 us idle) BEFORE the counter runs. */
void pwm_init(void);

/* Set a channel's pulse width in microseconds. 'us' is clamped to [1000, 2000]
 * (mechanical / ESC protection). Glitch-free: the new value takes effect at the
 * next update event via the CCR preload shadow. */
void pwm_set_us(pwm_ch_t ch, uint32_t us);

/* Read back a channel's current pulse width in microseconds (from its CCR).
 * Used by the heartbeat / debug line. */
uint32_t pwm_get_us(pwm_ch_t ch);

#endif /* PWM_H */
