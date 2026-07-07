/*
 * TUGRUL flight controller — Milestone M4 (gyro bias boot calibration)
 *
 * One-shot boot calibration of the gyro's static (zero-rate) bias. Averages
 * GYROCAL_SAMPLES good gyro samples with the board held STILL, stores the mean
 * per axis in RAW LSB, and thereafter subtracts it from every gyro sample before
 * scaling / attitude use. This exists because the clone MPU on the bench carries
 * a large X-axis zero-rate offset (~ -6.2 dps uncalibrated); left in, it
 * dominates the gyro-only drift track and the complementary-filter estimate.
 *
 * WHY RAW-LSB BIAS (not scaled cdps): the correction is applied to the raw
 * int16 sample BEFORE mpu9250_scale(), so the whole scaling / attitude path sees
 * an already-zeroed gyro and needs no knowledge of calibration. Storing the bias
 * in the same units it is measured and subtracted keeps the arithmetic exact
 * (no double rounding through the cdps conversion).
 *
 * STATIONARY ASSUMPTION: the average is only a valid bias if the board did not
 * move during the window. The user is instructed to hold the airframe still at
 * boot. As a cheap honesty guard (NOT a full stillness proof, no variance math)
 * the collector tracks per-axis min/max over the window and RESTARTS the window
 * if any axis's peak-to-peak spread exceeds GYROCAL_MAX_SPREAD_LSB — so a bump
 * or a hand-nudge during calibration cannot poison the bias; it just costs a
 * restart, counted and reported for honesty.
 *
 * TIME / RATE: at the 1 kHz control rate GYROCAL_SAMPLES = 1000 is ~1 s of
 * collection. Calibration is fed once per GOOD sensor read from the control tick;
 * if the sensor is lost it simply stalls (no feeds) and resumes when reads return
 * — the freshness failsafe handles the loss independently.
 *
 * INTEGER ONLY: no float anywhere (project rule). Accumulators are int32; the
 * overflow proof is in gyrocal.c. The average uses symmetric rounding (add ±N/2
 * before an integer divide that truncates toward zero) so a negative bias rounds
 * the same magnitude as its positive mirror.
 *
 * This unit owns no hardware and calls nothing else; it is pure integer state.
 */

#ifndef GYROCAL_H
#define GYROCAL_H

#include <stdint.h>

/*
 * Number of good samples averaged for the bias. 1000 == ~1 s at the 1 kHz
 * control rate. The board MUST be STATIONARY for the whole window (the user is
 * instructed); the stillness guard below restarts the window otherwise.
 */
#define GYROCAL_SAMPLES 1000u

/*
 * Stillness guard threshold: max allowed per-axis peak-to-peak spread (max-min)
 * over the window, in RAW gyro LSB. If ANY axis exceeds it the window restarts.
 *
 * SCALE-DERIVED JUSTIFICATION: mpu9250_init() configures the gyro at FS_SEL=0,
 * i.e. ±250 dps full scale -> 131 LSB/dps (mpu9250.c, product spec §3.1). So:
 *     400 LSB / 131 LSB/dps ≈ 3.05 dps peak-to-peak.
 * A genuinely still bench sits well under this even on a noisy clone (still-gyro
 * noise is a fraction of a dps p-p), while any intentional motion — a bump or a
 * hand nudge is tens of dps — blows straight past it and forces a restart.
 * The trade-off is explicit: too tight and a noisy-but-still clone never
 * converges (endless restarts); too loose and slow motion is accepted as bias.
 * ~3 dps balances the two. Tunable on the bench alongside the filter alpha.
 *
 * NOTE (verifiable deviation from the unit spec): the spec justified this
 * threshold at "±2000 dps FS, 16.4 LSB/dps". This firmware runs the gyro at
 * ±250 dps (131 LSB/dps) — see mpu9250.c MPU_CFG_GYRO_CONFIG=0x00 and
 * MPU9250_GYRO_LSB_PER_DPS=131 — so the ±250 dps scale is used here instead.
 */
#define GYROCAL_MAX_SPREAD_LSB 400

/* Reset to NOT-CALIBRATED and clear all accumulators, min/max, bias, counters.
 * Call once at boot, and again to force a fresh window (e.g. after the sensor is
 * re-initialised mid-calibration, where stale partial sums are untrustworthy). */
void gyrocal_init(void);

/*
 * Feed one GOOD raw gyro sample (raw int16 LSB) while calibrating. Call exactly
 * once per successful sensor read until gyrocal_done() is true. Returns 1 on the
 * single sample that COMPLETES calibration (bias just stored), else 0. Calling
 * after completion is a harmless no-op returning 0.
 */
int gyrocal_feed(int16_t gx, int16_t gy, int16_t gz);

/* 1 once calibration has completed (bias is valid), 0 while still collecting. */
int gyrocal_done(void);

/*
 * Subtract the stored raw-LSB bias from a raw gyro sample, in place, SATURATING
 * to the int16 range. Saturation (not wrap) is deliberate: a near-full-scale raw
 * value minus a bias could exceed int16; wrapping would flip its sign and inject
 * a catastrophic bogus rate into the filter, whereas saturation clamps to the
 * nearest representable extreme — a bounded, monotonic error. In practice the
 * bias is small (hundreds of LSB) and the gyro is far from full scale on a bench,
 * so saturation essentially never engages; it just makes the operation total.
 * Before completion the stored bias is 0, so this is a no-op (safe to call).
 */
void gyrocal_apply(int16_t *gx, int16_t *gy, int16_t *gz);

/* Read back the stored per-axis bias in raw LSB for the heartbeat line.
 * axis: 0=x, 1=y, 2=z. Out-of-range axis returns 0. */
int16_t gyrocal_bias(int axis);

/* Samples collected in the CURRENT window so far (resets to 0 on a restart).
 * Feeds the heartbeat progress line "gyrocal <n>/1000". */
uint32_t gyrocal_progress(void);

/* Number of times the stillness guard restarted the window. Exposed so the
 * heartbeat can report calibration honesty ("cal restarted N times"). */
uint32_t gyrocal_restarts(void);

#endif /* GYROCAL_H */
