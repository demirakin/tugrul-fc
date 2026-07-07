/*
 * TUGRUL flight controller — Milestone M4-prep (attitude estimation)
 *
 * Integer-only roll/pitch attitude from the MPU-9250, NO yaw (yaw needs the
 * magnetometer/AK8963, deferred). Everything here is fixed-point — NO float
 * anywhere (project rule: deterministic integer path; the FPU is deliberately
 * left idle at this stage so no lazy-stacking cost enters the control loop).
 *
 * Two building blocks:
 *   - A vectoring-mode CORDIC (atan2 + hypot) that turns accel components into
 *     tilt angles with a bounded, documented error (<= 0.05 deg, well under the
 *     M4 static-accuracy target of ~1 deg).
 *   - A complementary filter (per axis) that trusts the gyro over a short
 *     horizon and lets the accel correct slow drift.
 *
 * Public angle unit is ALWAYS centi-degrees (deg x 100), int32 — matching the
 * mg / centi-dps / centi-degC convention the rest of the firmware already uses.
 * A stale marker is the caller's job: attitude_update() runs ONLY on a good
 * sample, so if the sensor is absent the last estimate simply stops advancing
 * and the caller must treat it as stale (see attitude_valid in main.c).
 *
 * Frame note: inputs are used in the MPU-9250 CHIP frame directly. The chip ->
 * aircraft body mapping is decided at mounting time (docs/axis-conventions.md);
 * until then roll uses (ay,az) and pitch uses (-ax, hypot(ay,az)) as-printed on
 * the package, and gyro roll/pitch rates map to gx/gy. Every such use is
 * commented "chip frame, mapping TBD".
 */

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>
#include "mpu9250.h"        /* mpu_scaled_t (mg / centi-dps inputs) */

/*
 * Vectoring-mode CORDIC, 14 iterations, FIXED count (never early-exit — the
 * point is constant time). Both routines share one core.
 *
 *   cordic_atan2_cd(y, x): atan2(y, x) in centi-degrees, full range (-18000,
 *                          +18000] cd = (-180, +180] deg. All four quadrants
 *                          and the x=0 / y=0 edges are handled explicitly.
 *   cordic_hypot(y, x):    sqrt(x*x + y*y) in the SAME units as the inputs,
 *                          i.e. the CORDIC gain K has been divided back out.
 *
 * Inputs are generic int32 but the deployment feeds small values (accel mg,
 * |v| <~ 2000, and a hypot <~ 3000), far inside every bound below. Documented
 * per-component (|x|,|y|) input bounds — they DIFFER because only hypot runs the
 * gain-correction multiply:
 *   cordic_atan2_cd: safe up to ~150000. Magnitude is discarded, so the only
 *                    limit is the pre-shift iteration overflow.
 *   cordic_hypot:    safe up to ~46000. Tighter: the gain multiply needs
 *                    K*hypot < 2^31/19898 ~= 107925, i.e. hypot < ~65500, so
 *                    each component < ~46000.
 * See the overflow notes in attitude.c.
 */
int32_t cordic_atan2_cd(int32_t y, int32_t x);
int32_t cordic_hypot(int32_t y, int32_t x);

/*
 * Complementary filter, roll + pitch, one instance held privately in attitude.c.
 *
 *   attitude_reset(s):  seed BOTH the filtered state and the gyro-only parallel
 *                       track from the accel tilt angles in *s (snap to the
 *                       measured attitude instead of ramping from zero). Call on
 *                       the first good sample after (re)init.
 *   attitude_update(s): advance one 1 ms tick — gyro predict + accel correct for
 *                       the filtered track, pure gyro integration for the
 *                       parallel track. Call ONCE per millisecond, on a good
 *                       sample only.
 *   attitude_get(...):        latest filtered roll/pitch, centi-degrees.
 *   attitude_get_gyro_only(...): latest pure-gyro-integrated roll/pitch, cd.
 *
 * The gyro-only track carries no accel correction on purpose: the M4 proof is
 * the side-by-side drift comparison (filtered vs raw integration), so both are
 * maintained from the same seed. *s and the out-params must be non-NULL.
 */
void attitude_reset(const mpu_scaled_t *s);
void attitude_update(const mpu_scaled_t *s);
void attitude_get(int32_t *roll_cd, int32_t *pitch_cd);
void attitude_get_gyro_only(int32_t *roll_cd, int32_t *pitch_cd);

#endif /* ATTITUDE_H */
