/*
 * TUGRUL flight controller — Milestone M4-prep (attitude estimation)
 *
 * Integer-only CORDIC (atan2 + hypot) and a per-axis complementary filter.
 * NO float anywhere (grep-clean). Constant work: the CORDIC runs a FIXED 14
 * iterations with no early exit, and every other loop-free branch is a sign
 * test, never a value-dependent loop.
 *
 * Angle representations used here:
 *   - Public API: centi-degrees (deg x 100), int32.  90 deg = 9000 cd.
 *   - CORDIC internal angle accumulator: Q8 on cd, i.e. cd x 2^8. This keeps
 *     ~1/256 cd (~0.004 deg) of headroom so the tiny late-iteration table
 *     entries are not quantised away (a plain-cd table would round the last
 *     entries badly and blow the error budget).
 *   - Filter state: Q8 on cd, cd x 2^8, so a single 1 ms gyro increment (which
 *     can be a small fraction of a cd) is not lost per tick.
 *
 * Right-shift of a signed value is arithmetic on arm-none-eabi (two's
 * complement, sign-extending) — the same implementation-defined behaviour the
 * rest of this firmware already relies on. Used below for the CORDIC shifts and
 * the Q8->cd conversions.
 */

#include "attitude.h"

/* ---- CORDIC configuration ------------------------------------------------ */

#define CORDIC_ITERS        14  /* fixed iteration count (constant time)        */
#define ANGLE_FRAC_BITS     8   /* Q8: internal angle unit is cd x 2^8          */

/*
 * Pre-shift applied to (x,y) before iterating. The per-iteration shifts x>>i /
 * y>>i (i up to 13) would annihilate the low bits of small inputs (accel mg are
 * only ~11 bits), starving the last iterations and coarsening the angle. Shifting
 * the inputs left by 12 first preserves those bits through the whole run.
 *
 * Overflow proof: during vectoring, |x| grows by at most the CORDIC gain
 * K ~= 1.6468, so the largest internal magnitude is K * hypot(x,y) * 2^12.
 * Requiring that < 2^31 gives hypot(x,y) < 2^31 / (1.6468 * 4096) ~= 318000.
 * Our inputs (accel mg <~2000, hypot <~3000) sit orders of magnitude below that;
 * even full int16-range inputs (hypot <~46341) are safe. Documented bound in the
 * header: |x|,|y| <~ 150000.
 */
#define CORDIC_PRESHIFT     12

/*
 * atan(2^-i) table, i = 0..13, stored in the internal Q8-on-cd unit:
 *   table[i] = round( atan(2^-i) [deg] * 100 * 256 )   ( = deg * 25600 )
 * Values computed at write time from atan(2^-i); derivation shown per line
 * (degrees then x25600). Half-LSB table-quantisation error is <= 0.5/256 cd =
 * ~0.00002 deg per entry, <= ~0.0003 deg summed over 14 entries — negligible.
 *
 *   i   2^-i           atan [deg]        * 25600      -> rounded
 *   0   1.0            45.00000000       1152000.0       1152000
 *   1   0.5            26.56505118        680065.3        680065
 *   2   0.25           14.03624347        359327.8        359328
 *   3   0.125           7.12501635        182400.4        182400
 *   4   0.0625          3.57633437         91554.2         91554
 *   5   0.03125         1.78991061         45821.7         45822
 *   6   0.015625        0.89517371         22916.4         22916
 *   7   0.0078125       0.44761417         11458.9         11459
 *   8   0.00390625      0.22381050          5729.5          5730
 *   9   0.001953125     0.11190555          2864.8          2865
 *  10   0.0009765625    0.05595285          1432.4          1432
 *  11   0.00048828125   0.02797643           716.2           716
 *  12   0.000244140625  0.01398822           358.1           358
 *  13   0.0001220703125 0.00699411           179.1           179
 *
 * Sum of all entries = 2556824 (Q8) = 99.876 deg, the CORDIC convergence range;
 * that is the max |accumulator|, far inside int32. Algorithmic angle error after
 * 14 iterations is <= atan(2^-13) ~= 0.007 deg; adding table quantisation and the
 * final cd rounding (~0.005 deg) the worst-case total is ~0.012 deg << 0.05 deg.
 */
static const int32_t atan_table_q8[CORDIC_ITERS] = {
    1152000, 680065, 359328, 182400, 91554, 45822, 22916,
    11459,   5730,   2865,   1432,   716,   358,   179
};

/*
 * CORDIC gain correction. Vectoring mode leaves x = K * hypot(x,y) with
 *   K = prod_{i=0..13} sqrt(1 + 2^-2i) ~= 1.64676025812.
 * We divide it back out with 1/K in Q15:
 *   1/K = 0.60725293501,  round(0.60725293501 * 32768) = 19898.
 * hypot = descaled_x * 19898 >> 15 (with rounding), where descaled_x has had the
 * 2^12 pre-shift removed first — doing it in that order keeps the whole gain
 * correction in int32 (see cordic core), no int64 needed.
 */
#define CORDIC_INV_GAIN_Q15 19898

#define Q15_ONE             32768   /* 1.0 in Q15                                */
#define Q15_ROUND           16384   /* 0.5 in Q15, for round-half-up on >>15     */

/* Round-half-away-from-zero conversion Q8-on-cd -> plain centi-degrees. */
static int32_t q8_to_cd(int32_t q8)
{
    return (q8 >= 0) ? ( (q8 + (1 << (ANGLE_FRAC_BITS - 1))) >> ANGLE_FRAC_BITS)
                     : -(((-q8) + (1 << (ANGLE_FRAC_BITS - 1))) >> ANGLE_FRAC_BITS);
}

/*
 * Shared CORDIC vectoring core. Writes atan2(y,x) in centi-degrees to *angle_cd
 * (if non-NULL) and hypot(x,y) in input units to *mag (if non-NULL). Quadrants
 * and the x=0 / y=0 edges are handled explicitly.
 */
static void cordic_vec(int32_t y, int32_t x, int32_t *angle_cd, int32_t *mag)
{
    /* Edge: x == 0. atan2 is +/-90 deg on the sign of y (0 for the origin);
     * hypot(0,y) = |y| exactly. Avoids feeding a zero x into the fold/iterate. */
    if (x == 0) {
        if (y == 0) {
            if (angle_cd) { *angle_cd = 0; }
            if (mag)      { *mag = 0; }
            return;
        }
        if (angle_cd) { *angle_cd = (y > 0) ? 9000 : -9000; } /* +/-90.00 cd */
        if (mag)      { *mag = (y > 0) ? y : -y; }
        return;
    }

    /*
     * Quadrant fold: vectoring converges only for x > 0. For x < 0 rotate the
     * vector by 180 deg (negate both components) into the right half-plane, run
     * the core, then add back +/-180 deg by the ORIGINAL y sign:
     *   x<0, y>=0 (2nd quadrant): core in [-90,0],  +180 -> (90,180]
     *   x<0, y<0  (3rd quadrant): core in [0,90),   -180 -> (-180,-90)
     */
    int32_t add180 = 0;
    int32_t xv = x;
    int32_t yv = y;
    if (x < 0) {
        xv = -x;
        yv = -y;
        add180 = (y >= 0) ? 18000 : -18000;   /* +/-180.00 cd */
    }

    /* Pre-shift for iteration precision (overflow proof at CORDIC_PRESHIFT). */
    xv <<= CORDIC_PRESHIFT;
    yv <<= CORDIC_PRESHIFT;

    int32_t ang_q8 = 0;
    for (int i = 0; i < CORDIC_ITERS; i++) {
        int32_t xs = xv >> i;   /* arithmetic shift (sign-extending) on target */
        int32_t ys = yv >> i;
        if (yv >= 0) {
            /* y positive -> rotate clockwise, add the micro-angle. */
            xv = xv + ys;
            yv = yv - xs;
            ang_q8 += atan_table_q8[i];
        } else {
            xv = xv - ys;
            yv = yv + xs;
            ang_q8 -= atan_table_q8[i];
        }
    }
    /* After convergence yv ~ 0 and xv = K * hypot * 2^PRESHIFT, positive. */

    if (angle_cd) {
        *angle_cd = q8_to_cd(ang_q8) + add180;
    }
    if (mag) {
        /* Remove the 2^PRESHIFT first (xv drops to ~K*hypot), THEN apply 1/K in
         * Q15 — keeping both multiplies in int32. Binding overflow limit is this
         * second multiply: descaled = K*hypot must satisfy descaled * 19898 <
         * 2^31, i.e. descaled < 107925  ->  hypot < ~65500 (each component <
         * ~46000). Accel-sized inputs (hypot <~3000 -> descaled ~5000) sit orders
         * of magnitude inside that. (The atan2-only path skips this multiply, so
         * it tolerates the wider pre-shift bound instead — see header.) */
        int32_t descaled = (xv + (1 << (CORDIC_PRESHIFT - 1))) >> CORDIC_PRESHIFT;
        *mag = (descaled * CORDIC_INV_GAIN_Q15 + Q15_ROUND) >> 15;
    }
}

int32_t cordic_atan2_cd(int32_t y, int32_t x)
{
    int32_t a;
    cordic_vec(y, x, &a, 0);
    return a;
}

int32_t cordic_hypot(int32_t y, int32_t x)
{
    int32_t m;
    cordic_vec(y, x, 0, &m);
    return m;
}

/* ---- Complementary filter ------------------------------------------------ */

/*
 * alpha in Q15. alpha = 0.98 -> round(0.98 * 32768) = 32113 (= 0.980011).
 * Meaning: each 1 ms tick keeps 98% of the gyro-propagated estimate and pulls 2%
 * toward the accel tilt. The 2%/tick accel pull has a ~50 ms time constant
 * (1/(1-alpha) ticks at dt=1 ms), so the gyro is trusted over ~50 ms and the
 * accel corrects only slower drift. alpha is a tuning knob that belongs to the
 * hardware bring-up day (needs real gyro-noise vs accel-vibration data); this
 * value is a defensible starting point, flagged for revisit. NEEDS CLARIFICATION
 * deferred: final alpha is a bench-tuning task, not a code decision.
 */
#define ALPHA_Q15           32113
#define ONE_MINUS_ALPHA_Q15 (Q15_ONE - ALPHA_Q15)   /* 655 = 0.019989 */

/*
 * Disagreement clamp for the blend, in Q8-on-cd. The blend correction is
 * diff_q8 * ONE_MINUS_ALPHA_Q15 >> 15, and that multiply must stay in int32:
 *   diff_q8 <= 3000000  =>  3000000 * 655 = 1,965,000,000 < 2,147,483,647 (2^31-1).
 * A raw diff could otherwise reach ~2 * 180_00 * 256 = ~9.2e6 (accel vs predicted
 * at opposite extremes), and * 655 = ~6.0e9 would overflow int32. Physically any
 * accel/gyro disagreement beyond ~117 deg (3000000/256/100) is a fault or a roll
 * wrap, not real motion, so clamping the correction there is honest and keeps the
 * whole blend int32-only with FULL Q8 precision in the normal small-disagreement
 * regime. (Roll wrap at +/-180 is a known limitation left for hardware day; pitch
 * cannot wrap because hypot keeps its atan2 x-argument >= 0, bounding pitch to
 * [-90,90].)
 */
#define DIFF_CLAMP_Q8       3000000

/* Ticks per second: dt = 1 ms -> 1000 ticks. */
#define TICKS_PER_SEC       1000

/*
 * Saturation bound for the gyro-only integrator, in Q8-on-cd. +/-200000 cd =
 * +/-2000 deg — comfortably past anything the drift demo shows, but far below
 * int32. Without it, a pure += would eventually reach signed overflow (UB) after
 * ~5-6 min at max slew; saturating keeps the demo honest with no UB risk.
 */
#define GYRO_ONLY_SAT_Q8    (200000 << ANGLE_FRAC_BITS)   /* 51,200,000 */

/* Filtered roll/pitch and the parallel gyro-only track, all Q8-on-cd int32. */
static int32_t roll_q8;
static int32_t pitch_q8;
static int32_t roll_gyro_q8;
static int32_t pitch_gyro_q8;

/*
 * Per-tick gyro increment in Q8-on-cd from a centi-dps rate.
 * Derivation: rate is deg*100/s (centi-dps). Over dt = 1 ms the angle change is
 *   d[deg] = (cdps/100) * 0.001 ; in cd (x100): d[cd] = cdps/1000 ;
 *   in Q8 (x256):                 d_q8 = cdps * 256 / 1000  (round-half-away).
 * Bound: cdps <= 25000 (+/-250 dps) -> 25000*256 = 6.4e6, no overflow.
 */
static int32_t gyro_delta_q8(int32_t cdps)
{
    int32_t num = cdps * 256;   /* |num| <= 6.4e6 */
    return (num >= 0) ? ( (num + TICKS_PER_SEC / 2) / TICKS_PER_SEC)
                      : -(((-num) + TICKS_PER_SEC / 2) / TICKS_PER_SEC);
}

/*
 * Blend correction term: (1-alpha) * (accel - predicted), all in Q8-on-cd, with
 * the disagreement clamped so the single multiply stays int32 (proof above).
 * Round-half-away on the >>15.
 */
static int32_t blend_correction_q8(int32_t diff_q8)
{
    if (diff_q8 >  DIFF_CLAMP_Q8) { diff_q8 =  DIFF_CLAMP_Q8; }
    else if (diff_q8 < -DIFF_CLAMP_Q8) { diff_q8 = -DIFF_CLAMP_Q8; }

    int32_t prod = diff_q8 * ONE_MINUS_ALPHA_Q15;   /* |prod| < 2^31 (proven) */
    return (prod >= 0) ? ( (prod + Q15_ROUND) >> 15)
                       : -(((-prod) + Q15_ROUND) >> 15);
}

/*
 * Saturating add for the gyro-only integrator (Q8-on-cd). 'acc' is already
 * clamped to +/-GYRO_ONLY_SAT_Q8 and |delta| <= 6400, so acc+delta cannot
 * overflow int32 before the clamp — the clamp then holds the bound.
 */
static int32_t sat_add_q8(int32_t acc, int32_t delta)
{
    int32_t v = acc + delta;
    if (v >  GYRO_ONLY_SAT_Q8) { v =  GYRO_ONLY_SAT_Q8; }
    else if (v < -GYRO_ONLY_SAT_Q8) { v = -GYRO_ONLY_SAT_Q8; }
    return v;
}

/*
 * Accel tilt angles from a scaled sample, CHIP frame (mapping TBD at mounting,
 * docs/axis-conventions.md):
 *   roll  = atan2(ay, az)
 *   pitch = atan2(-ax, hypot(ay, az))   -> x-arg >= 0, so pitch in [-90,90].
 * mg values fit int32 trivially; hypot keeps input units (mg).
 */
static void accel_angles_cd(const mpu_scaled_t *s, int32_t *roll_cd, int32_t *pitch_cd)
{
    *roll_cd  = cordic_atan2_cd(s->ay_mg, s->az_mg);
    *pitch_cd = cordic_atan2_cd(-s->ax_mg, cordic_hypot(s->ay_mg, s->az_mg));
}

void attitude_reset(const mpu_scaled_t *s)
{
    int32_t r_cd, p_cd;
    accel_angles_cd(s, &r_cd, &p_cd);

    /* Snap both tracks to the measured tilt (seed = accel angle << 8 -> Q8). */
    roll_q8       = r_cd << ANGLE_FRAC_BITS;
    pitch_q8      = p_cd << ANGLE_FRAC_BITS;
    roll_gyro_q8  = roll_q8;     /* gyro-only starts aligned for a fair drift race */
    pitch_gyro_q8 = pitch_q8;
}

void attitude_update(const mpu_scaled_t *s)
{
    /*
     * Gyro rates map to roll/pitch as gx/gy in the CHIP frame (mapping and any
     * sign flip TBD at mounting, docs/axis-conventions.md).
     */
    int32_t d_roll_q8  = gyro_delta_q8(s->gx_cdps);
    int32_t d_pitch_q8 = gyro_delta_q8(s->gy_cdps);

    /* Accel reference angles for this tick. */
    int32_t roll_cd, pitch_cd;
    accel_angles_cd(s, &roll_cd, &pitch_cd);

    /*
     * Complementary blend, int32-only:
     *   predicted = state + gyro_delta
     *   state     = predicted + (1-alpha)*(accel - predicted)
     *             ( = alpha*predicted + (1-alpha)*accel )
     */
    int32_t roll_pred_q8 = roll_q8 + d_roll_q8;
    roll_q8 = roll_pred_q8
              + blend_correction_q8((roll_cd << ANGLE_FRAC_BITS) - roll_pred_q8);

    int32_t pitch_pred_q8 = pitch_q8 + d_pitch_q8;
    pitch_q8 = pitch_pred_q8
               + blend_correction_q8((pitch_cd << ANGLE_FRAC_BITS) - pitch_pred_q8);

    /*
     * Parallel gyro-only track: pure integration, NO accel correction — the
     * point is to watch it drift away from the filtered track (M4 evidence).
     * Saturating add (not a raw +=) caps it at +/-2000 deg: the drift demo never
     * gets near that, but the cap removes the signed-overflow UB a raw integrator
     * would eventually hit at sustained max slew. Still a pure integrator inside
     * the bound.
     */
    roll_gyro_q8  = sat_add_q8(roll_gyro_q8,  d_roll_q8);
    pitch_gyro_q8 = sat_add_q8(pitch_gyro_q8, d_pitch_q8);
}

void attitude_get(int32_t *roll_cd, int32_t *pitch_cd)
{
    *roll_cd  = q8_to_cd(roll_q8);
    *pitch_cd = q8_to_cd(pitch_q8);
}

void attitude_get_gyro_only(int32_t *roll_cd, int32_t *pitch_cd)
{
    *roll_cd  = q8_to_cd(roll_gyro_q8);
    *pitch_cd = q8_to_cd(pitch_gyro_q8);
}
