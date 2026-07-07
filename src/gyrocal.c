/*
 * TUGRUL flight controller — Milestone M4 (gyro bias boot calibration)
 *
 * Implementation of the boot-time gyro zero-rate calibration declared in
 * gyrocal.h. Pure integer, no hardware, no float. Single module-private instance
 * (one gyro for the whole vehicle).
 *
 * See gyrocal.h for the raw-LSB rationale, the stationary assumption, and the
 * stillness-guard trade-off.
 */

#include "gyrocal.h"

/* ---- Module-private state (single instance) ------------------------------ */

/*
 * Per-axis running sum of raw samples, int32.
 *
 * OVERFLOW PROOF: |raw|max = 32768 (int16 min magnitude); the worst-case window
 * accumulates GYROCAL_SAMPLES of it:
 *     1000 * 32768 = 32,768,000  <  2,147,483,647 (2^31 - 1).
 * (The spec quoted 1000 * 32767 = 32,767,000; even the slightly larger -32768
 * magnitude is an order of magnitude inside int32.) No int64 needed.
 */
static int32_t  s_acc[3];

static int16_t  s_min[3];        /* per-axis window minimum (stillness guard)  */
static int16_t  s_max[3];        /* per-axis window maximum (stillness guard)  */
static int16_t  s_bias[3];       /* stored bias, raw LSB, valid once s_done    */
static uint32_t s_count;         /* samples collected in the CURRENT window    */
static uint32_t s_restarts;      /* times the stillness guard restarted        */
static int      s_done;          /* 1 once calibration has completed           */

/*
 * Symmetric-rounding average of a window sum over GYROCAL_SAMPLES.
 *
 * C integer division truncates TOWARD ZERO, so a plain (sum / N) would round
 * negative sums the opposite way to positive ones (e.g. -102.5 -> -102 but
 * +102.5 -> +102), biasing the correction. To round half away from zero
 * symmetrically we add +N/2 to a non-negative sum, or subtract N/2 from a
 * negative one (done here by negating, adding N/2, dividing, negating back — so
 * every divide operates on a NON-negative numerator where trunc == floor).
 *
 * The result is a mean of int16 samples, so the (int16_t) cast cannot lose data,
 * but the bound is ASYMMETRIC: the most negative mean is -32768 (all samples at
 * INT16_MIN), which DOES fit int16; the most positive mean is +32767 (all samples
 * at INT16_MAX), because no sample can exceed +32767. So +32768 is unreachable —
 * every representable window average lands in [-32768, +32767].
 */
static int16_t avg_round(int32_t sum)
{
    int32_t n    = (int32_t)GYROCAL_SAMPLES;
    int32_t half = n / 2;                 /* N/2 = 500 */
    int32_t avg  = (sum >= 0) ?  ((sum + half) / n)
                             : -(((-sum) + half) / n);
    return (int16_t)avg;
}

/* Saturate an int32 to the int16 range (see gyrocal_apply rationale). */
static int16_t sat_i16(int32_t v)
{
    if (v >  32767) { return  32767; }
    if (v < -32768) { return -32768; }
    return (int16_t)v;
}

void gyrocal_init(void)
{
    for (int a = 0; a < 3; a++) {
        s_acc[a]  = 0;
        s_min[a]  = 0;
        s_max[a]  = 0;
        s_bias[a] = 0;   /* zero bias until completion -> gyrocal_apply is a no-op */
    }
    s_count    = 0u;
    s_restarts = 0u;
    s_done     = 0;      /* NOT-CALIBRATED */
}

int gyrocal_feed(int16_t gx, int16_t gy, int16_t gz)
{
    if (s_done) {
        return 0;   /* defensive: never accumulate after completion */
    }

    int16_t g[3] = { gx, gy, gz };

    /* Extend (or, at window start, seed) the per-axis min/max with this sample. */
    if (s_count == 0u) {
        for (int a = 0; a < 3; a++) {
            s_min[a] = g[a];
            s_max[a] = g[a];
        }
    } else {
        for (int a = 0; a < 3; a++) {
            if (g[a] < s_min[a]) { s_min[a] = g[a]; }
            if (g[a] > s_max[a]) { s_max[a] = g[a]; }
        }
    }

    /* Accumulate (int32; overflow proof at s_acc). */
    for (int a = 0; a < 3; a++) {
        s_acc[a] += (int32_t)g[a];
    }
    s_count++;

    /*
     * STILLNESS GUARD: if any axis's peak-to-peak spread now exceeds the
     * threshold, the board moved — throw the whole window away and restart.
     * (Compare in int32 so max-min cannot overflow int16.) The min/max are not
     * cleared here; they are re-seeded on the next feed when s_count == 0.
     */
    for (int a = 0; a < 3; a++) {
        if ((int32_t)s_max[a] - (int32_t)s_min[a] > GYROCAL_MAX_SPREAD_LSB) {
            s_acc[0] = 0;
            s_acc[1] = 0;
            s_acc[2] = 0;
            s_count  = 0u;
            s_restarts++;
            return 0;
        }
    }

    /* Window full and still: store the per-axis bias and latch calibrated. */
    if (s_count >= GYROCAL_SAMPLES) {
        for (int a = 0; a < 3; a++) {
            s_bias[a] = avg_round(s_acc[a]);
        }
        s_done = 1;
        return 1;   /* completion edge */
    }

    return 0;
}

int gyrocal_done(void)
{
    return s_done;
}

void gyrocal_apply(int16_t *gx, int16_t *gy, int16_t *gz)
{
    /* Subtract the raw-LSB bias in place, saturating to int16 (see header). */
    *gx = sat_i16((int32_t)*gx - (int32_t)s_bias[0]);
    *gy = sat_i16((int32_t)*gy - (int32_t)s_bias[1]);
    *gz = sat_i16((int32_t)*gz - (int32_t)s_bias[2]);
}

int16_t gyrocal_bias(int axis)
{
    if (axis < 0 || axis > 2) {
        return 0;
    }
    return s_bias[axis];
}

uint32_t gyrocal_progress(void)
{
    return s_count;
}

uint32_t gyrocal_restarts(void)
{
    return s_restarts;
}
