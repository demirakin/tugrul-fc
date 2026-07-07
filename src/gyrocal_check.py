#!/usr/bin/env python3
"""
gyrocal_check.py — host mirror of gyrocal.c integer arithmetic (TUGRUL M4).

Mirrors EXACTLY the firmware's integer math so the on-target behaviour can be
checked off-target:
  - int32 accumulate (with an explicit int32-range assertion, as the overflow
    proof in gyrocal.c claims),
  - symmetric-rounding divide (add +/- N/2 before an integer divide that
    truncates toward zero),
  - int16 saturation in apply().

No float is used in the arithmetic under test. Prints PASS/FAIL per test and
exits nonzero if any test fails. The orchestrator runs this; do not build here.
"""

GYROCAL_SAMPLES        = 1000
GYROCAL_MAX_SPREAD_LSB = 400

INT32_MAX = (1 << 31) - 1
INT32_MIN = -(1 << 31)
INT16_MAX = 32767
INT16_MIN = -32768


def avg_round(total):
    """Symmetric-rounding average over GYROCAL_SAMPLES.

    Mirrors gyrocal.c avg_round(): every divide operates on a NON-negative
    numerator, where Python // (floor) equals C's trunc-toward-zero. Rounds half
    away from zero, symmetric across sign.
    """
    n = GYROCAL_SAMPLES
    half = n // 2                      # N/2 = 500
    if total >= 0:
        return (total + half) // n
    return -(((-total) + half) // n)


def sat_i16(v):
    """Saturate to the int16 range (mirrors gyrocal.c sat_i16)."""
    if v > INT16_MAX:
        return INT16_MAX
    if v < INT16_MIN:
        return INT16_MIN
    return v


class GyroCal:
    """Bit-for-bit mirror of the gyrocal.c state machine."""

    def __init__(self):
        self.init()

    def init(self):
        self.acc = [0, 0, 0]
        self.mn = [0, 0, 0]
        self.mx = [0, 0, 0]
        self.bias = [0, 0, 0]
        self.count = 0
        self.restarts = 0
        self.done = False

    def feed(self, gx, gy, gz):
        if self.done:
            return 0
        # Python ints are unbounded, but the firmware's gyrocal_feed() takes
        # int16_t parameters — a wider value could never physically reach it.
        # Assert the int16 domain so the mirror is TOTAL and cannot silently
        # validate inputs the C function could not receive.
        for v in (gx, gy, gz):
            assert INT16_MIN <= v <= INT16_MAX, "input out of int16 range"
        g = [gx, gy, gz]

        if self.count == 0:
            for a in range(3):
                self.mn[a] = g[a]
                self.mx[a] = g[a]
        else:
            for a in range(3):
                if g[a] < self.mn[a]:
                    self.mn[a] = g[a]
                if g[a] > self.mx[a]:
                    self.mx[a] = g[a]

        for a in range(3):
            self.acc[a] += g[a]
            # The gyrocal.c overflow proof claims int32 suffices; verify it here.
            assert INT32_MIN <= self.acc[a] <= INT32_MAX, "int32 accumulator overflow"
        self.count += 1

        for a in range(3):
            if self.mx[a] - self.mn[a] > GYROCAL_MAX_SPREAD_LSB:
                self.acc = [0, 0, 0]
                self.count = 0
                self.restarts += 1
                return 0

        if self.count >= GYROCAL_SAMPLES:
            for a in range(3):
                self.bias[a] = avg_round(self.acc[a])
            self.done = True
            return 1
        return 0

    def apply(self, gx, gy, gz):
        return (sat_i16(gx - self.bias[0]),
                sat_i16(gy - self.bias[1]),
                sat_i16(gz - self.bias[2]))


def _run_to_done(cal, samples):
    """Feed a sequence; return the return value of the completing feed (or 0)."""
    done_edge = 0
    for (gx, gy, gz) in samples:
        r = cal.feed(gx, gy, gz)
        if r == 1:
            done_edge = 1
    return done_edge


def test_a_constant_bias_plus_noise():
    """(a) Constant bias + bounded zero-mean noise recovers the true bias
    within one LSB of rounding."""
    true_bias = (-812, 37, -5)     # -812 LSB ~ -6.2 dps at 131 LSB/dps (clone X)
    # Small deterministic zero-mean noise (|noise| <= 3 LSB, spread 6 << 400).
    noise_cycle = [0, 1, -1, 2, -2, 3, -3, 0, 1, -1]  # sums to 0 over 10
    cal = GyroCal()
    samples = []
    for i in range(GYROCAL_SAMPLES):
        nz = noise_cycle[i % len(noise_cycle)]
        samples.append((true_bias[0] + nz, true_bias[1] + nz, true_bias[2] + nz))
    edge = _run_to_done(cal, samples)
    ok = (edge == 1 and cal.done and cal.restarts == 0)
    for a in range(3):
        ok = ok and abs(cal.bias[a] - true_bias[a]) <= 1
    print(f"(a) constant bias +/- noise   -> bias={cal.bias} "
          f"true={list(true_bias)} restarts={cal.restarts} : "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def test_b_extremes_no_overflow():
    """(b) All samples at the int16 extremes: accumulate stays inside int32, the
    average equals the extreme, and apply() saturates rather than wrapping."""
    ok = True
    for extreme in (INT16_MAX, INT16_MIN):
        cal = GyroCal()
        edge = _run_to_done(cal, [(extreme, extreme, extreme)] * GYROCAL_SAMPLES)
        # int32 assertion inside feed() would have raised on overflow.
        ok = ok and (edge == 1)
        for a in range(3):
            ok = ok and cal.bias[a] == extreme
        # |sum| for INT16_MIN = 1000*32768 = 32,768,000 < 2^31-1 : confirm.
        expect_sum = extreme * GYROCAL_SAMPLES
        ok = ok and (INT32_MIN <= expect_sum <= INT32_MAX)

    # apply() saturation: a near-full-scale raw minus a bias must clamp, not wrap.
    cal = GyroCal()
    cal.bias = [-1000, 1000, 0]     # force a bias so subtraction overflows int16
    gx, gy, gz = cal.apply(32000, -32000, 5)
    ok = ok and gx == INT16_MAX     # 32000 - (-1000) = 33000 -> clamp 32767
    ok = ok and gy == INT16_MIN     # -32000 - 1000   = -33000 -> clamp -32768
    ok = ok and gz == 5
    print(f"(b) extremes + no overflow    -> sat=({gx},{gy},{gz}) : "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def test_c_negative_rounding_symmetry():
    """(c) Negative bias rounds symmetrically to its positive mirror; a true
    bias of -102 (bounded noise) recovers -102, not a systematic -101/-103."""
    # Property: avg_round(-s) == -avg_round(s) for arbitrary sums, incl. the .5
    # tie (e.g. +/-102500 -> +/-103, rounded away from zero symmetrically).
    ok = True
    for s in (0, 1, 499, 500, 501, 102000, 102500, 1499, 1500, 32768000):
        ok = ok and (avg_round(-s) == -avg_round(s))
    ok = ok and avg_round(102500) == 103 and avg_round(-102500) == -103
    ok = ok and avg_round(102000) == 102 and avg_round(-102000) == -102

    # End-to-end: 1000 samples averaging exactly to -102 recover -102 exactly.
    cal = GyroCal()
    noise_cycle = [0, 1, -1, 2, -2, 0, 1, -1, 2, -2]  # zero-mean over 10
    samples = [(-102 + noise_cycle[i % 10], 0, 0) for i in range(GYROCAL_SAMPLES)]
    _run_to_done(cal, samples)
    ok = ok and cal.bias[0] == -102
    print(f"(c) negative rounding symmetry -> bias_x={cal.bias[0]} "
          f"(expect -102) : {'PASS' if ok else 'FAIL'}")
    return ok


def test_d_spread_guard_restart():
    """(d) An injected step beyond the spread threshold restarts the window:
    restart counter increments and the sample count resets to 0."""
    cal = GyroCal()
    for _ in range(100):
        cal.feed(10, 10, 10)          # still window, count -> 100
    assert cal.count == 100 and cal.restarts == 0
    # Inject a step on X: spread 500-10 = 490 > 400 -> restart.
    cal.feed(500, 10, 10)
    ok = (cal.restarts == 1 and cal.count == 0 and cal.acc == [0, 0, 0])
    # A sub-threshold step (spread 300 <= 400) must NOT restart.
    cal.feed(10, 10, 10)
    cal.feed(310, 10, 10)             # spread 300 on X
    ok = ok and (cal.restarts == 1 and cal.count == 2)
    print(f"(d) spread guard restart       -> restarts={cal.restarts} "
          f"count={cal.count} : {'PASS' if ok else 'FAIL'}")
    return ok


def test_e_spread_on_last_sample_restarts():
    """(e) A spread violation ON the 1000th sample must RESTART, not complete:
    the guard is checked BEFORE the completion check (guard-before-completion)."""
    cal = GyroCal()
    for _ in range(GYROCAL_SAMPLES - 1):
        cal.feed(0, 0, 0)             # 999 still samples, min=max=0
    assert cal.count == GYROCAL_SAMPLES - 1 and not cal.done
    edge = cal.feed(500, 0, 0)        # 1000th: spread 500 > 400 on X
    ok = (edge == 0 and not cal.done and cal.count == 0 and cal.restarts == 1)
    print(f"(e) spread on 1000th -> restart -> done={cal.done} "
          f"count={cal.count} restarts={cal.restarts} : "
          f"{'PASS' if ok else 'FAIL'}")
    return ok


def test_f_exact_boundary_completes():
    """(f) Exact-boundary spread == GYROCAL_MAX_SPREAD_LSB (400) must NOT restart
    (guard is strict '>'); the window completes normally."""
    cal = GyroCal()
    edge = 0
    for i in range(GYROCAL_SAMPLES):
        # Values 0 and 400 -> min=0, max=400, spread exactly 400 (not > 400).
        v = 400 if (i % 2 == 0) else 0
        r = cal.feed(v, v, v)
        if r == 1:
            edge = 1
    ok = (edge == 1 and cal.done and cal.restarts == 0)
    print(f"(f) spread == 400 -> completes  -> done={cal.done} "
          f"restarts={cal.restarts} : {'PASS' if ok else 'FAIL'}")
    return ok


def main():
    results = [
        test_a_constant_bias_plus_noise(),
        test_b_extremes_no_overflow(),
        test_c_negative_rounding_symmetry(),
        test_d_spread_guard_restart(),
        test_e_spread_on_last_sample_restarts(),
        test_f_exact_boundary_completes(),
    ]
    passed = sum(1 for r in results if r)
    print(f"\n{passed}/{len(results)} tests PASS")
    return 0 if passed == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
