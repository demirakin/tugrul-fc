/*
 * TUGRUL flight controller — Milestone M5-prep (failsafe skeleton)
 *
 * Implementation of the sensor-freshness watchdog declared in failsafe.h.
 * Pure integer, no busy-wait, no hardware of its own — it only reads a caller-
 * supplied millisecond stamp and commands pwm_set_us() on the transition.
 *
 * See failsafe.h for the two-state model, the latch rationale, and the
 * wraparound-safe time-base note.
 */

#include "failsafe.h"
#include "pwm.h"        /* pwm_set_us + exported PWM_US_NEUTRAL / PWM_US_IDLE */

/* Watchdog state. Module-private (single instance for the whole vehicle). */
static fs_state_t s_state          = FS_ARMED;
static uint32_t   s_last_good_ms   = 0u;   /* stamp of the most recent GOOD burst  */
static uint32_t   s_age_at_trip_ms = 0u;   /* freshness age frozen at the trip edge */
static int        s_ever_fed       = 0;    /* 1 after the FIRST good feed (see note) */

void fs_init(uint32_t now_ms)
{
    /* Boot ARMED. Staleness-guarding does NOT begin here — it begins at the
     * FIRST successful fs_feed() (s_ever_fed). The watchdog guards the LOSS of a
     * previously-alive source; it must not trip on a source that has not lived
     * yet. Sensor bring-up happens at heartbeat pace (~1 s), which is far past
     * the 100 ms timeout, so seeding an active timeout here would latch FAILSAFE
     * on every normal boot (live-board defect, GDB-confirmed). The seed below is
     * therefore only informational until the first feed overwrites it. A
     * never-alive source is already covered by construction: pwm_init() loaded
     * the safe values (1500/1500/1000) before the counter started, and the
     * heartbeat honestly reports "no response". */
    s_state        = FS_ARMED;
    s_last_good_ms = now_ms;   /* informational until the first fs_feed() */
    s_ever_fed     = 0;        /* no proof of life yet -> timeout not evaluated */
}

void fs_feed(uint32_t now_ms)
{
    /* Record freshness. This only moves the stamp forward; it never changes
     * state. The latch is enforced entirely in fs_tick(), so a stray feed can
     * never un-latch a tripped failsafe — even if the sensor recovers, feeding
     * resets s_last_good_ms but fs_tick() will not return from FAILSAFE. Such a
     * post-latch feed is harmless and intentionally IGNORED for reporting:
     * fs_age_ms() returns the frozen trip-age while latched, so the log never
     * shows a contradictory "fs=FAILSAFE age=2ms" after a recovered sensor. */
    s_last_good_ms = now_ms;
    s_ever_fed     = 1;   /* first proof of life: from now on the timeout is armed */
}

fs_state_t fs_tick(uint32_t now_ms)
{
    if (s_state == FS_ARMED) {
        /* Guard only a source that has ALREADY proven life. Before the first
         * feed there is nothing to "lose", so the timeout is not evaluated and
         * no trip is possible — otherwise the ~1 s sensor bring-up (>> 100 ms)
         * would latch FAILSAFE on every boot. */
        if (s_ever_fed) {
            /* Wraparound-safe age (see header). '>=' so exactly FS_TIMEOUT_MS trips. */
            uint32_t age = (uint32_t)(now_ms - s_last_good_ms);
            if (age >= FS_TIMEOUT_MS) {
                s_state          = FS_FAILSAFE;   /* latch; never returns to ARMED */
                s_age_at_trip_ms = age;           /* freeze reported age at the edge */
            }
        }
    }

    if (s_state == FS_FAILSAFE) {
        /* RE-ASSERT the safe state on EVERY latched tick, not just the edge. The
         * latch must protect the OUTPUTS, not merely the state variable: the
         * moment a future writer (M5 stabilization) starts commanding PWM every
         * tick, it would otherwise silently overwrite these CCRs while we still
         * report fs=FAILSAFE. Re-asserting here makes the failsafe self-healing
         * against any such writer without needing a gate at every call site.
         * Cost is three register writes via the CCR preload shadow — idempotent
         * and cheap; the surface/ESC never twitches (glitch-free at update event).
         * Constants are the SAME ones pwm_init() starts at, exported from pwm.h
         * (single source of truth, no duplicated literals). */
        pwm_set_us(PWM_AILERON,  PWM_US_NEUTRAL);   /* surface centre */
        pwm_set_us(PWM_ELEVATOR, PWM_US_NEUTRAL);   /* surface centre */
        pwm_set_us(PWM_THROTTLE, PWM_US_IDLE);      /* ESC idle       */
    }

    return s_state;
}

uint32_t fs_age_ms(uint32_t now_ms)
{
    if (s_state == FS_FAILSAFE) {
        /* Frozen at the trip edge: report the age that CAUSED the failsafe, not
         * a live one. A recovered sensor feeding after the latch would otherwise
         * make this shrink and contradict the latched state in the log. */
        return s_age_at_trip_ms;
    }
    if (!s_ever_fed) {
        /* No source yet, nothing to age — the heartbeat may show fs=ARMED
         * age=0ms during the first second, before the first good feed. */
        return 0u;
    }
    /* ARMED and alive: live, wraparound-safe modular difference — valid across
     * the ~49.7-day ms wrap. */
    return (uint32_t)(now_ms - s_last_good_ms);
}
