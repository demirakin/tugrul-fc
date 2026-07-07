/*
 * TUGRUL flight controller — Milestone M5-prep (failsafe skeleton)
 *
 * Sensor-freshness watchdog. It answers exactly one question: has a GOOD IMU
 * burst arrived recently enough to trust the attitude estimate? If freshness
 * lapses, it drives the PWM outputs to their safe state and LATCHES there.
 *
 * TWO STATES ONLY (hard scope limit — no third state at M5-prep):
 *   FS_ARMED     freshness is current; control is allowed to run.
 *   FS_FAILSAFE  freshness lapsed at least once; PWM forced safe, LATCHED.
 *
 * GUARDS LOSS, NOT ABSENCE: the watchdog protects against losing a source that
 * was ALREADY alive. The timeout is NOT evaluated until the FIRST successful
 * fs_feed() (first proof of life); until then fs_tick() stays ARMED and cannot
 * trip. This is essential because sensor bring-up happens at heartbeat pace
 * (~1 s) — far past FS_TIMEOUT_MS — so an active timeout seeded at boot would
 * latch FAILSAFE on every normal boot (a real live-board defect). A source that
 * never comes alive is already covered by construction: pwm_init() loaded the
 * safe values before the counter started, and the heartbeat reports "no
 * response" honestly — no latch needed to keep the outputs safe.
 *
 * OUTPUT PROTECTION (not just the state flag): while latched, fs_tick()
 * RE-ASSERTS the three safe PWM values on EVERY tick, not only on the trip edge.
 * The latch must protect the actual outputs — once a future writer (M5
 * stabilization) commands PWM every tick, it would otherwise silently overwrite
 * the forced-safe CCRs. Re-asserting makes the failsafe self-healing against any
 * such writer with no gate needed at each call site; the three preload CCR
 * writes are idempotent and cheap. ORDERING INVARIANT this relies on: fs_tick()
 * must run LAST among PWM writers within a control tick — a writer placed after
 * it would land after the re-assert and defeat the latch for that tick. Keep
 * fs_tick() at the end of the tick when integrating the M5 stabilization loop.
 *
 * FROZEN AGE: fs_age_ms() reports the LIVE freshness age while ARMED, but the
 * age FROZEN at the trip edge while latched. A sensor that recovers may still be
 * fed (harmless), which would otherwise shrink a live age and print a
 * contradictory "fs=FAILSAFE age=2ms"; freezing keeps the log self-consistent.
 *
 * TIME BASE: free-running uint32_t millisecond stamps (SysTick millis()). All
 * age math uses wraparound-safe unsigned subtraction (uint32_t)(now - last),
 * so the 32-bit ms counter's ~49.7-day wrap is handled correctly (the modular
 * difference stays valid across the wrap as long as the true elapsed time is
 * under 2^31 ms — always true for a control-loop freshness window).
 *
 * LATCH RATIONALE:
 *   Losing the IMU invalidates the attitude estimate itself — the thing the
 *   whole control law is built on. Unlike an RC-link dropout (where auto-resume
 *   the instant the signal returns is the conventional, defensible policy),
 *   silently resuming control from an IMU that just came back — without
 *   revalidating that its data is sane and the filter is re-seeded — is NOT
 *   defensible: you would hand full authority back to an estimate of unknown
 *   quality. Therefore failsafe LATCHES; re-arming is a deliberate reset
 *   (reboot), never automatic. The separate question of what to do on RC-command
 *   loss is an M5-stabilization policy decision, not part of this watchdog.
 *
 * This unit owns no hardware directly; it calls pwm_set_us() (pwm.c) to command
 * the safe state and reuses the exported PWM safe-state constants so it commands
 * the exact values pwm_init() starts at (single source of truth).
 */

#ifndef FAILSAFE_H
#define FAILSAFE_H

#include <stdint.h>

/* Watchdog states — two only, by design (see header note). */
typedef enum {
    FS_ARMED    = 0,
    FS_FAILSAFE = 1
} fs_state_t;

/*
 * Freshness timeout. 100 ms == 100 lost samples at the 1 kHz control rate:
 * generous versus a single-read glitch (one dropped burst never trips it), yet
 * small versus any airframe dynamics we would actually care about at this stage.
 * Tunable at M5 stabilization.
 */
#define FS_TIMEOUT_MS 100u

/* Start ARMED. Records the state and an informational freshness seed from the
 * boot time 'now_ms'; staleness-guarding does not actually begin here but at the
 * first fs_feed() (see GUARDS LOSS, NOT ABSENCE above). */
void fs_init(uint32_t now_ms);

/* Record a fresh, GOOD sensor burst at 'now_ms'. Call ONLY on a genuinely
 * successful read — never on a stale or failed one. Feeding AFTER the latch is
 * harmless and intentionally ignored for reporting (see FROZEN AGE above). */
void fs_feed(uint32_t now_ms);

/* Run one watchdog step; call every control tick. In ARMED, and only AFTER the
 * first feed, if the freshness age has reached FS_TIMEOUT_MS, transition to
 * FAILSAFE (latched); before the first feed the timeout is not evaluated. In
 * FAILSAFE, RE-ASSERT the three safe PWM values every tick (output protection,
 * see above). Returns the current state. */
fs_state_t fs_tick(uint32_t now_ms);

/* Freshness age in ms for logging: 0 before the first feed (no source yet), the
 * LIVE age (uint32_t)(now_ms - last_good) while ARMED and alive, or the age
 * FROZEN at the trip edge while latched in FAILSAFE. */
uint32_t fs_age_ms(uint32_t now_ms);

#endif /* FAILSAFE_H */
