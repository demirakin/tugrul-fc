/*
 * TUGRUL flight controller — Milestone M5-c (independent watchdog)
 *
 * Thin wrapper around the STM32F446 IWDG (Independent Watchdog). The IWDG is a
 * free-running down-counter clocked by the dedicated LSI RC oscillator — a clock
 * source entirely separate from the SYSCLK/HSE tree. That independence is the
 * whole point: if the main PLL, the SysTick, or the superloop itself dies, the
 * IWDG keeps counting on the LSI and resets the chip. It guards against a HUNG
 * CONTROL LOOP, which no software-internal check can catch (a hung loop cannot
 * run its own watchdog).
 *
 * IRREVERSIBLE ONCE STARTED — READ THIS:
 *   The start sequence (KR=0xCCCC) has NO counterpart. There is no stop bit, no
 *   disable, no "kick to sleep". Once iwdg_init() runs, the counter runs until
 *   the next hardware reset, and it keeps running through most low-power modes
 *   (it only halts in Standby, and — via DBG_IWDG_STOP below — under an attached
 *   debugger). From that point on, EVERY code path that can execute for longer
 *   than the timeout MUST reach an iwdg_feed(), or the board resets. This is why
 *   iwdg_init() is the LAST boot step: nothing after it is allowed to block.
 *
 * TIMEOUT MATH (nominal + LSI tolerance):
 *   LSI nominal 32 kHz, prescaler /16 -> 2 kHz count clock. Reload RLR=999 gives
 *   1000 counts, so nominal timeout = 1000 / 2000 Hz = 500 ms.
 *   The LSI is an untrimmed RC oscillator with a wide spread over voltage/temp.
 *   Its exact F446 datasheet min/max are NOT yet confirmed (datasheet-agent debt);
 *   using a conservative envelope of ~17..48 kHz the real timeout spans
 *     fast LSI 48 kHz -> /16 = 3000 Hz   -> 1000/3000   ~= 333 ms (shortest)
 *     slow LSI 17 kHz -> /16 = 1062.5 Hz -> 1000/1062.5 ~= 941 ms (longest)
 *   These are ENVELOPE figures, deliberately rounded outward pending the real
 *   datasheet table. Feed cadence is ~1 ms (once per executed 1 kHz control tick),
 *   so even at the shortest ~333 ms window the margin is >300x worst-case — a
 *   healthy loop never comes close. A genuine hang is caught in under 1 s
 *   worst-case (<=941 ms).
 *
 * This unit owns the IWDG registers, the DBGMCU freeze bit for the IWDG, and the
 * reset-cause read/clear on RCC->CSR. It touches no other peripheral.
 */

#ifndef IWDG_H
#define IWDG_H

#include <stdint.h>

/*
 * Read the reset-cause flags from RCC->CSR, then clear ALL of them (RMVF), and
 * return the RAW CSR word captured BEFORE the clear (so the caller may inspect
 * other reset causes — POR/pin/software/window-watchdog — for logging).
 *
 * Test the return value with RCC_CSR_IWDGRSTF: nonzero in that bit == the last
 * reset was forced by THIS watchdog.
 *
 * BOOT-ORDER CONSTRAINT (must be documented and obeyed): call this exactly ONCE,
 * as early as possible at boot, BEFORE iwdg_init() and before anything else that
 * might latch or react to a reset cause. The reset-cause flags are STICKY across
 * resets: RCC->CSR retains IWDGRSTF until RMVF is written. If we never clear it,
 * the flag set by one IWDG reset would still read as set on every subsequent
 * (unrelated) boot, mis-attributing all future boots to the watchdog. Reading
 * and clearing once, up front, is the only correct point.
 */
uint32_t iwdg_read_and_clear_reset_cause(void);

/*
 * Run the full IWDG start sequence and arm the watchdog. IRREVERSIBLE (see the
 * header note): once this returns, the loop MUST keep feeding. Call as the LAST
 * boot step, after every other init that could block. Also freezes the IWDG
 * counter while a debugger has the core halted (DBG_IWDG_STOP), so single-step /
 * breakpoint sessions do not end in a reset storm; that bit is harmless with no
 * debugger attached.
 *
 * SEQUENCE ORDER (RM0390 IWDG configuration procedure — order is load-bearing):
 *   1) KR=0xCCCC  START. This also FORCE-ENABLES the LSI, so LSI edges begin to
 *                 flow; the PVU/RVU handshake in step 5 depends on that.
 *   2) KR=0x5555  enable write access to PR/RLR.
 *   3) write PR (/16), 4) write RLR (999), 5) spin until SR PVU/RVU clear,
 *   6) KR=0xAAAA  refresh the counter from the new reload.
 *   START MUST precede the PVU/RVU spin: issuing START last (LSI still off on a
 *   cold boot) would leave no LSI edges to advance the update logic and the spin
 *   would hang forever. Between step 1 and step 6 the dog runs briefly on its
 *   RESET DEFAULTS (PR=/4, RLR=0xFFF -> ~512 ms nominal); configuration takes
 *   only microseconds, orders of magnitude under that window, so there is no
 *   spurious-reset risk during the gap.
 */
void iwdg_init(void);

/*
 * Reload the counter (KR=0xAAAA) — the "kick". Call once per executed control
 * tick. Cheap: a single register write, no read, no handshake.
 */
void iwdg_feed(void);

#endif /* IWDG_H */
