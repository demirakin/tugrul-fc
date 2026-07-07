/*
 * TUGRUL flight controller — Milestone M5-c (independent watchdog)
 *
 * Implementation of the IWDG wrapper declared in iwdg.h. Pure register access,
 * no busy-wait beyond the short PVU/RVU handshake poll (bounded by LSI hardware,
 * not by us). See iwdg.h for the timeout math, the LSI-tolerance window, and the
 * IRREVERSIBLE / boot-order notes.
 *
 * Register-name cross-check (all from cmsis/stm32f446xx.h):
 *   IWDG->KR / PR / RLR / SR          IWDG_TypeDef
 *   IWDG_SR_PVU / IWDG_SR_RVU         SR update-in-progress flags
 *   IWDG_PR_PR_1 (== 0x2, PR[2:0]=010) prescaler divide-by-16
 *   RCC_CSR_IWDGRSTF / RCC_CSR_RMVF   reset-cause flag / remove-flags bit
 *   DBGMCU->APB1FZ / DBGMCU_APB1_FZ_DBG_IWDG_STOP  debug freeze bit
 */

#include "iwdg.h"
#include "stm32f446xx.h"

/* IWDG key-register magic values (RM0390, IWDG_KR). Not provided as named CMSIS
 * defines, so spelled out here with their meaning. */
#define IWDG_KEY_UNLOCK  0x5555u   /* enable write access to PR and RLR          */
#define IWDG_KEY_RELOAD  0xAAAAu   /* reload counter from RLR (the "feed" kick)   */
#define IWDG_KEY_START   0xCCCCu   /* start the watchdog (irreversible)           */

/* Reload value for a 500 ms nominal timeout at LSI 32 kHz /16 = 2 kHz:
 * 1000 counts -> RLR = 1000 - 1 = 999 (counter counts down and reloads on 0). */
#define IWDG_RELOAD_999  999u

uint32_t iwdg_read_and_clear_reset_cause(void)
{
    /* Capture the raw reset-cause word FIRST (before clearing) so the caller can
     * inspect any cause bit for logging. RCC is always clocked; safe this early. */
    uint32_t csr = RCC->CSR;   /* RCC->CSR: sticky reset-cause flags (IWDGRSTF etc.) */

    /* Clear ALL reset flags. Without RMVF the sticky IWDGRSTF would still read as
     * set on every LATER (unrelated) boot and mis-attribute it to the watchdog. */
    RCC->CSR |= RCC_CSR_RMVF;   /* RCC->CSR: RMVF (bit 24) clears all reset-cause flags */

    return csr;
}

void iwdg_init(void)
{
    /* Freeze the IWDG counter whenever a debugger has the core halted. Without
     * this, every breakpoint / single-step pause lets the LSI counter run out and
     * the chip resets under the debugger (a "reset storm" that makes debugging
     * impossible). Harmless when no debugger is attached — the bit only takes
     * effect while the core is in debug halt. Set BEFORE starting the watchdog. */
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;  /* DBGMCU_APB1FZ: DBG_IWDG_STOP (bit 12) */

    /* fixed: reordered to the RM0390 IWDG configuration procedure. START (0xCCCC)
     * MUST come FIRST — it force-enables the LSI, and the PVU/RVU handshake below
     * only advances once LSI edges are flowing. Issuing START last (the original
     * order) on a cold boot with LSI off left no clock to clear PVU/RVU, hanging
     * the spin forever. */

    IWDG->KR = IWDG_KEY_START;    /* IWDG_KR: 0xCCCC -> START (also force-enables LSI; irreversible) */

    /* Between START above and the 0xAAAA refresh below, the watchdog runs on its
     * RESET DEFAULTS (PR=/4, RLR=0xFFF -> ~512 ms nominal at 32 kHz LSI). The
     * reconfiguration that follows takes only microseconds — orders of magnitude
     * under that ~512 ms window — so there is no risk of a spurious reset during
     * the brief default-config gap. */

    IWDG->KR = IWDG_KEY_UNLOCK;   /* IWDG_KR: 0x5555 -> enable write access to PR/RLR */

    /* Prescaler /16: PR[2:0] = 010b == IWDG_PR_PR_1. 32 kHz LSI -> 2 kHz count. */
    IWDG->PR = IWDG_PR_PR_1;      /* IWDG_PR: prescaler divide-by-16 */

    /* Reload 999 -> 1000 counts -> 500 ms nominal (see iwdg.h timeout math). */
    IWDG->RLR = IWDG_RELOAD_999;  /* IWDG_RLR: 12-bit reload value */

    /* Wait for the PR/RLR writes to transfer into the VDD (LSI) voltage domain.
     * PR and RLR live in the low-speed VDD domain and are written from the APB1
     * (VDD core) domain across a clock-domain boundary; the SR PVU (prescaler
     * value update) and RVU (reload value update) flags stay 1 until the transfer
     * completes. Refreshing the counter (0xAAAA) before the transfer settles would
     * act on the stale reload, so we spin here. Because START above already
     * enabled the LSI, edges are flowing and this clears within a few LSI cycles
     * (a few hundred us); it can only hang if the LSI itself is dead — in which
     * case the watchdog could not run anyway. RM0390: IWDG_SR PVU/RVU. */
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0u) {
        /* spin until both update flags clear */
    }

    IWDG->KR = IWDG_KEY_RELOAD;   /* IWDG_KR: 0xAAAA -> refresh counter from the new reload */
}

void iwdg_feed(void)
{
    /* The kick: reload the down-counter from RLR. Single write, no handshake. */
    IWDG->KR = IWDG_KEY_RELOAD;   /* IWDG_KR: 0xAAAA -> reload counter (feed the dog) */
}
