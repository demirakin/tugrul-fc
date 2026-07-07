/**
 * TUGRUL flight controller — Milestone M5-prep (attitude + control-loop skeleton)
 * ----------------------------------------------------------------------------
 * Brings up the 84 MHz clock tree, the USART2 console, and the SPI1 master, then
 * runs TWO cadences on the SysTick millisecond base:
 *
 *   1 kHz CONTROL TICK (every elapsed millisecond, with catch-up):
 *     - toggles PA8 once per EXECUTED update iteration (the jitter probe / M5
 *       scope hook; D7 on the Arduino header);
 *     - reads one raw MPU-9250 burst per loop pass and advances the integer
 *       attitude filter (roll/pitch, complementary + a parallel gyro-only track)
 *       by the number of elapsed milliseconds.
 *
 *   1 Hz HEARTBEAT:
 *     - toggles the Nucleo-F446RE user LED (LD2, PA5);
 *     - when no sensor is up: prints the WHO_AM_I probe and attempts init;
 *     - when up: prints raw + scaled evidence lines (from the cached sample) and
 *       an attitude line (filtered R/P alongside the gyro-only Rg/Pg drift track),
 *       or "attitude: stale (no sensor)" when there is no live estimate.
 *
 * JITTER SOURCE RESOLVED (M5-prep): the heartbeat's UART writes used to BLOCK for
 * several ms at 115200, stalling this single-threaded loop. TX is now non-blocking
 * (uart.c: interrupt-driven 512-byte ring; USART2_IRQHandler drains it), so the
 * heartbeat enqueues its lines in microseconds and returns. The control-tick
 * catch-up loop below is therefore expected to fire PA8 catch-up BURSTS only under
 * genuine stalls now (e.g. a real long-running section), not on every heartbeat —
 * so a clean PA8 square wave is the normal, hardened-loop signature.
 *
 * The MPU-9250 breakout may not be wired yet (board may not have arrived). Every
 * step reports honestly: "no response" with no sensor, a real id the moment it
 * is plugged in, init retried until it succeeds, and a lost sensor caught on the
 * next read — so the heartbeat doubles as a live wiring / hot-plug indicator.
 *
 * No HAL, no RTOS. Register access goes through the official ST CMSIS-Device
 * header (see cmsis/stm32f446xx.h), which only provides the address layout —
 * every write below is deliberate and explicit.
 *
 * Clocking: clock_init() brings us from the reset-default 16 MHz HSI up to
 * 84 MHz off the HSE (bypass, ST-LINK MCO). If that fails we stay on HSI and
 * flash a fast panic pattern so the failure is visible on the bench.
 */

#include "stm32f446xx.h"
#include "clock.h"
#include "uart.h"
#include "spi.h"
#include "mpu9250.h"
#include "attitude.h"
#include "pwm.h"
#include "failsafe.h"
#include "gyrocal.h"
#include "iwdg.h"

/*
 * Control-loop catch-up clamp. If the single-threaded loop stalls (the blocking
 * heartbeat UART is the M4-prep culprit) for more than this many milliseconds,
 * we do NOT fabricate that many 1 ms integration steps from one stale sample —
 * we resync honestly (snap to the accel tilt). 50 ms is generous versus the few
 * ms a heartbeat print costs, yet short enough that the constant-rate assumption
 * behind catch-up stays defensible.
 */
#define CONTROL_CATCHUP_MAX_MS  50u

/**
 * Crude busy-wait, iteration-count based (NOT time). Used ONLY for the panic
 * blink below: if clock_init() failed we cannot trust the SysTick time base
 * (wrong frequency, or the clock never came up), so delay_ms() is off-limits
 * there and a raw spin is the honest choice. 'volatile' stops the compiler
 * from deleting the empty loop.
 */
static void panic_delay(volatile uint32_t count)
{
    while (count != 0u) {
        count--;
    }
}

/**
 * Print a signed 32-bit value in decimal. uart.h only offers an unsigned
 * decimal print and we may NOT edit uart.*, so wrap it: emit a '-' for negative
 * inputs and print the magnitude via uart_write_dec().
 *
 * INT32_MIN edge: negating INT32_MIN (-2147483648) overflows int32_t (UB), so
 * the usual v = -v trick is unsafe in general. Here it CANNOT occur — every
 * caller feeds a physically bounded value: raw samples are int16_t widened to
 * int32_t (|v| <= 32768), and scaled values are bounded by the sensor ranges
 * (+/-16 g -> +/-16000 mg, +/-2000 dps -> +/-200000 cdps, temp ~ +/-8500 c°C),
 * all orders of magnitude below |INT32_MIN|, so the negation is well defined.
 */
static void print_sdec(int32_t v)
{
    if (v < 0) {
        uart_write_byte((uint8_t)'-');
        v = -v;                     /* safe: callers are physically bounded, see above */
    }
    uart_write_dec((uint32_t)v);
}

int main(void)
{
    /* 0) Bring the core up to 84 MHz. Clock only — SysTick is started later
     *    (step 4), after the panic branch, so the panic path never depends
     *    on a timer that may not be running. */
    int clk_err = clock_init();

    /* 0b) Read AND clear the reset-cause flags immediately, before anything else
     *     can care about them. This is placed even before the panic branch and
     *     before GPIO/UART bring-up on purpose: RCC->CSR is always clocked, the
     *     read has no dependency, and the flags are STICKY across resets — if the
     *     last reset was an IWDG timeout, IWDGRSTF stays set until we write RMVF,
     *     so leaving it would mis-attribute EVERY later boot to the watchdog. We
     *     stash the raw CSR here and print the human line later, once UART is up
     *     (a "reset cause: IWDG watchdog" banner line is only meaningful on the
     *     console). See iwdg.h boot-order constraint. */
    uint32_t reset_csr = iwdg_read_and_clear_reset_cause();

    /* 1) Feed a clock to GPIO port A. A peripheral with no clock is dead
     *    silicon: writes to its registers simply do nothing. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* 1b) Read RCC->AHB1ENR back before touching GPIOA. On STM32F4 the
     *     clock-enable write can lag a few cycles; without this read-back the
     *     very next GPIOA access may hit the peripheral before its clock is
     *     live, dropping the one-shot MODER write and leaving the LED dark.
     *     'volatile' + discarded value forces the read and stalls until it
     *     completes; the compiler cannot elide it. */
    volatile uint32_t ahb1enr_readback = RCC->AHB1ENR;
    (void)ahb1enr_readback;

    /* 2) Make PA5 a general-purpose output.
     *    MODER holds 2 bits per pin: 00=input, 01=output, 10=alt-fn, 11=analog.
     *    First clear both bits for pin 5, then set the low bit -> 01 (output). */
    GPIOA->MODER &= ~GPIO_MODER_MODER5;
    GPIOA->MODER |=  GPIO_MODER_MODER5_0;

    /* 2b) Make PA8 a general-purpose output: the control-loop jitter probe.
     *     It toggles once per EXECUTED tick that produces a good sensor read —
     *     inside the mpu_ready sub-branches only (calibration feed, filter seed,
     *     resync, or one edge per catch-up iteration), NOT on every loop pass. So
     *     a scope on PA8 shows the true executed-tick edges (and honest catch-up
     *     bursts) — the M5 jitter-evidence hook. PA8 is Arduino header pin D7 on
     *     the Nucleo-F446RE. Same MODER recipe as PA5: clear both bits for pin 8,
     *     set the low bit -> 01. */
    GPIOA->MODER &= ~GPIO_MODER_MODER8;
    GPIOA->MODER |=  GPIO_MODER_MODER8_0;

    /* 3) If the clock never reached 84 MHz, do NOT proceed to the normal blink
     *    (its timing would be wrong and SysTick may be dead). Instead flash a
     *    fast panic pattern forever on HSI, driven by a raw busy-wait — the one
     *    place a busy-wait is legitimate, because we explicitly cannot rely on
     *    the SysTick time base here. */
    if (clk_err != 0) {
        for (;;) {
            GPIOA->ODR ^= GPIO_ODR_OD5;   /* toggle PA5 */
            panic_delay(200000u);         /* ~fast blink @ 16 MHz HSI */
        }
    }

    /* 4) Clock is confirmed at 84 MHz: start the 1 ms SysTick time base. This
     *    is intentionally past the panic branch — SysTick is only meaningful
     *    once HCLK is the known 84 MHz that its 84000-count reload assumes. */
    systick_init();

    /* 4b) Console up. uart_init() depends on the confirmed 42 MHz PCLK1 for its
     *     baud divisor, so it lives here — past the panic branch, never on the
     *     clock-failure path (a wrong SYSCLK would give a wrong baud and garbage
     *     on the terminal, so on panic we stay silent rather than lie). */
    uart_init();
    uart_write_str("TUGRUL M5-prep boot - SYSCLK 84 MHz\n");

    /* 4b') Reset-cause banner. The raw CSR was captured+cleared at step 0b; only
     *      now, with the console alive, can we surface it. Print the IWDG line
     *      ONLY when this boot was caused by a watchdog timeout (the interesting,
     *      alarming case). A normal power-on / pin reset stays quiet to avoid
     *      noise. This line is also the acceptance evidence for the deliberate
     *      hang test (see IWDG_TEST_HANG below): after the forced hang, the board
     *      resets and this line proves the watchdog fired. */
    if ((reset_csr & RCC_CSR_IWDGRSTF) != 0u) {
        uart_write_str("reset cause: IWDG watchdog\n");
    }

    /* 4c) SPI1 master up (PB3/4/5 AF5 + PB6 CS), at CONFIG speed. Depends on the
     *     confirmed 84 MHz APB2 for its prescaler math, so it also lives past
     *     the panic branch. No sensor is required for init to succeed. */
    spi1_init();
    uart_write_str("SPI1 up - probing MPU-9250 WHO_AM_I @656kHz\n");

    /* 4d) Servo PWM up (TIM3, 50 Hz, PA6/PA7/PB0 AF2), started at safe defaults
     *     (surfaces 1500 us neutral, throttle 1000 us idle). Scope-verifiable
     *     with no servo/ESC attached. NOTE: there is deliberately NO attitude->PWM
     *     mapping in this build — turning the attitude estimate into surface
     *     commands is the stabilization-loop unit's job (scope guard). */
    pwm_init();
    uart_write_str("PWM up - TIM3 50Hz on PA6/PA7/PB0 (1500/1500/1000 us)\n");

    /* 4e) Sensor-freshness failsafe watchdog. Boots ARMED with an informational
     *     freshness seed. Staleness-guarding only arms at the FIRST good read
     *     (first fs_feed), NOT at boot: sensor bring-up runs at heartbeat pace
     *     (~1 s) >> the 100 ms timeout, so an active-at-boot timeout would latch
     *     FAILSAFE on every boot. If the MPU never comes up, the watchdog stays
     *     ARMED (nothing alive to lose) and the outputs are already safe from
     *     pwm_init() — see failsafe.h "GUARDS LOSS, NOT ABSENCE". */
    fs_init(millis());
    uart_write_str("Failsafe ARMED - IMU freshness watchdog (100 ms)\n");

    /* 4f) Gyro bias boot calibration. Boots NOT-CALIBRATED: the control tick will
     *     average GYROCAL_SAMPLES (~1 s at 1 kHz) good gyro samples with the board
     *     held STILL, then subtract that raw-LSB bias from every later sample
     *     before scaling/attitude. Motivation: the clone MPU carries a large
     *     X-axis zero-rate offset (~ -6.2 dps) that otherwise dominates the drift
     *     track. Attitude is NOT seeded/advanced until this completes (no filter
     *     update on an uncalibrated gyro). Board must be stationary at boot. */
    gyrocal_init();
    uart_write_str("Gyrocal NOT-CALIBRATED - hold board STILL (~1 s)\n");

    /* 4g) Independent watchdog. INTENTIONALLY THE LAST init step. Once iwdg_init()
     *     runs the IWDG is armed and IRREVERSIBLE (no stop bit): from here on the
     *     superloop MUST execute and feed within the timeout window or the chip
     *     resets. Arming it last guarantees every other bring-up (which may block
     *     — SPI probes, UART) has already completed, so nothing between here and
     *     the loop can trip it. It guards the CONTROL LOOP's liveness, not the
     *     sensor: sensor loss is failsafe's job (see feed note in the loop). */
    iwdg_init();
    uart_write_str("IWDG armed - 500 ms nominal (~333-941 ms w/ LSI tolerance)\n");

    /* MPU-9250 lifecycle flag. 0 = not brought up yet (probe + init at heartbeat
     * pace); 1 = init succeeded (read + attitude update at 1 kHz). A read failure
     * clears it back to 0 so the next heartbeat re-probes — honest hot-plug. */
    int mpu_ready = 0;

    /* Attitude bookkeeping:
     *   seeded         - the first good sample after (re)init has seeded the
     *                    filter via attitude_reset(); further samples update it.
     *   attitude_valid - the estimate reflects a live sensor. Cleared the moment
     *                    the sensor is lost so the heartbeat prints "stale"
     *                    instead of a frozen (lying) attitude.
     *   have_sample    - a good scaled sample is cached for the heartbeat to
     *                    print (raw + scaled evidence lines). */
    int seeded         = 0;
    int attitude_valid = 0;
    int have_sample    = 0;
    mpu_raw_t    last_raw = {0};   /* most recent good raw burst (evidence trail) */
    mpu_scaled_t last_sc  = {0};   /* most recent good scaled sample              */

    /* Failsafe bookkeeping (mirrors fs_state so the heartbeat can print without
     * calling fs_tick itself):
     *   fs_state    - last state returned by fs_tick(); starts ARMED to match
     *                 fs_init(). Read by the heartbeat status line.
     *   fs_announce - one-shot flag: set on the ARMED->FAILSAFE edge in the
     *                 (lean) 1 kHz tick, consumed by the 1 Hz heartbeat which
     *                 prints the "FAILSAFE ENTERED" line once (no UART in the
     *                 control tick beyond nothing — keep it lean).
     *   fs_entry_ms - millis() at that transition, for the one-shot message. */
    fs_state_t fs_state    = FS_ARMED;
    int        fs_announce = 0;
    uint32_t   fs_entry_ms = 0u;

    /* Gyrocal one-shot: set on the calibration-completion edge in the 1 kHz tick,
     * consumed by the 1 Hz heartbeat which prints the "gyrocal done" line exactly
     * once (mirrors the fs_announce pattern; keeps the control tick UART-free). */
    int gyrocal_announce = 0;

    /* 5) Two cadences on the SysTick millisecond base, both non-blocking:
     *      - a 1 kHz CONTROL TICK (every new millis value) that owns the PA8
     *        jitter probe, the sensor read, and the attitude update;
     *      - a 1 Hz HEARTBEAT that owns the LD2 LED, probe/init when no sensor is
     *        up, and all console printing.
     *    'last_ms' detects a new millisecond; 'next' schedules the heartbeat with
     *    the same wraparound-immune unsigned-subtraction idiom delay_ms uses, so
     *    the 32-bit millis() wrap (every ~49.7 days) is handled correctly. */
    uint32_t last_ms = millis();          /* last millisecond the control tick ran */
    uint32_t next    = millis() + 1000u;  /* first heartbeat one second from now    */
    uint32_t uptime  = 0u;                /* whole seconds since boot               */

    for (;;) {
        uint32_t now = millis();

#ifdef IWDG_TEST_HANG
        /* ===================================================================
         * ##  THIS MUST NEVER SHIP ENABLED  ##  THIS MUST NEVER SHIP ENABLED ##
         * ===================================================================
         * DELIBERATE HANG — watchdog acceptance hook ONLY. Compile-gated, default
         * OFF; enabled solely via `make TEST_HANG=1`. At t=10 s it announces, then
         * enters an infinite loop that NEVER feeds the IWDG. Expected result: the
         * board resets within one timeout window (<=941 ms worst-case) and the
         * next boot prints "reset cause: IWDG watchdog" — end-to-end proof the
         * watchdog is live. The drain wait below is acceptable BECAUSE we are
         * about to hang forever anyway: delay_ms() still works here (SysTick keeps
         * ticking until the reset), it just lets the announce line reach the
         * terminal before the counter runs out.
         * ================================================================== */
        if (now > 10000u) {
            uart_write_str("TEST HANG: deliberate infinite loop - expect IWDG reset <=1 s\n");
            delay_ms(100u);          /* let the line drain before we stop feeding */
            for (;;) {
                /* no iwdg_feed() here on purpose: the IWDG must time out and reset */
            }
        }
#endif /* IWDG_TEST_HANG */

        /* ---- 1 kHz control tick with CATCH-UP. 'elapsed' is the number of whole
         * milliseconds since the tick last ran, via wraparound-safe unsigned
         * subtraction. It is normally 1, but jumps to several when the blocking
         * heartbeat UART stalls this single-threaded loop — so we must advance the
         * integration by that many 1 ms steps, not just one, or the estimate
         * under-integrates. The sensor is read ONCE per loop pass; the integration
         * then iterates over that one sample (constant-rate assumption across the
         * short gap). Blocking UART is the known jitter source at M4-prep;
         * non-blocking / deferred TX is the M5 loop-hardening task. */
        uint32_t elapsed = now - last_ms;
        if (elapsed != 0u) {
            last_ms = now;

            if (mpu_ready) {
                /* One raw burst per loop pass. NOTE: mpu9250_read_raw() flips the
                 * SPI prescaler DATA->CONFIG on every call — acceptable overhead
                 * for now (loop hardening deferred to M5). A read failure = sensor
                 * lost: drop to not-ready, mark attitude stale (honest hot-unplug),
                 * re-probe at heartbeat pace. */
                mpu_raw_t s;
                if (mpu9250_read_raw(&s) == 0) {
                    /* GENUINELY FRESH, SUCCESSFUL burst — this is the ONLY place
                     * a good read is confirmed (read_raw() returns nonzero on a
                     * dead/all-0xFF/all-0x00 bus without touching *s). Feed the
                     * watchdog FIRST so stale/failed reads can never refresh it.
                     * Freshness is independent of calibration — fs_feed stays here
                     * unconditionally, before the calibration gate below. */
                    fs_feed(now);

                    /* Snapshot the calibration state ONCE for this tick: gyrocal_feed
                     * below may flip it true mid-tick (on the completing sample), but
                     * that final uncalibrated sample must NOT also drive attitude —
                     * the first CALIBRATED sample (next tick) seeds the filter. */
                    int calibrating = !gyrocal_done();

                    if (calibrating) {
                        /* CALIBRATING: accumulate the raw gyro bias. Attitude is
                         * deliberately NOT seeded or advanced (no filter update on
                         * an uncalibrated gyro — the whole point of this unit).
                         * 'seeded' stays 0, so the FIRST calibrated sample seeds
                         * the filter cleanly with no stale seed to redo. One
                         * executed tick still toggles PA8 so the jitter probe keeps
                         * a continuous trace during the ~1 s calibration window. */
                        if (gyrocal_feed(s.gx, s.gy, s.gz)) {
                            gyrocal_announce = 1;   /* completion edge -> heartbeat */
                        }
                        GPIOA->ODR ^= GPIO_ODR_OD8;   /* PA8: one executed tick */
                    } else {
                        /* CALIBRATED: remove the stored raw-LSB bias in place,
                         * BEFORE scaling / attitude use (saturating to int16). */
                        gyrocal_apply(&s.gx, &s.gy, &s.gz);
                    }

                    /* Scale ONCE: the bias-corrected raw when calibrated, or the
                     * raw sample while calibrating (so the pre-cal bias stays
                     * visible in the heartbeat evidence lines). */
                    mpu_scaled_t sc;
                    mpu9250_scale(&s, &sc);

                    if (!calibrating) {
                        if (!seeded) {
                            /* First calibrated sample after init/completion: snap
                             * the filter to it. One executed tick -> one PA8 edge. */
                            attitude_reset(&sc);
                            seeded         = 1;
                            attitude_valid = 1;
                            GPIOA->ODR ^= GPIO_ODR_OD8;   /* PA8: one executed tick */
                        } else if (elapsed > CONTROL_CATCHUP_MAX_MS) {
                            /* Pathological stall: refuse to fabricate 50+ integration
                             * steps from a single sample. Snap (resync) to the accel
                             * tilt instead — the gyro continuity across the gap is
                             * genuinely lost, so this is the honest choice. Resets
                             * both tracks (the gyro-only drift race restarts too). */
                            attitude_reset(&sc);
                            attitude_valid = 1;
                            GPIOA->ODR ^= GPIO_ODR_OD8;   /* PA8: one executed tick */
                            uart_write_str("ATT resync (loop stall > 50 ms)\n");
                        } else {
                            /* Catch-up: one 1 ms filter tick per elapsed ms. PA8
                             * toggles once per EXECUTED iteration, so the scope shows
                             * the true (bursty) execution pattern — the catch-up burst
                             * trailing a blocking heartbeat print is HONEST jitter
                             * evidence, not hidden behind an even square wave. */
                            for (uint32_t k = 0u; k < elapsed; k++) {
                                GPIOA->ODR ^= GPIO_ODR_OD8;   /* PA8 per executed tick */
                                attitude_update(&sc);
                            }
                        }
                    }

                    /* Cache for the heartbeat's evidence lines. */
                    last_raw    = s;
                    last_sc     = sc;
                    have_sample = 1;
                } else {
                    mpu_ready      = 0;
                    seeded         = 0;
                    attitude_valid = 0;
                    have_sample    = 0;
                    uart_write_str("MPU read FAIL (sensor lost?)\n");
                }
            }

            /* Failsafe watchdog: runs EVERY executed control tick regardless of
             * sensor state (fed above only on a genuine success). Detect the
             * ARMED->FAILSAFE rising edge here and defer the announcement to the
             * heartbeat — the 1 kHz tick stays lean (no UART). fs_tick() itself
             * forces the PWM safe state on that edge (once, latched). */
            fs_state_t fs_new = fs_tick(now);
            if (fs_new == FS_FAILSAFE && fs_state == FS_ARMED) {
                fs_entry_ms = now;   /* timestamp for the one-shot heartbeat line */
                fs_announce = 1;
            }
            fs_state = fs_new;

            /* Feed the independent watchdog ONCE per EXECUTED control tick — here,
             * inside the 'elapsed != 0' block, NOT once per catch-up iteration and
             * NOT from the raw main loop. Rationale for THIS placement:
             *   - Feeding from the executed tick ties the watchdog to the control
             *     loop's actual heartbeat. If SysTick dies, 'elapsed' is forever 0,
             *     this block never runs, feeds stop, and the IWDG resets the board
             *     (correct — a dead time base means dead control).
             *   - Feeding from the raw main loop instead would keep feeding even
             *     while the control tick is starved, letting a zombie loop spin
             *     forever with control dead — exactly what the watchdog must catch.
             *   - Once per pass, not per catch-up iteration: a single feed proves
             *     this pass ran; the catch-up count is irrelevant to liveness.
             * INTERPLAY: heartbeat / UART / sensor loss do NOT gate this feed. The
             * IWDG guards the control loop's liveness; sensor freshness is the
             * failsafe watchdog's job (fs_feed above). A lost sensor keeps feeding
             * the IWDG (loop is alive) while failsafe latches the outputs safe —
             * two independent guards, two independent responsibilities. */
            iwdg_feed();
        }

        /* ---- 1 Hz heartbeat: LED + (probe/init | evidence lines) + attitude. */
        if ((int32_t)(now - next) >= 0) {
            next += 1000u;                /* schedule next tick (no drift accrual) */
            uptime++;

            GPIOA->ODR ^= GPIO_ODR_OD5;   /* toggle the PA5 (LD2) output level */

            if (!mpu_ready) {
                /* Not brought up yet: run the live WHO_AM_I probe (1 Hz @ 656 kHz
                 * is harmless) as a wiring indicator, and try to init once the
                 * device answers with an accepted id. */
                uint8_t raw_id = 0u;
                mpu_probe_result_t verdict = mpu9250_probe(&raw_id);

                /* Probe line: "t=<sec>s WHO_AM_I=0x<id> <verdict>". */
                uart_write_str("t=");
                uart_write_dec(uptime);
                uart_write_str("s WHO_AM_I=0x");
                uart_write_hex8(raw_id);
                uart_write_byte((uint8_t)' ');
                switch (verdict) {
                    case MPU_OK:
                        /* Name the specific family member from the raw id. */
                        if (raw_id == 0x71u) {
                            uart_write_str("OK (MPU-9250)");
                        } else if (raw_id == 0x73u) {
                            uart_write_str("OK (MPU-9255)");
                        } else if (raw_id == 0x75u) {
                            /* Clone die (verified real via PWR_MGMT_1==0x01);
                             * must read distinctly from a genuine 0x71 part. */
                            uart_write_str("OK (clone die)");
                        } else {
                            uart_write_str("OK (MPU-6500 die)");
                        }
                        break;
                    case MPU_NO_RESPONSE:
                        uart_write_str("no response (sensor absent or wiring)");
                        break;
                    case MPU_UNEXPECTED_ID:
                    default:
                        uart_write_str("unexpected ID");
                        /* The 0x75 echo-ambiguity is now resolved inside
                         * mpu9250_probe itself (it accepts 0x75 only after
                         * PWR_MGMT_1 reads its 0x01 reset default). This branch
                         * therefore serves any OTHER unexpected id, plus a 0x75
                         * that FAILED that discriminator — reading PWR_MGMT_1 here
                         * surfaces the echo pattern (0x6B) honestly in the log:
                         *   0x6B -> read echoed the address -> bus artifact
                         *   0x01 -> device drove its real reset default -> real die
                         *   else -> inconclusive. */
                        {
                            uint8_t pm1 = mpu9250_read_reg(MPU_REG_PWR_MGMT_1);
                            uart_write_str(" PM1=0x");
                            uart_write_hex8(pm1);
                            if (pm1 == 0x6Bu) {
                                uart_write_str(" (echo artifact?)");
                            } else if (pm1 == 0x01u) {
                                uart_write_str(" (real die)");
                            } else {
                                uart_write_str(" (unknown)");
                            }
                        }
                        break;
                }
                uart_write_byte((uint8_t)'\n');

                /* Device present: attempt bring-up. On success the next control
                 * tick seeds the filter. On failure, report the stage code and
                 * stay not-ready so the next heartbeat retries. */
                if (verdict == MPU_OK) {
                    int stage = mpu9250_init();
                    if (stage == 0) {
                        mpu_ready = 1;
                        seeded    = 0;    /* let the next good read seed the filter */
                        /* If the sensor was lost DURING calibration, restart the
                         * window: partial sums from a re-powered sensor are not
                         * trustworthy (offset can shift across a power cycle).
                         * A COMPLETED calibration is kept — the spec scopes the
                         * restart to in-progress windows only. gyrocal_init() also
                         * clears the restart counter, an honest fresh slate. */
                        if (!gyrocal_done()) {
                            gyrocal_init();
                        }
                        uart_write_str("MPU-9250 init OK\n");
                    } else {
                        uart_write_str("MPU init FAIL stage ");
                        uart_write_dec((uint32_t)stage);
                        uart_write_byte((uint8_t)'\n');
                    }
                }
            } else if (have_sample) {
                /* Brought up and at least one good sample cached (captured by the
                 * 1 kHz tick). Emit the raw and scaled evidence lines from the
                 * cache — NOT a fresh read, so the print cannot perturb the
                 * control-tick cadence. */

                /* Line 1 — RAW signed counts (unscaled ground truth). */
                uart_write_str("t=");
                uart_write_dec(uptime);
                uart_write_str("s A=");
                print_sdec((int32_t)last_raw.ax);
                uart_write_byte((uint8_t)',');
                print_sdec((int32_t)last_raw.ay);
                uart_write_byte((uint8_t)',');
                print_sdec((int32_t)last_raw.az);
                uart_write_str(" G=");
                print_sdec((int32_t)last_raw.gx);
                uart_write_byte((uint8_t)',');
                print_sdec((int32_t)last_raw.gy);
                uart_write_byte((uint8_t)',');
                print_sdec((int32_t)last_raw.gz);
                uart_write_str(" T=");
                print_sdec((int32_t)last_raw.temp);
                uart_write_byte((uint8_t)'\n');

                /* Line 2 — SCALED fixed-point units (mg / centi-dps / centi-°C),
                 * pure integer, printed as plain integers (no decimal point). */
                uart_write_str("   -> A[mg]=");
                print_sdec(last_sc.ax_mg);
                uart_write_byte((uint8_t)',');
                print_sdec(last_sc.ay_mg);
                uart_write_byte((uint8_t)',');
                print_sdec(last_sc.az_mg);
                uart_write_str(" G[cdps]=");
                print_sdec(last_sc.gx_cdps);
                uart_write_byte((uint8_t)',');
                print_sdec(last_sc.gy_cdps);
                uart_write_byte((uint8_t)',');
                print_sdec(last_sc.gz_cdps);
                uart_write_str(" T[C100]=");
                print_sdec(last_sc.t_c100);
                uart_write_byte((uint8_t)'\n');
            }

            /* Line 3 — ATTITUDE. Filtered roll/pitch alongside the pure-gyro-
             * integrated track (the M4 drift comparison), all in centi-degrees.
             * If no live estimate exists, say so honestly rather than print a
             * frozen value. */
            if (attitude_valid) {
                int32_t r_cd, p_cd, rg_cd, pg_cd;
                attitude_get(&r_cd, &p_cd);
                attitude_get_gyro_only(&rg_cd, &pg_cd);
                uart_write_str("   -> R=");
                print_sdec(r_cd);
                uart_write_str(" P=");
                print_sdec(p_cd);
                uart_write_str(" (Rg=");
                print_sdec(rg_cd);
                uart_write_str(" Pg=");
                print_sdec(pg_cd);
                uart_write_str(")\n");
            } else if (mpu_ready && !gyrocal_done()) {
                /* Sensor IS present and sampling, but attitude is intentionally
                 * held off until gyro-bias calibration completes — distinguish
                 * this from a genuinely absent sensor so the log is honest. */
                uart_write_str("   -> attitude: calibrating (gyro bias)\n");
            } else {
                uart_write_str("   -> attitude: stale (no sensor)\n");
            }

            /* Line 3b — GYROCAL. While calibrating, a per-second progress line.
             * On the completion edge, the one-shot "done" line (bias in raw LSB +
             * restart count), printed exactly once via the gyrocal_announce flag
             * the 1 kHz tick set — mirrors the FAILSAFE ENTERED pattern. After
             * completion, neither line prints (calibration is silent). */
            if (gyrocal_announce) {
                gyrocal_announce = 0;
                uart_write_str("   -> gyrocal done: bias=");
                print_sdec((int32_t)gyrocal_bias(0));
                uart_write_byte((uint8_t)',');
                print_sdec((int32_t)gyrocal_bias(1));
                uart_write_byte((uint8_t)',');
                print_sdec((int32_t)gyrocal_bias(2));
                uart_write_str(" LSB (restarts=");
                uart_write_dec(gyrocal_restarts());
                uart_write_str(")\n");
            } else if (!gyrocal_done()) {
                uart_write_str("   -> gyrocal ");
                uart_write_dec(gyrocal_progress());
                uart_write_byte((uint8_t)'/');
                uart_write_dec(GYROCAL_SAMPLES);
                uart_write_str(" (restarts=");
                uart_write_dec(gyrocal_restarts());
                uart_write_str(")\n");
            }

            /* Line 4 — PWM. Read the three CCRs back (aileron/elevator/throttle)
             * so the heartbeat shows the live commanded pulse widths. In this
             * build these stay at the safe defaults (no attitude->PWM mapping
             * yet); once the stabilization loop drives pwm_set_us(), this line
             * becomes the command trail. */
            uart_write_str("   -> pwm A=");
            uart_write_dec(pwm_get_us(PWM_AILERON));
            uart_write_str(" E=");
            uart_write_dec(pwm_get_us(PWM_ELEVATOR));
            uart_write_str(" T=");
            uart_write_dec(pwm_get_us(PWM_THROTTLE));
            uart_write_byte((uint8_t)'\n');

            /* Line 5a — FAILSAFE one-shot: printed exactly once, the first
             * heartbeat after the watchdog tripped. The 1 kHz tick set the flag
             * and captured the entry time; the print lives here to keep the
             * control tick lean. */
            if (fs_announce) {
                fs_announce = 0;
                uart_write_str("   -> FAILSAFE ENTERED at t=");
                uart_write_dec(fs_entry_ms);
                uart_write_str("ms\n");
            }

            /* Line 5b — FAILSAFE status: current state + freshness age every
             * heartbeat. 'fs_state' mirrors the last fs_tick() result; the age
             * is computed live with the wraparound-safe helper. */
            uart_write_str("   -> fs=");
            uart_write_str((fs_state == FS_FAILSAFE) ? "FAILSAFE" : "ARMED");
            uart_write_str(" age=");
            uart_write_dec(fs_age_ms(now));
            uart_write_str("ms\n");

            /* TX-ring honesty line: report dropped console bytes ONCE per second,
             * but only when nonzero so a healthy link stays quiet (no noise). A
             * nonzero count means a heartbeat burst outran the 512-byte ring —
             * evidence the console is oversubscribed, never silently lost. */
            uint32_t drops = uart_get_dropped();
            if (drops > 0u) {
                uart_write_str("   -> uart drops=");
                uart_write_dec(drops);
                uart_write_byte((uint8_t)'\n');
            }
        }
        /* else: nothing due yet — fall through and re-poll millis(). */
    }
}
