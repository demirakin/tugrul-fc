/*
 * TUGRUL flight controller — Milestone M3-prep (MPU-9250 wake-up + raw burst)
 * Register-level, over the SPI1 driver in spi.c.
 *
 * SPI read protocol for the MPU-9250 (register-map datasheet, "SPI Interface"):
 *   - Every register access is a 2-byte frame while CS is held low.
 *   - Byte 0: the register address. Bit7 = R/W: 1 = read, 0 = write. So a read
 *     of register N sends (N | 0x80).
 *   - Byte 1: a dummy TX byte whose only job is to generate 8 more clocks; the
 *     slave drives the register contents onto MISO during that byte, which we
 *     capture as the RX byte.
 * Read  sequence:  CS low -> transfer(reg|0x80) [discard RX] -> transfer(dummy)
 *                  [RX = data] -> CS high.
 * Write sequence:  CS low -> transfer(reg) [discard RX] -> transfer(val)
 *                  [discard RX] -> CS high.
 * Burst read (multi-byte): CS low -> transfer(startreg|0x80) -> transfer(dummy)
 *                  N times (auto-increment on-chip) -> CS high.
 */

#include "mpu9250.h"
#include "spi.h"
#include "clock.h"          /* delay_ms() for the datasheet post-reset settle */

/*
 * Register/bit constants. Covers exactly what M3-prep touches (identity, power,
 * interface, basic config, raw data block) — nothing speculative beyond that.
 *   WHO_AM_I = 0x75 : device identity register (MPU-9250 register map §3).
 *   READ_FLAG = 0x80: bit7 of the address byte selects a read transfer.
 */
#define MPU_REG_WHO_AM_I    0x75u
#define MPU_READ_FLAG       0x80u

/*
 * Register map used by init/read (register-map doc RM-MPU-9250A-00). Only the
 * registers M3-prep touches — the rest stay out until they are needed.
 */
#define MPU_REG_SMPLRT_DIV      0x19u   /* sample-rate divider                    */
#define MPU_REG_CONFIG          0x1Au   /* DLPF / FSYNC config (gyro+temp DLPF)   */
#define MPU_REG_GYRO_CONFIG     0x1Bu   /* gyro full-scale select                 */
#define MPU_REG_ACCEL_CONFIG    0x1Cu   /* accel full-scale select                */
#define MPU_REG_ACCEL_CONFIG2   0x1Du   /* accel DLPF config                      */
#define MPU_REG_ACCEL_XOUT_H    0x3Bu   /* first byte of the 14-byte data block   */
/* MPU_REG_PWR_MGMT_1 (0x6B) lives in mpu9250.h: it doubles as the public
 * identity-echo discriminator, so main.c needs the named constant too.        */
#define MPU_REG_USER_CTRL       0x6Au   /* FIFO/I2C master/ I2C_IF_DIS control     */

/* Bit/value constants for the bring-up sequence. */
#define MPU_PWR1_H_RESET        0x80u   /* PWR_MGMT_1 bit7: full device reset      */
#define MPU_PWR1_CLKSEL_PLL     0x01u   /* PWR_MGMT_1 CLKSEL=1: auto PLL, SLEEP=0   */
#define MPU_USERCTRL_I2C_IF_DIS 0x10u   /* USER_CTRL bit4: disable I2C slave iface  */

/* Bring-up config values (see mpu9250_init() for the per-register rationale). */
#define MPU_CFG_SMPLRT_DIV      0x00u   /* divider 0 -> 1 kHz base sample rate      */
#define MPU_CFG_CONFIG          0x03u   /* DLPF_CFG=3 -> gyro/temp ~41 Hz bandwidth */
#define MPU_CFG_GYRO_CONFIG     0x00u   /* FS_SEL=0   -> ±250 dps, no self-test     */
#define MPU_CFG_ACCEL_CONFIG    0x00u   /* ACCEL_FS=0 -> ±2 g, no self-test         */
#define MPU_CFG_ACCEL_CONFIG2   0x03u   /* A_DLPFCFG=3 -> accel ~41 Hz bandwidth    */

/*
 * Sensitivity table used by mpu9250_scale(). These constants are bound ONE-TO-
 * ONE to the full-scale selects written just above: if you change MPU_CFG_
 * GYRO_CONFIG (FS_SEL) or MPU_CFG_ACCEL_CONFIG (ACCEL_FS), you MUST change this
 * table too, or the scaling silently drifts off. Sources cited per line.
 *   MPU9250_ACCEL_LSB_PER_G  16384 LSB/g at ±2 g  (product spec §3.2).
 *   MPU9250_GYRO_LSB_PER_DPS 131   LSB/dps at ±250 dps (product spec §3.1).
 *   MPU9250_TEMP_SENS_X100   333.87 LSB/°C -> stored ×100 as 33387 to stay
 *                            integer (product spec §3.4.2 / register-map
 *                            TEMP_OUT formula).
 *   MPU9250_TEMP_OFFSET_C100 21 °C room-temp offset, in °C×100 = 2100. The
 *                            RoomTemp_Offset term of the formula is 0.
 */
#define MPU9250_ACCEL_LSB_PER_G   16384
#define MPU9250_GYRO_LSB_PER_DPS  131
#define MPU9250_TEMP_SENS_X100    33387
#define MPU9250_TEMP_OFFSET_C100  2100

/* Size of the raw sensor block: accel(6) + temp(2) + gyro(6). */
#define MPU_BURST_LEN           14u

/* Dummy byte clocked out to read a byte back. Value is irrelevant on a read —
 * the slave ignores MOSI during the data phase — 0x00 is the conventional pick. */
#define MPU_DUMMY_TX        0x00u

/*
 * Accepted WHO_AM_I identities (all valid silicon we may see on this bus):
 *   0x71 = MPU-9250 (the real target),
 *   0x73 = MPU-9255 (pin/register compatible sibling),
 *   0x70 = MPU-6500 die (the 6-axis core inside the 9250 family; some clones /
 *          early parts report the bare die id),
 *   0x75 = clone die on MPU-9250-labeled breakouts (see MPU_ID_CLONE_75 below).
 * 0x75 is accepted ONLY after the PWR_MGMT_1 == 0x01 discriminator confirms a
 * real responder, because 0x75 is also the WHO_AM_I register address — a bus
 * that echoes register addresses would fake this id. The other three ids are
 * unambiguous and accepted directly.
 */
#define MPU_ID_9250         0x71u
#define MPU_ID_9255         0x73u
#define MPU_ID_6500         0x70u
/*
 * 0x75 = clone die sold on MPU-9250-labeled breakouts. Verified as a REAL
 * responder (not an address-echo artifact) via the PWR_MGMT_1 == 0x01 reset-
 * default discriminator on 2026-07-03 — note 0x75 is also the WHO_AM_I register
 * address, which is why the discriminator was needed. Its magnetometer (AK8963)
 * is most likely absent or non-functional; irrelevant for now since AK8963
 * support is deferred anyway.
 */
#define MPU_ID_CLONE_75     0x75u

mpu_probe_result_t mpu9250_probe(uint8_t *raw_id)
{
    /* Force the slow config bus for register access. M2 never uses DATA speed,
     * but calling this makes the probe self-contained and correct even if some
     * later code left the prescaler at 10.5 MHz. */
    spi1_set_speed(SPI_SPEED_CONFIG);

    /* Frame the whole 2-byte read with a single CS-low window. */
    spi1_cs_low();
    (void)spi1_transfer(MPU_REG_WHO_AM_I | MPU_READ_FLAG); /* address phase; RX here is stale, discard */
    uint8_t id = spi1_transfer(MPU_DUMMY_TX);              /* data phase; RX = WHO_AM_I contents */
    spi1_cs_high();

    /* Always surface the raw byte, whatever the verdict — the caller logs it. */
    *raw_id = id;

    /*
     * 0x00 and 0xFF are NOT valid ids and both mean "nobody drove MISO":
     *   - 0xFF: MISO idles high (our PB4 pull-up, or an open-drain/absent slave)
     *     so every read bit is 1 -> a stuck-high / disconnected line.
     *   - 0x00: MISO stuck low (no slave releasing it, or held at ground) so
     *     every read bit is 0.
     * With no sensor wired (M2-prep reality) we expect exactly one of these.
     * Treat them as an honest "no response" rather than a bogus id.
     */
    if ((id == 0x00u) || (id == 0xFFu)) {
        return MPU_NO_RESPONSE;
    }

    if ((id == MPU_ID_9250) || (id == MPU_ID_9255) || (id == MPU_ID_6500)) {
        return MPU_OK;
    }

    /*
     * 0x75 needs a second look: it is a real clone-die identity AND the WHO_AM_I
     * register's own address, so a bus that echoes register addresses would fake
     * it. Confirm a real responder before accepting — read PWR_MGMT_1 (reset
     * default 0x01) via the shared read path, still at CONFIG speed set above.
     * 0x01 => genuine clone die -> MPU_OK. Anything else (e.g. 0x6B, the echoed
     * address) => fall through to MPU_UNEXPECTED_ID so main.c's PM1 diagnostic
     * prints the echo pattern honestly.
     */
    if (id == MPU_ID_CLONE_75) {
        if (mpu9250_read_reg(MPU_REG_PWR_MGMT_1) == 0x01u) { /* 0x01 = PWR_MGMT_1 reset default */
            return MPU_OK;
        }
    }

    /* A plausible byte came back but it is not a family member — real silicon of
     * an unknown type, a wiring/bit-order fault, or bus noise. Flag it distinctly
     * so the log shows "unexpected" rather than a false OK. */
    return MPU_UNEXPECTED_ID;
}

/*
 * Single register write. CS framing follows the SPI write sequence: address
 * byte (bit7=0) then the value byte, both inside one CS-low window. CS is
 * ALWAYS released before returning — there is no early-return path, so the bus
 * can never be left with a slave selected (no CS leak). Caller sets bus speed.
 */
static void mpu_write_reg(uint8_t reg, uint8_t val)
{
    spi1_cs_low();
    (void)spi1_transfer(reg & (uint8_t)~MPU_READ_FLAG); /* addr phase, bit7=0 => write */
    (void)spi1_transfer(val);                           /* data phase: the value byte  */
    spi1_cs_high();
}

/*
 * Single register read. CS framing follows the SPI read sequence: (reg|0x80)
 * then a dummy byte whose RX is the register contents. CS is ALWAYS released
 * before returning (no early-return leak). Caller sets bus speed.
 */
static uint8_t mpu_read_reg(uint8_t reg)
{
    spi1_cs_low();
    (void)spi1_transfer(reg | MPU_READ_FLAG);   /* addr phase, bit7=1 => read; RX stale */
    uint8_t val = spi1_transfer(MPU_DUMMY_TX);  /* data phase: RX = register contents   */
    spi1_cs_high();
    return val;
}

/*
 * Public single-register read for diagnostics (e.g. the identity-echo probe).
 * Forces the slow CONFIG bus first, exactly like mpu9250_probe(), so the read
 * is self-contained; then reuses the shared CS + address-phase/data-phase
 * pattern above. CS is ALWAYS released before returning (no leak).
 */
uint8_t mpu9250_read_reg(uint8_t reg)
{
    spi1_set_speed(SPI_SPEED_CONFIG);   /* config-speed register access (<=1 MHz) */
    return mpu_read_reg(reg);
}

int mpu9250_init(void)
{
    /* All register traffic here is config traffic: force the slow (<=1 MHz)
     * bus. mpu9250_read_raw() is the only thing that ever raises the speed. */
    spi1_set_speed(SPI_SPEED_CONFIG);

    /* a) Presence gate. Reuse the WHO_AM_I probe: if no accepted id answers,
     *    there is nothing to bring up — resetting/writing a device that is not
     *    there just burns 100 ms and reports false progress. Fail fast (1). */
    uint8_t raw_id = 0u;
    if (mpu9250_probe(&raw_id) != MPU_OK) {
        return 1;   /* stage 1: no accepted WHO_AM_I */
    }

    /* b) Full reset. PWR_MGMT_1 H_RESET (bit7) restores all registers to their
     *    power-on defaults, clearing whatever state a prior run / warm reset
     *    left behind. The datasheet specifies a device start-up time after
     *    reset (accel start-up dominates, up to ~30-100 ms depending on part);
     *    100 ms is the conservative bound so every register below writes onto a
     *    settled, out-of-reset device. delay_ms() is bounded (SysTick-based). */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, MPU_PWR1_H_RESET); /* 0x6B <- 0x80: H_RESET */
    delay_ms(100u);

    /* c) Wake + best clock. After reset the device is asleep on the internal
     *    20 MHz oscillator. CLKSEL=1 (auto-select PLL, falls back to the
     *    internal osc if the gyro PLL is not ready) is preferred over CLKSEL=0:
     *    the PLL clock is lower-jitter and more stable than the bare internal
     *    oscillator. Writing 0x01 also clears SLEEP (bit6), waking the device. */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, MPU_PWR1_CLKSEL_PLL); /* 0x6B <- 0x01: PLL, awake */

    /* d) SPI-only operation. On the 9250 the SPI and I2C slave share the same
     *    pads; with I2C still enabled the I2C state machine can contend on those
     *    pads and corrupt SPI traffic. The datasheet requires setting
     *    USER_CTRL.I2C_IF_DIS (bit4) to disable the I2C slave interface for
     *    SPI-only use. H_RESET cleared this bit, so it must be re-set here. */
    mpu_write_reg(MPU_REG_USER_CTRL, MPU_USERCTRL_I2C_IF_DIS); /* 0x6A <- 0x10: I2C off */

    /* e) Prove the writes are landing. Over SPI a write is fire-and-forget —
     *    read-back is our ONLY evidence that MOSI/MISO/CLK and the slave all
     *    work end to end. Expect the CLKSEL=PLL value we just wrote; anything
     *    else means the bus is not carrying writes reliably. Fail (2). */
    uint8_t pwr1 = mpu_read_reg(MPU_REG_PWR_MGMT_1); /* 0x6B read-back */
    if (pwr1 != MPU_PWR1_CLKSEL_PLL) {
        return 2;   /* stage 2: PWR_MGMT_1 read-back mismatch */
    }

    /* f) Bring-up config defaults. Written once, NOT verified — read-back on
     *    every one of these would double the traffic for little gain now that
     *    step (e) already proved the write path. Rationale per register:
     *      SMPLRT_DIV=0     -> divider off; output data rate = 1 kHz base.
     *      CONFIG=0x03      -> DLPF_CFG=3, gyro/temp bandwidth ~41 Hz. Tradeoff:
     *                          heavier low-pass = less noise but ~5.9 ms group
     *                          delay; fine for bring-up, revisit for the M4/M5
     *                          control loop where phase lag costs stability.
     *      GYRO_CONFIG=0x00 -> ±250 dps, the most sensitive range (best counts
     *                          per dps) for a bench that is not being slewed.
     *      ACCEL_CONFIG=0x00-> ±2 g, likewise the most sensitive accel range.
     *      ACCEL_CONFIG2=0x03-> accel DLPF ~41 Hz, matching the gyro path. */
    mpu_write_reg(MPU_REG_SMPLRT_DIV,    MPU_CFG_SMPLRT_DIV);    /* 0x19 <- 0x00 */
    mpu_write_reg(MPU_REG_CONFIG,        MPU_CFG_CONFIG);        /* 0x1A <- 0x03 */
    mpu_write_reg(MPU_REG_GYRO_CONFIG,   MPU_CFG_GYRO_CONFIG);   /* 0x1B <- 0x00 */
    mpu_write_reg(MPU_REG_ACCEL_CONFIG,  MPU_CFG_ACCEL_CONFIG);  /* 0x1C <- 0x00 */
    mpu_write_reg(MPU_REG_ACCEL_CONFIG2, MPU_CFG_ACCEL_CONFIG2); /* 0x1D <- 0x03 */

    return 0;   /* device awake, SPI-only, configured */
}

int mpu9250_read_raw(mpu_raw_t *out)
{
    uint8_t buf[MPU_BURST_LEN];

    /* Sensor-register burst is allowed up to 20 MHz — use DATA speed (10.5 MHz)
     * to clock 14 bytes quickly. One CS-low window covers the whole burst: send
     * the start address with the read flag, then clock the block out; the device
     * auto-increments its internal register pointer for each dummy byte. */
    spi1_set_speed(SPI_SPEED_DATA);

    spi1_cs_low();
    (void)spi1_transfer(MPU_REG_ACCEL_XOUT_H | MPU_READ_FLAG); /* addr phase; RX stale */
    uint8_t all_ff = 0xFFu;     /* AND-accumulate: stays 0xFF iff every byte is 0xFF */
    uint8_t all_00 = 0x00u;     /* OR-accumulate:  stays 0x00 iff every byte is 0x00 */
    for (uint32_t i = 0u; i < MPU_BURST_LEN; i++) {
        uint8_t b = spi1_transfer(MPU_DUMMY_TX);    /* data phase: next block byte */
        buf[i]  = b;
        all_ff &= b;
        all_00 |= b;
    }
    spi1_cs_high();

    /* Policy: config speed is the resting default. Data reads are not the hot
     * path until M4+, so drop back to the conservative <=1 MHz bus between
     * samples rather than leaving the fast prescaler latched. */
    spi1_set_speed(SPI_SPEED_CONFIG);

    /* Dead-bus guard. With no sensor (or a lost one) MISO parks high (0xFF) or
     * low (0x00) for the entire burst. Either all-ones or all-zeros means no
     * device drove the block — reject WITHOUT touching *out so the caller keeps
     * its last good sample and can re-probe. (all_ff==0xFF => every byte 0xFF;
     * all_00==0x00 => every byte 0x00.) */
    if ((all_ff == 0xFFu) || (all_00 == 0x00u)) {
        return 1;   /* burst all-0xFF or all-0x00: bus dead / sensor gone */
    }

    /*
     * Assemble big-endian byte pairs into signed 16-bit counts. The cast chain
     * is deliberate and implementation-defined-behaviour-free:
     *   (uint16_t)hi << 8 | lo   builds the value in UNSIGNED 16-bit space, so
     *                            the left shift never touches a sign bit (no UB
     *                            from shifting into/over a signed sign bit),
     *   (int16_t)(...)           then reinterprets the 16-bit pattern as signed.
     * Converting an out-of-range unsigned to a signed type is implementation-
     * defined (not UB) and on arm-none-eabi (two's complement) yields exactly
     * the intended two's-complement value.
     */
    out->ax   = (int16_t)((uint16_t)((uint16_t)buf[0]  << 8) | buf[1]);
    out->ay   = (int16_t)((uint16_t)((uint16_t)buf[2]  << 8) | buf[3]);
    out->az   = (int16_t)((uint16_t)((uint16_t)buf[4]  << 8) | buf[5]);
    out->temp = (int16_t)((uint16_t)((uint16_t)buf[6]  << 8) | buf[7]);
    out->gx   = (int16_t)((uint16_t)((uint16_t)buf[8]  << 8) | buf[9]);
    out->gy   = (int16_t)((uint16_t)((uint16_t)buf[10] << 8) | buf[11]);
    out->gz   = (int16_t)((uint16_t)((uint16_t)buf[12] << 8) | buf[13]);

    return 0;
}

/*
 * Integer divide with round-half-away-from-zero, sign-aware. 'den' is ALWAYS a
 * positive constant from the sensitivity table, so only 'num' carries a sign.
 *   positive num: (num + den/2) / den
 *   negative num: (num - den/2) / den
 * Truncation (plain '/') was rejected on purpose: C integer division truncates
 * toward zero, which biases every negative result up and every positive result
 * down — an asymmetric bias around zero. That bias would survive into the M4
 * attitude stage, where samples straddling zero are averaged, and show up as a
 * DC offset. Symmetric rounding keeps the estimator honest.
 * Constant-time: the only branch is on the sign of 'num', no value-dependent
 * loops. Overflow-safe for our inputs — see mpu9250_scale() worst-case notes.
 */
static int32_t div_round(int32_t num, int32_t den)
{
    return (num >= 0) ? (num + den / 2) / den
                      : (num - den / 2) / den;
}

void mpu9250_scale(const mpu_raw_t *in, mpu_scaled_t *out)
{
    /*
     * Overflow discipline: every raw field is int16 (|v| <= 32768). Each is
     * widened to int32 BEFORE the multiply so the product lives in int32 space:
     *   accel: 32768 * 1000  =  32,768,000   (well inside int32 ±2.147e9)
     *   gyro:  32768 * 100    =   3,276,800
     *   temp:  32768 * 10000  = 327,680,000
     * div_round adds den/2 (<= 16693) before dividing; that sum stays inside
     * int32 too (327,680,000 + 16,693 = 327,696,693). No intermediate overflows.
     */

    /* accel -> mg: ax_mg = ax_raw * 1000 / 16384  (16384 LSB/g, product spec §3.2). */
    out->ax_mg = div_round((int32_t)in->ax * 1000, MPU9250_ACCEL_LSB_PER_G);
    out->ay_mg = div_round((int32_t)in->ay * 1000, MPU9250_ACCEL_LSB_PER_G);
    out->az_mg = div_round((int32_t)in->az * 1000, MPU9250_ACCEL_LSB_PER_G);

    /* gyro -> centi-dps: g_cdps = g_raw * 100 / 131  (131 LSB/dps, product spec §3.1). */
    out->gx_cdps = div_round((int32_t)in->gx * 100, MPU9250_GYRO_LSB_PER_DPS);
    out->gy_cdps = div_round((int32_t)in->gy * 100, MPU9250_GYRO_LSB_PER_DPS);
    out->gz_cdps = div_round((int32_t)in->gz * 100, MPU9250_GYRO_LSB_PER_DPS);

    /*
     * temp -> centi-°C. Register-map formula: T_°C = TEMP_OUT/333.87 + 21
     * (RoomTemp_Offset = 0). In °C×100 that is TEMP_OUT * 100/333.87 + 2100.
     * Scaling trick: 100/333.87 = 10000/33387 EXACTLY, so multiply by 10000 and
     * divide by 33387 to keep the whole path in integers with no precision loss
     * from a pre-rounded constant. The +2100 offset is exact (no rounding), so
     * only the division is rounded.
     */
    out->t_c100 = div_round((int32_t)in->temp * 10000, MPU9250_TEMP_SENS_X100)
                  + MPU9250_TEMP_OFFSET_C100;
}
