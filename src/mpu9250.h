/*
 * TUGRUL flight controller — Milestone M3-prep (MPU-9250 wake-up + raw burst)
 *
 * Presence check + device bring-up + raw sensor burst read + fixed-point
 * scaling for the MPU-9250 IMU over SPI1. Layers, in order of trust:
 *   - mpu9250_probe():    minimal WHO_AM_I presence check (M2, unchanged).
 *   - mpu9250_init():     reset, wake, disable I2C slave, load bring-up config.
 *   - mpu9250_read_raw(): one 14-byte accel/temp/gyro burst, raw signed int16.
 *   - mpu9250_scale():    pure integer conversion to mg / centi-dps / centi-degC
 *                         (no float — deterministic integer path).
 *
 * M3 scope guard: NO calibration, NO filtering, NO magnetometer (AK8963
 * deferred). Raw counts stay available alongside scaled values (evidence trail).
 *
 * The board may not be wired (or may not have arrived) yet: every layer reports
 * an honest failure when no sensor answers, never a fabricated reading. init
 * failures are stage-coded so a caller can retry on the next heartbeat, and a
 * mid-run read failure is distinguishable so the caller can re-probe (hot-plug).
 *
 * Requires spi1_init() to have run first.
 */

#ifndef MPU9250_H
#define MPU9250_H

#include <stdint.h>

/*
 * Verdict of a WHO_AM_I probe:
 *   MPU_OK            raw id is one of the accepted family values
 *   MPU_UNEXPECTED_ID bus answered with a plausible byte, but not a known id
 *   MPU_NO_RESPONSE   raw id is 0x00 or 0xFF -> floating/stuck MISO, no sensor
 * The raw byte is always returned via the out-param regardless of verdict.
 */
typedef enum {
    MPU_OK = 0,
    MPU_UNEXPECTED_ID,
    MPU_NO_RESPONSE
} mpu_probe_result_t;

/* Read WHO_AM_I over SPI1 at CONFIG speed. Writes the raw byte to *raw_id
 * (must be non-NULL) and returns the verdict above. */
mpu_probe_result_t mpu9250_probe(uint8_t *raw_id);

/*
 * PWR_MGMT_1 register address (power management 1). Reset default 0x01. Exposed
 * here (not private to mpu9250.c) because it doubles as the identity-echo
 * discriminator: WHO_AM_I's address (0x75) can echo back on a broken bus, so a
 * live read of this register's known 0x01 default separates a genuine die from a
 * bus that merely echoes the register address. mpu9250_read_reg() is the
 * intended accessor.
 */
#define MPU_REG_PWR_MGMT_1  0x6Bu

/*
 * Read a single register over SPI1 at CONFIG speed and return its byte. Uses
 * the same CS + address-phase/data-phase framing as mpu9250_probe(). Intended
 * for diagnostics such as the identity-echo discriminator (reading PWR_MGMT_1's
 * known reset default to tell a genuine die from a bus that echoes addresses).
 */
uint8_t mpu9250_read_reg(uint8_t reg);

/*
 * One raw sample of the 14-byte sensor block (ACCEL_XOUT_H..GYRO_ZOUT_L). All
 * fields are RAW signed 16-bit counts straight off the wire — NO scaling, NO
 * unit conversion (M3-prep scope). Order matches the on-chip register layout:
 * accel XYZ, then temperature, then gyro XYZ.
 */
typedef struct {
    int16_t ax, ay, az;     /* ACCEL_{X,Y,Z}OUT, raw counts (±2 g at init cfg) */
    int16_t temp;           /* TEMP_OUT, raw counts (die temperature)          */
    int16_t gx, gy, gz;     /* GYRO_{X,Y,Z}OUT, raw counts (±250 dps at init)  */
} mpu_raw_t;

/*
 * One raw sample converted to human/control units by pure integer fixed-point
 * arithmetic — NO float anywhere. The FPU is present, but at this stage the
 * deterministic-latency narrative prefers an integer path: float in an ISR
 * would open lazy-stacking cost discussions, and scaling must stay constant-
 * time. Units are chosen so no fractional type is ever needed:
 *   accel -> mg    (milli-g,   1 g = 1000 mg)
 *   gyro  -> cdps  (centi-dps, 1 dps = 100 counts, i.e. dps×100)
 *   temp  -> c100  (centi-°C,  1 °C = 100 counts, i.e. °C×100)
 * Valid ONLY for the ±2 g / ±250 dps config mpu9250_init() writes; the LSB
 * constants live next to ACCEL_CONFIG/GYRO_CONFIG in mpu9250.c.
 */
typedef struct {
    int32_t ax_mg, ay_mg, az_mg;    /* accel, milli-g                          */
    int32_t t_c100;                 /* die temperature, °C×100                  */
    int32_t gx_cdps, gy_cdps, gz_cdps; /* gyro, dps×100                        */
} mpu_scaled_t;

/*
 * Convert a raw sample to fixed-point units. Pure function: no I/O, no state,
 * no hardware, constant-time (no value-dependent branches beyond rounding sign).
 * *in and *out must be non-NULL; *out is fully written.
 */
void mpu9250_scale(const mpu_raw_t *in, mpu_scaled_t *out);

/*
 * Reset, wake, and configure the device for bring-up (runs at CONFIG speed).
 * Returns 0 on success, or a nonzero STAGE code identifying where it gave up:
 *   1 = WHO_AM_I not an accepted id (nothing to bring up; fail fast)
 *   2 = post-wake PWR_MGMT_1 read-back mismatch (writes are not landing)
 * A nonzero return is normal when no sensor is wired — the caller retries.
 */
int mpu9250_init(void);

/*
 * Read one raw 14-byte burst into *out (must be non-NULL) at DATA speed, then
 * restore CONFIG speed. Returns 0 on success. Returns nonzero WITHOUT touching
 * *out if the whole burst is all-0xFF or all-0x00 (dead bus / sensor gone),
 * so a caller can distinguish "sensor lost" from a valid sample.
 */
int mpu9250_read_raw(mpu_raw_t *out);

#endif /* MPU9250_H */
