# MPU-9250 breakout ↔ Nucleo-F446RE wiring (SPI1)

Reference for the bench hookup the firmware expects. Pin choices are fixed
project decisions (see CLAUDE.md): SPI1 lives on PB3/PB4/PB5 because the
default PA5 SCK would conflict with the LD2 user LED, and CS is a plain GPIO
(PB6) because the MPU-9250 wants per-transaction CS framing.

## Connection table

| MPU-9250 breakout pin | Signal (SPI role)      | Nucleo pin | Arduino header |
|-----------------------|------------------------|------------|----------------|
| VCC                   | 3.3 V supply           | 3V3        | CN6 "3V3"      |
| GND                   | ground                 | GND        | CN6 "GND"      |
| SCL / SCLK            | SPI clock (SCK)        | PB3        | D3             |
| SDA / SDI             | master out (MOSI)      | PB5        | D4             |
| AD0 / SDO             | master in (MISO)       | PB4        | D5             |
| NCS                   | chip select (manual)   | PB6        | D10            |
| INT                   | — not used at M3       | (none)     | —              |
| FSYNC                 | — tie to GND if the breakout leaves it floating | GND | — |
| EDA / ECL (aux I2C)   | — not used (AK8963 deferred) | (none) | —          |

Cross-check the Arduino-header positions against UM1724 (Nucleo-64 user
manual) for your board revision before soldering anything.

## Notes and traps

- **Voltage: 3.3 V only.** The MPU-9250 core is a 2.4–3.6 V part. Most
  breakouts tie VDD and VDDIO together at the VCC pin, which is exactly what
  we want at 3.3 V. Do NOT feed 5 V unless your specific breakout documents
  an onboard regulator AND level shifting — and even then 3V3 is the safer,
  simpler choice (the Nucleo's 3V3 rail can source it comfortably).
- **AD0/SDO dual role.** In I2C mode AD0 selects the slave address; in SPI
  mode the same pad is SDO, i.e. the sensor's data output → our MISO (PB4).
  No pull or strap needed: as soon as the firmware talks SPI (NCS toggling),
  the pad is an output. The firmware also disables the chip's I2C slave
  interface (USER_CTRL.I2C_IF_DIS) at init, per the datasheet, so the pad
  can never be re-interpreted as an address strap.
- **NCS must actually move.** The MPU-9250 samples its interface mode from
  NCS activity; a permanently-low CS is not "always selected", it can wedge
  the interface. The firmware frames every transaction (CS low → bytes →
  CS high), so just wire it — don't strap it to GND.
- **Keep leads short (~10 cm or less).** WHO_AM_I traffic runs at 656 kHz,
  but M3 burst reads run at 10.5 MHz; jumper-wire rat's nests that pass at
  1 MHz start ringing at 10 MHz. Twist or run SCK away from MISO if the
  read-back looks flaky, and make sure GND is a solid single run.
- **Expected behavior once wired:** the 1 Hz heartbeat line switches from
  `WHO_AM_I=0xFF no response` to `WHO_AM_I=0x71 OK (MPU-9250)` (or 0x73 /
  0x70 for the -9255 / -6500-die variants — the firmware accepts and logs
  all three) within one second of plugging in. No reset or reflash needed;
  the probe is live every heartbeat.
- **INT stays unwired at M3.** Data-ready interrupts come with the control
  loop work (M4+); the heartbeat polls. FSYNC unused: ground it if your
  breakout doesn't already — datasheet practice for unused inputs. (With our
  CONFIG value EXT_SYNC_SET=0, FSYNC is ignored anyway; grounding is cheap
  insurance, not a functional requirement.)
