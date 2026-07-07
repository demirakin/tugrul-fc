# Axis conventions — MPU-9250 body frame ↔ aircraft NED (draft / template)

Status: **TEMPLATE — final mapping is decided at mounting time.** The sensor is
not mounted yet, so the chip→aircraft rotation below is a placeholder. What is
fixed already: the conventions, the rules any mapping must satisfy, and the
AK8963 trap. Fill the mapping table when the breakout is physically fixed to
the airframe and re-verify with the bench test at the bottom.

## Frames

- **Chip frame (accel/gyro):** right-handed, as printed on the MPU-9250
  package (datasheet orientation figure): +X and +Y in the package plane,
  +Z up out of the lid. Positive gyro rate = right-hand rotation about the
  matching axis.
- **Aircraft body frame (NED-style):** +X forward (nose), +Y right
  (starboard), +Z down. This is the frame all attitude math (M4+) uses.

## Mapping rules (must all hold, whatever the mounting)

1. The mapping is a **signed axis permutation** (rotation matrix with entries
   in {−1, 0, +1}) — never a per-axis "flip until the demo looks right".
2. **Determinant must be +1** (proper rotation). A mapping with det = −1
   mirrors the world: gyro integration will diverge from accel tilt and the
   bug will look like "filter tuning".
3. Accel and gyro use the **same** mapping (same die, same axes).
4. Write the mapping ONCE, in one place in code (a small transform applied
   right after mpu9250_scale()), not scattered through the filter math.

## Mapping table (fill at mounting time)

| Aircraft axis | Chip axis (sign) | Verified by bench test |
|---------------|------------------|------------------------|
| X (forward)   | TBD              | [ ]                    |
| Y (right)     | TBD              | [ ]                    |
| Z (down)      | TBD              | [ ]                    |

## AK8963 magnetometer trap (deferred, but recorded now)

The AK8963 inside the MPU-9250 has its **own** axis convention that does NOT
match the accel/gyro axes: X and Y are swapped, Z points the opposite way
(datasheet orientation figure overlays both). When the magnetometer joins the
fusion (post-M5, if ever), its readings must be transformed
(mx,my,mz) → (my, mx, −mz) into the chip frame BEFORE the body mapping is
applied. Skipping this yields a heading that is mirrored/rotated and a fusion
that fights itself. Not relevant until the AK8963 is enabled — recorded here
so it cannot be forgotten.

## Bench verification (run once mounted, before M4 work)

With the scaled output line (`A[mg]=...`) on the terminal:

1. **Static Z:** airframe level on the bench → expect A ≈ (0, 0, −1000) mg in
   the aircraft frame (the accelerometer reads −1g on the down axis when
   static: it measures specific force = −gravity, not the gravity vector
   itself; the full comment lives in the M4 filter unit).
2. **Nose down 90°:** expect A ≈ (−1000, 0, 0) mg (check sign per your
   chosen mapping).
3. **Right wing down 90°:** expect A ≈ (0, −1000, 0) mg.
4. **Yaw right by hand:** gyro Z (aircraft frame) must read positive while
   rotating. Repeat for roll-right / pitch-up sign checks.

Any sign that fails → the mapping table entry is wrong; fix the table, not
the filter.
