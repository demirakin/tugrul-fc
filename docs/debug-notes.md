# TUGRUL — Debug backbone notes (GDB + OpenOCD, M0)

Board: STM32 Nucleo-F446RE, on-board ST-LINK/V2-1, SWD transport.
These files are ready to use but were written before the board arrived, so the
exact enumeration/blink behaviour below is expected, not yet observed.

## Prerequisites

- **OpenOCD** (Windows). Recommended source: the xPack builds
  (https://github.com/xpack-dev-tools/openocd-xpack/releases) or a recent
  official release. Use OpenOCD >= 0.11 so that `interface/stlink.cfg` and
  `target/stm32f4x.cfg` exist with current names. Put `openocd.exe` on PATH.
- **arm-none-eabi-gdb**, already part of the toolchain used to build (15.2.1).
- **ST-LINK USB driver.** On Windows the ST-LINK enumerates but needs a libusb
  (WinUSB) backend for OpenOCD. If OpenOCD cannot open the adapter, install the
  WinUSB driver for the ST-LINK interface with Zadig (https://zadig.akeo.ie).
  ST's own ST-LINK driver is fine for ST tools but OpenOCD wants libusb/WinUSB.

## Two-terminal workflow (interactive debugging)

Terminal A — start the GDB server (this blocks/stays in foreground):

    make openocd

Terminal B — build if needed, then launch GDB against the running server:

    make debug

`make debug` runs `arm-none-eabi-gdb -x debug.gdb`, which loads the ELF,
connects to `localhost:3333`, resets+halts, flashes the image, and stops at
`Reset_Handler`. From there use `continue`, `stepi`, `info registers`, etc.

## One-shot flash (no interactive session)

    make flash

This runs OpenOCD once with `program ... verify reset exit`: it flashes,
verifies, resets the MCU to run, and exits. Good for "just load and run".

## What to expect on first connect

- The board appears as an **ST-LINK/V2-1 composite USB device** (debug + a
  Virtual COM port + mass storage). OpenOCD talks to the debug interface only.
- OpenOCD prints the detected target voltage (should be ~3.3 V) and detects
  the Cortex-M core. (With the ST-LINK HLA driver the raw SWD-DP IDCODE is not
  surfaced the way it is with a low-level DAP adapter.)
- **M0 success criterion:** after `make flash` (or `load` + `continue`), the
  user LED **LD2 (PA5) blinks** — that is the visible sign M0 firmware runs.

## Troubleshooting

- **`libusb_open() failed` / adapter not found** — WinUSB/libusb driver missing.
  Run Zadig and assign WinUSB to the ST-LINK interface.
- **`Can't find interface/stlink.cfg` (or target script)** — OpenOCD too old or
  installed without its scripts. Use a recent build; older versions used
  `interface/stlink-v2-1.cfg` instead of the unified `stlink.cfg`.
- **`target voltage may be too low` / voltage 0.0 V** — the Nucleo is not
  powered or the USB cable is charge-only. Use a data cable; check LD3 (the
  power LED — per UM1724 LD1 is the ST-LINK COM LED and LD2 is the user LED).
- **`unable to halt target` / reset problems** — under the ST-LINK HLA driver
  the adapter largely handles reset itself, so a `reset_config srst_only` (or
  `none`) tweak may simply be ignored. Also try `monitor reset init`, a plain
  `monitor reset halt`, or power-cycling the board before reconnecting.
- **`Error: init mode failed (unable to connect to the target)`** — check the
  transport line (`hla_swd`) and that no other tool (ST-Link Utility, CubeIDE)
  holds the ST-LINK open.
