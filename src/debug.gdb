# TUGRUL flight controller — GDB startup script (M0 debug backbone)
# Load with: arm-none-eabi-gdb -x debug.gdb
#
# Deliberately NOT named .gdbinit: GDB's auto-load safe-path would silently
# ignore a project-local .gdbinit unless the workspace is whitelisted, which
# hides the setup. Passing this explicitly with -x is unambiguous.
#
# Assumes an OpenOCD GDB server is already listening on localhost:3333
# (start it with `make openocd` in a separate terminal).

# Do not prompt for confirmation on commands like `quit` — keep the flow fast.
set confirm off

# Load the ELF so GDB has symbols and can flash/verify the image.
file build/tugrul-m0.elf

# Connect to the OpenOCD GDB server (extended-remote allows program restart).
target extended-remote localhost:3333

# fixed (finding 1): flash FIRST, then reset. `load` writes the new image but
# only sets PC to the entry point; it does NOT re-latch MSP. Reset_Handler in
# startup_stm32f446.s never sets SP itself — the hardware reset sequence loads
# MSP from vector[0]. So we must reset AFTER load to re-latch SP/PC from the
# freshly-flashed vector table (the old order only worked when the reflashed
# image was byte-identical to what was already on the device).
load

# Reset+halt after flashing: re-latches SP and PC from the new vector table
# and stops at the reset vector before any code runs.
monitor reset halt

# Break at Reset_Handler rather than main: M0 has no C main() with a normal
# call flow to rely on yet, and stopping at the reset vector lets us verify
# the vector table, initial SP and startup path from the very first instruction.
# The breakpoint also catches subsequent `monitor reset run` / `continue`
# cycles, re-halting at the reset vector each time.
break Reset_Handler
