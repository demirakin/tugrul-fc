# TUGRUL flight controller — Milestone M0
# Build recipe for the STM32F446RE (bare metal, no HAL).
#
# Usage:
#   make            -> build  -> build/tugrul-m0.elf and .bin
#   make clean      -> remove the build/ directory
#   make flash      -> one-shot: program + verify + reset via OpenOCD
#   make openocd    -> start the OpenOCD GDB server (foreground; blocks)
#   make debug      -> launch GDB (needs `make openocd` running elsewhere)
#   make TEST_HANG=1-> build with the deliberate IWDG hang hook enabled
#                      (-DIWDG_TEST_HANG). At t=10 s the firmware stops feeding
#                      the watchdog on purpose so the board resets — watchdog
#                      acceptance test ONLY. NEVER ship an image built this way.
#                      Default build (no TEST_HANG) is unchanged.

# ---- Toolchain ----------------------------------------------------------
PREFIX  = arm-none-eabi-
CC      = $(PREFIX)gcc
OBJCOPY = $(PREFIX)objcopy
SIZE    = $(PREFIX)size
GDB     = $(PREFIX)gdb
OPENOCD = openocd

# ---- Project ------------------------------------------------------------
TARGET  = tugrul-m0
BUILD   = build

C_SOURCES   = main.c clock.c uart.c spi.c mpu9250.c attitude.c pwm.c failsafe.c gyrocal.c iwdg.c
ASM_SOURCES = startup_stm32f446.s
LDSCRIPT    = linker.ld

# ---- CPU / FPU selection (must match the silicon) -----------------------
#   cortex-m4 core, Thumb instruction set, single-precision FPU, hard-float ABI
MCU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

# ---- Includes & device define ------------------------------------------
INCLUDES = -Icmsis
DEFINES  = -DSTM32F446xx

# ---- Compiler flags -----------------------------------------------------
#   -O0 -g            : no optimization + debug info (easy to step through)
#   -Wall -Wextra     : turn on warnings; we want to hear about mistakes
#   -ffunction/-fdata-sections + --gc-sections : drop unused code/data
#   -MMD -MP          : emit .d dependency files next to each .o so header
#                       edits trigger rebuilds (the "Nothing to be done" trap:
#                       without this, a changed .h silently ships a stale image)
CFLAGS  = $(MCU) $(DEFINES) $(INCLUDES) -Wall -Wextra -g -O0 \
          -ffunction-sections -fdata-sections -MMD -MP
ASFLAGS = $(MCU) -g

# ---- Optional: deliberate IWDG hang for watchdog acceptance --------------
# `make TEST_HANG=1` appends -DIWDG_TEST_HANG so the firmware forces a hang at
# t=10 s (see main.c). Test images ONLY — never ship. Default build unaffected.
ifeq ($(TEST_HANG),1)
  CFLAGS += -DIWDG_TEST_HANG
endif

# ---- Linker flags -------------------------------------------------------
#   -nostartfiles     : we provide our own startup, skip the toolchain's
#   --gc-sections     : garbage-collect unreferenced sections
#   -Map              : produce a map file (what ended up where)
LDFLAGS = $(MCU) -T$(LDSCRIPT) -nostartfiles \
          -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(TARGET).map

# ---- Object list --------------------------------------------------------
OBJECTS  = $(addprefix $(BUILD)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD)/,$(ASM_SOURCES:.s=.o))

# ---- OS-specific shell commands (Windows cmd vs Unix sh) ----------------
# 'mkdir -p' and 'rm -rf' are Unix-only; Windows cmd needs its own syntax.
ifeq ($(OS),Windows_NT)
  MKDIR_BUILD = if not exist "$(BUILD)" mkdir "$(BUILD)"
  CLEAN_BUILD = if exist "$(BUILD)" rmdir /S /Q "$(BUILD)"
else
  MKDIR_BUILD = mkdir -p $(BUILD)
  CLEAN_BUILD = rm -rf $(BUILD)
endif

# ---- Rules --------------------------------------------------------------
all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin

$(BUILD):
	$(MKDIR_BUILD)

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@
	@echo ---- size ----
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

clean:
	$(CLEAN_BUILD)

# ---- Debug / flash (OpenOCD + GDB) --------------------------------------
# flash   : one-shot program+verify+reset+exit; no server left running.
flash: $(BUILD)/$(TARGET).elf
	$(OPENOCD) -f openocd.cfg -c "program $(BUILD)/$(TARGET).elf verify reset exit"

# openocd : start the GDB server in the foreground (blocks this terminal).
#           Run this in one terminal, then `make debug` in another.
openocd:
	$(OPENOCD) -f openocd.cfg

# debug   : launch GDB with our startup script. Assumes the OpenOCD server
#           (`make openocd`) is already running and listening on :3333.
debug: $(BUILD)/$(TARGET).elf
	$(GDB) -x debug.gdb

.PHONY: all clean flash openocd debug

# ---- Auto-generated header dependencies (-MMD) ---------------------------
# '-include' (with dash) ignores missing .d files on the first/clean build.
-include $(OBJECTS:.o=.d)
