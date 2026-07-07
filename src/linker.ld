/*
 * TUGRUL flight controller — Milestone M0
 * Linker script for the STM32F446RE.
 *
 * Job: tell the linker WHERE each part of the program lives in the chip's
 * memory. A desktop OS does this for you; on bare metal there is no OS, so
 * we describe the memory map by hand.
 *
 *   FLASH  512 KB @ 0x08000000  — non-volatile; the program is stored here
 *   RAM    128 KB @ 0x20000000  — volatile; variables live here at run time
 */

/* Symbolic entry point (used by debuggers/loaders). The real first
   instruction is found via the reset vector in the vector table. */
ENTRY(Reset_Handler)

/* Initial stack pointer = top of RAM. The stack grows downward from here. */
_estack = ORIGIN(RAM) + LENGTH(RAM);

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
  /* The interrupt vector table MUST sit at the very start of FLASH, because
     the CPU reads [0x08000000] for the initial stack pointer and
     [0x08000004] for the reset address the instant power comes up.
     KEEP() stops --gc-sections from discarding it (nothing "calls" it). */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  /* Executable code and read-only constants. */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
    _etext = .;
  } > FLASH

  /* Initialized variables (e.g. 'int x = 5;').
     At run time they live in RAM, but their initial values must survive power
     loss, so the image is stored in FLASH right after .text. The "> RAM AT >
     FLASH" syntax means: addresses are in RAM, but the bytes are loaded into
     FLASH. Reset_Handler copies them across at boot. */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  /* Zero-initialized / uninitialized variables ('static int n;').
     They take no space in FLASH; Reset_Handler just zeroes this RAM region. */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* Discard the C++ exception/unwind tables we don't use, to keep the image
     clean and avoid pulling in unwanted runtime support. */
  /DISCARD/ :
  {
    *(.ARM.exidx*)
    *(.ARM.extab*)
  }
}
