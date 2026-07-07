/*
 * TUGRUL flight controller — Milestone M0 (core) / M5-prep (external vectors)
 * Startup code for the STM32F446RE (Cortex-M4F).
 *
 * Two responsibilities:
 *   1) Provide the interrupt vector table (the very first thing in FLASH).
 *   2) Provide Reset_Handler: the first code that runs after power-up. It
 *      prepares the C run-time environment, then calls main().
 *
 * M5-prep change: the M0 table stopped at SysTick because nothing used a
 * peripheral interrupt yet. The non-blocking UART TX (uart.c) needs the USART2
 * vector to actually be dispatched, so the full STM32F446 external interrupt set
 * (positions 0..96, ground truth = the IRQn_Type enum in cmsis/stm32f446xx.h,
 * max = FMPI2C1_ER_IRQn = 96) is now appended, each with the same weak-alias-to-
 * Default_Handler pattern already used for the core exceptions. Any future
 * peripheral ISR (e.g. TIM for M5 PWM) now works simply by defining its
 * <PERIPH>_IRQHandler symbol in C — no further startup edits needed. Reserved
 * gaps in the enum (no peripheral at that position) are left as .word 0, the
 * same convention this file already uses for reserved core-exception slots.
 */

  .syntax unified
  .cpu cortex-m4
  .fpu fpv4-sp-d16
  .thumb

/* ======================================================================== */
/*  Reset handler — runs first, sets up C run-time, then calls main()        */
/* ======================================================================== */
  .section .text.Reset_Handler
  .weak    Reset_Handler
  .type    Reset_Handler, %function
Reset_Handler:

  /* --- Enable the FPU (give full access to coprocessors CP10 & CP11). ---
     We build with hard-float, so the FPU must be on before any floating
     point instruction executes, otherwise the CPU faults. CPACR @ 0xE000ED88. */
  ldr   r0, =0xE000ED88
  ldr   r1, [r0]
  orr   r1, r1, #(0xF << 20)
  str   r1, [r0]
  dsb
  isb

  /* --- Copy .data (initialized variables) from FLASH into RAM. --- */
  ldr   r0, =_sdata        /* destination start (RAM)            */
  ldr   r1, =_edata        /* destination end   (RAM)            */
  ldr   r2, =_sidata       /* source start      (FLASH image)    */
  movs  r3, #0
  b     LoopCopyData
CopyData:
  ldr   r4, [r2, r3]
  str   r4, [r0, r3]
  adds  r3, r3, #4
LoopCopyData:
  adds  r4, r0, r3
  cmp   r4, r1
  bcc   CopyData

  /* --- Zero-fill .bss (uninitialized variables). --- */
  ldr   r2, =_sbss
  ldr   r4, =_ebss
  movs  r3, #0
  b     LoopFillZerobss
FillZerobss:
  str   r3, [r2]
  adds  r2, r2, #4
LoopFillZerobss:
  cmp   r2, r4
  bcc   FillZerobss

  /* --- Hand control to the C program. --- */
  bl    main

  /* main() should never return; if it does, trap here forever. */
LoopForever:
  b     LoopForever
  .size Reset_Handler, .-Reset_Handler

/* ======================================================================== */
/*  Default handler — any unexpected exception lands here                    */
/* ======================================================================== */
  .section .text.Default_Handler, "ax", %progbits
  .type    Default_Handler, %function
Default_Handler:
  b     Default_Handler
  .size Default_Handler, .-Default_Handler

/* ======================================================================== */
/*  Cortex-M4 core vector table                                              */
/*  Peripheral IRQ vectors (UART, SPI, timers...) are added in later         */
/*  milestones as we start using those interrupts.                           */
/* ======================================================================== */
  .section .isr_vector, "a", %progbits
  .global  g_pfnVectors
  .type    g_pfnVectors, %object
g_pfnVectors:
  .word _estack             /* 0x00  Initial Stack Pointer        */
  .word Reset_Handler       /* 0x04  Reset                        */
  .word NMI_Handler         /* 0x08  Non-Maskable Interrupt       */
  .word HardFault_Handler   /* 0x0C  Hard Fault                   */
  .word MemManage_Handler   /* 0x10  Memory Management Fault      */
  .word BusFault_Handler    /* 0x14  Bus Fault                    */
  .word UsageFault_Handler  /* 0x18  Usage Fault                  */
  .word 0                   /* 0x1C  Reserved                     */
  .word 0                   /* 0x20  Reserved                     */
  .word 0                   /* 0x24  Reserved                     */
  .word 0                   /* 0x28  Reserved                     */
  .word SVC_Handler         /* 0x2C  SVCall                       */
  .word DebugMon_Handler    /* 0x30  Debug Monitor                */
  .word 0                   /* 0x34  Reserved                     */
  .word PendSV_Handler      /* 0x38  PendSV                       */
  .word SysTick_Handler     /* 0x3C  SysTick                      */

  /* ---- STM32F446 external interrupts (NVIC), IRQ position 0..96. ----------
   * Byte offset = 0x40 + IRQpos*4  (0x40 = 16 core vectors * 4). Each line's
   * trailing comment shows "pos <n> @ <offset>". Verify USART2 lands right:
   *   USART2 IRQpos = 38  ->  vector index 16+38 = 54  ->  0x40 + 38*4 = 0xD8. */
  .word WWDG_IRQHandler                    /* pos 0  @ 0x40  Window Watchdog          */
  .word PVD_IRQHandler                     /* pos 1  @ 0x44  PVD via EXTI             */
  .word TAMP_STAMP_IRQHandler              /* pos 2  @ 0x48  Tamper / TimeStamp       */
  .word RTC_WKUP_IRQHandler                /* pos 3  @ 0x4C  RTC Wakeup               */
  .word FLASH_IRQHandler                   /* pos 4  @ 0x50  FLASH global             */
  .word RCC_IRQHandler                     /* pos 5  @ 0x54  RCC global               */
  .word EXTI0_IRQHandler                   /* pos 6  @ 0x58  EXTI Line0               */
  .word EXTI1_IRQHandler                   /* pos 7  @ 0x5C  EXTI Line1               */
  .word EXTI2_IRQHandler                   /* pos 8  @ 0x60  EXTI Line2               */
  .word EXTI3_IRQHandler                   /* pos 9  @ 0x64  EXTI Line3               */
  .word EXTI4_IRQHandler                   /* pos 10 @ 0x68  EXTI Line4               */
  .word DMA1_Stream0_IRQHandler            /* pos 11 @ 0x6C  DMA1 Stream0             */
  .word DMA1_Stream1_IRQHandler            /* pos 12 @ 0x70  DMA1 Stream1             */
  .word DMA1_Stream2_IRQHandler            /* pos 13 @ 0x74  DMA1 Stream2             */
  .word DMA1_Stream3_IRQHandler            /* pos 14 @ 0x78  DMA1 Stream3             */
  .word DMA1_Stream4_IRQHandler            /* pos 15 @ 0x7C  DMA1 Stream4             */
  .word DMA1_Stream5_IRQHandler            /* pos 16 @ 0x80  DMA1 Stream5             */
  .word DMA1_Stream6_IRQHandler            /* pos 17 @ 0x84  DMA1 Stream6             */
  .word ADC_IRQHandler                     /* pos 18 @ 0x88  ADC1/2/3                 */
  .word CAN1_TX_IRQHandler                 /* pos 19 @ 0x8C  CAN1 TX                  */
  .word CAN1_RX0_IRQHandler                /* pos 20 @ 0x90  CAN1 RX0                 */
  .word CAN1_RX1_IRQHandler                /* pos 21 @ 0x94  CAN1 RX1                 */
  .word CAN1_SCE_IRQHandler                /* pos 22 @ 0x98  CAN1 SCE                 */
  .word EXTI9_5_IRQHandler                 /* pos 23 @ 0x9C  EXTI Line[9:5]           */
  .word TIM1_BRK_TIM9_IRQHandler           /* pos 24 @ 0xA0  TIM1 Break / TIM9        */
  .word TIM1_UP_TIM10_IRQHandler           /* pos 25 @ 0xA4  TIM1 Update / TIM10      */
  .word TIM1_TRG_COM_TIM11_IRQHandler      /* pos 26 @ 0xA8  TIM1 Trg/Com / TIM11     */
  .word TIM1_CC_IRQHandler                 /* pos 27 @ 0xAC  TIM1 Capture Compare     */
  .word TIM2_IRQHandler                    /* pos 28 @ 0xB0  TIM2 global              */
  .word TIM3_IRQHandler                    /* pos 29 @ 0xB4  TIM3 global              */
  .word TIM4_IRQHandler                    /* pos 30 @ 0xB8  TIM4 global              */
  .word I2C1_EV_IRQHandler                 /* pos 31 @ 0xBC  I2C1 Event               */
  .word I2C1_ER_IRQHandler                 /* pos 32 @ 0xC0  I2C1 Error               */
  .word I2C2_EV_IRQHandler                 /* pos 33 @ 0xC4  I2C2 Event               */
  .word I2C2_ER_IRQHandler                 /* pos 34 @ 0xC8  I2C2 Error               */
  .word SPI1_IRQHandler                    /* pos 35 @ 0xCC  SPI1 global              */
  .word SPI2_IRQHandler                    /* pos 36 @ 0xD0  SPI2 global              */
  .word USART1_IRQHandler                  /* pos 37 @ 0xD4  USART1 global            */
  .word USART2_IRQHandler                  /* pos 38 @ 0xD8  USART2 global  <-- TX ISR */
  .word USART3_IRQHandler                  /* pos 39 @ 0xDC  USART3 global            */
  .word EXTI15_10_IRQHandler               /* pos 40 @ 0xE0  EXTI Line[15:10]         */
  .word RTC_Alarm_IRQHandler               /* pos 41 @ 0xE4  RTC Alarm A/B via EXTI   */
  .word OTG_FS_WKUP_IRQHandler             /* pos 42 @ 0xE8  USB OTG FS Wakeup        */
  .word TIM8_BRK_TIM12_IRQHandler          /* pos 43 @ 0xEC  TIM8 Break / TIM12       */
  .word TIM8_UP_TIM13_IRQHandler           /* pos 44 @ 0xF0  TIM8 Update / TIM13      */
  .word TIM8_TRG_COM_TIM14_IRQHandler      /* pos 45 @ 0xF4  TIM8 Trg/Com / TIM14     */
  .word TIM8_CC_IRQHandler                 /* pos 46 @ 0xF8  TIM8 Capture Compare     */
  .word DMA1_Stream7_IRQHandler            /* pos 47 @ 0xFC  DMA1 Stream7             */
  .word FMC_IRQHandler                     /* pos 48 @ 0x100 FMC global               */
  .word SDIO_IRQHandler                    /* pos 49 @ 0x104 SDIO global              */
  .word TIM5_IRQHandler                    /* pos 50 @ 0x108 TIM5 global              */
  .word SPI3_IRQHandler                    /* pos 51 @ 0x10C SPI3 global              */
  .word UART4_IRQHandler                   /* pos 52 @ 0x110 UART4 global             */
  .word UART5_IRQHandler                   /* pos 53 @ 0x114 UART5 global             */
  .word TIM6_DAC_IRQHandler                /* pos 54 @ 0x118 TIM6 / DAC underrun      */
  .word TIM7_IRQHandler                    /* pos 55 @ 0x11C TIM7 global              */
  .word DMA2_Stream0_IRQHandler            /* pos 56 @ 0x120 DMA2 Stream0             */
  .word DMA2_Stream1_IRQHandler            /* pos 57 @ 0x124 DMA2 Stream1             */
  .word DMA2_Stream2_IRQHandler            /* pos 58 @ 0x128 DMA2 Stream2             */
  .word DMA2_Stream3_IRQHandler            /* pos 59 @ 0x12C DMA2 Stream3             */
  .word DMA2_Stream4_IRQHandler            /* pos 60 @ 0x130 DMA2 Stream4             */
  .word 0                                  /* pos 61 @ 0x134 reserved (no peripheral) */
  .word 0                                  /* pos 62 @ 0x138 reserved (no peripheral) */
  .word CAN2_TX_IRQHandler                 /* pos 63 @ 0x13C CAN2 TX                  */
  .word CAN2_RX0_IRQHandler                /* pos 64 @ 0x140 CAN2 RX0                 */
  .word CAN2_RX1_IRQHandler                /* pos 65 @ 0x144 CAN2 RX1                 */
  .word CAN2_SCE_IRQHandler                /* pos 66 @ 0x148 CAN2 SCE                 */
  .word OTG_FS_IRQHandler                  /* pos 67 @ 0x14C USB OTG FS global        */
  .word DMA2_Stream5_IRQHandler            /* pos 68 @ 0x150 DMA2 Stream5             */
  .word DMA2_Stream6_IRQHandler            /* pos 69 @ 0x154 DMA2 Stream6             */
  .word DMA2_Stream7_IRQHandler            /* pos 70 @ 0x158 DMA2 Stream7             */
  .word USART6_IRQHandler                  /* pos 71 @ 0x15C USART6 global            */
  .word I2C3_EV_IRQHandler                 /* pos 72 @ 0x160 I2C3 Event               */
  .word I2C3_ER_IRQHandler                 /* pos 73 @ 0x164 I2C3 Error               */
  .word OTG_HS_EP1_OUT_IRQHandler          /* pos 74 @ 0x168 USB OTG HS EP1 Out       */
  .word OTG_HS_EP1_IN_IRQHandler           /* pos 75 @ 0x16C USB OTG HS EP1 In        */
  .word OTG_HS_WKUP_IRQHandler             /* pos 76 @ 0x170 USB OTG HS Wakeup         */
  .word OTG_HS_IRQHandler                  /* pos 77 @ 0x174 USB OTG HS global         */
  .word DCMI_IRQHandler                    /* pos 78 @ 0x178 DCMI global               */
  .word 0                                  /* pos 79 @ 0x17C reserved (no peripheral)  */
  .word 0                                  /* pos 80 @ 0x180 reserved (no peripheral)  */
  .word FPU_IRQHandler                     /* pos 81 @ 0x184 FPU global                */
  .word 0                                  /* pos 82 @ 0x188 reserved (no peripheral)  */
  .word 0                                  /* pos 83 @ 0x18C reserved (no peripheral)  */
  .word SPI4_IRQHandler                    /* pos 84 @ 0x190 SPI4 global               */
  .word 0                                  /* pos 85 @ 0x194 reserved (no peripheral)  */
  .word 0                                  /* pos 86 @ 0x198 reserved (no peripheral)  */
  .word SAI1_IRQHandler                    /* pos 87 @ 0x19C SAI1 global               */
  .word 0                                  /* pos 88 @ 0x1A0 reserved (no peripheral)  */
  .word 0                                  /* pos 89 @ 0x1A4 reserved (no peripheral)  */
  .word 0                                  /* pos 90 @ 0x1A8 reserved (no peripheral)  */
  .word SAI2_IRQHandler                    /* pos 91 @ 0x1AC SAI2 global               */
  .word QUADSPI_IRQHandler                 /* pos 92 @ 0x1B0 QuadSPI global            */
  .word CEC_IRQHandler                     /* pos 93 @ 0x1B4 CEC global                */
  .word SPDIF_RX_IRQHandler                /* pos 94 @ 0x1B8 SPDIF-RX global           */
  .word FMPI2C1_EV_IRQHandler              /* pos 95 @ 0x1BC FMPI2C1 Event             */
  .word FMPI2C1_ER_IRQHandler              /* pos 96 @ 0x1C0 FMPI2C1 Error (last)      */
  .size g_pfnVectors, .-g_pfnVectors

/* ======================================================================== */
/*  Weak aliases: every exception defaults to Default_Handler. Defining a     */
/*  function with the same name anywhere else automatically overrides these.  */
/* ======================================================================== */
  .weak NMI_Handler
  .thumb_set NMI_Handler, Default_Handler
  .weak HardFault_Handler
  .thumb_set HardFault_Handler, Default_Handler
  .weak MemManage_Handler
  .thumb_set MemManage_Handler, Default_Handler
  .weak BusFault_Handler
  .thumb_set BusFault_Handler, Default_Handler
  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler, Default_Handler
  .weak SVC_Handler
  .thumb_set SVC_Handler, Default_Handler
  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler, Default_Handler
  .weak PendSV_Handler
  .thumb_set PendSV_Handler, Default_Handler
  .weak SysTick_Handler
  .thumb_set SysTick_Handler, Default_Handler

/* ======================================================================== */
/*  Weak aliases for the STM32F446 external interrupts. Each defaults to      */
/*  Default_Handler; a strong C definition of <PERIPH>_IRQHandler overrides   */
/*  it (exactly how clock.c's SysTick_Handler and uart.c's USART2_IRQHandler  */
/*  take over their slots). Reserved positions have no symbol (they are       */
/*  .word 0 in the table above), so none is listed here.                      */
/* ======================================================================== */
  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler, Default_Handler
  .weak PVD_IRQHandler
  .thumb_set PVD_IRQHandler, Default_Handler
  .weak TAMP_STAMP_IRQHandler
  .thumb_set TAMP_STAMP_IRQHandler, Default_Handler
  .weak RTC_WKUP_IRQHandler
  .thumb_set RTC_WKUP_IRQHandler, Default_Handler
  .weak FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler, Default_Handler
  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler, Default_Handler
  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler, Default_Handler
  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler, Default_Handler
  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler, Default_Handler
  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler, Default_Handler
  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler, Default_Handler
  .weak DMA1_Stream0_IRQHandler
  .thumb_set DMA1_Stream0_IRQHandler, Default_Handler
  .weak DMA1_Stream1_IRQHandler
  .thumb_set DMA1_Stream1_IRQHandler, Default_Handler
  .weak DMA1_Stream2_IRQHandler
  .thumb_set DMA1_Stream2_IRQHandler, Default_Handler
  .weak DMA1_Stream3_IRQHandler
  .thumb_set DMA1_Stream3_IRQHandler, Default_Handler
  .weak DMA1_Stream4_IRQHandler
  .thumb_set DMA1_Stream4_IRQHandler, Default_Handler
  .weak DMA1_Stream5_IRQHandler
  .thumb_set DMA1_Stream5_IRQHandler, Default_Handler
  .weak DMA1_Stream6_IRQHandler
  .thumb_set DMA1_Stream6_IRQHandler, Default_Handler
  .weak ADC_IRQHandler
  .thumb_set ADC_IRQHandler, Default_Handler
  .weak CAN1_TX_IRQHandler
  .thumb_set CAN1_TX_IRQHandler, Default_Handler
  .weak CAN1_RX0_IRQHandler
  .thumb_set CAN1_RX0_IRQHandler, Default_Handler
  .weak CAN1_RX1_IRQHandler
  .thumb_set CAN1_RX1_IRQHandler, Default_Handler
  .weak CAN1_SCE_IRQHandler
  .thumb_set CAN1_SCE_IRQHandler, Default_Handler
  .weak EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler, Default_Handler
  .weak TIM1_BRK_TIM9_IRQHandler
  .thumb_set TIM1_BRK_TIM9_IRQHandler, Default_Handler
  .weak TIM1_UP_TIM10_IRQHandler
  .thumb_set TIM1_UP_TIM10_IRQHandler, Default_Handler
  .weak TIM1_TRG_COM_TIM11_IRQHandler
  .thumb_set TIM1_TRG_COM_TIM11_IRQHandler, Default_Handler
  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler, Default_Handler
  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler, Default_Handler
  .weak TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler, Default_Handler
  .weak TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler, Default_Handler
  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler, Default_Handler
  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler, Default_Handler
  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler, Default_Handler
  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler, Default_Handler
  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler, Default_Handler
  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler, Default_Handler
  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler, Default_Handler
  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler, Default_Handler
  .weak USART3_IRQHandler
  .thumb_set USART3_IRQHandler, Default_Handler
  .weak EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler, Default_Handler
  .weak RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler, Default_Handler
  .weak OTG_FS_WKUP_IRQHandler
  .thumb_set OTG_FS_WKUP_IRQHandler, Default_Handler
  .weak TIM8_BRK_TIM12_IRQHandler
  .thumb_set TIM8_BRK_TIM12_IRQHandler, Default_Handler
  .weak TIM8_UP_TIM13_IRQHandler
  .thumb_set TIM8_UP_TIM13_IRQHandler, Default_Handler
  .weak TIM8_TRG_COM_TIM14_IRQHandler
  .thumb_set TIM8_TRG_COM_TIM14_IRQHandler, Default_Handler
  .weak TIM8_CC_IRQHandler
  .thumb_set TIM8_CC_IRQHandler, Default_Handler
  .weak DMA1_Stream7_IRQHandler
  .thumb_set DMA1_Stream7_IRQHandler, Default_Handler
  .weak FMC_IRQHandler
  .thumb_set FMC_IRQHandler, Default_Handler
  .weak SDIO_IRQHandler
  .thumb_set SDIO_IRQHandler, Default_Handler
  .weak TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler, Default_Handler
  .weak SPI3_IRQHandler
  .thumb_set SPI3_IRQHandler, Default_Handler
  .weak UART4_IRQHandler
  .thumb_set UART4_IRQHandler, Default_Handler
  .weak UART5_IRQHandler
  .thumb_set UART5_IRQHandler, Default_Handler
  .weak TIM6_DAC_IRQHandler
  .thumb_set TIM6_DAC_IRQHandler, Default_Handler
  .weak TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler, Default_Handler
  .weak DMA2_Stream0_IRQHandler
  .thumb_set DMA2_Stream0_IRQHandler, Default_Handler
  .weak DMA2_Stream1_IRQHandler
  .thumb_set DMA2_Stream1_IRQHandler, Default_Handler
  .weak DMA2_Stream2_IRQHandler
  .thumb_set DMA2_Stream2_IRQHandler, Default_Handler
  .weak DMA2_Stream3_IRQHandler
  .thumb_set DMA2_Stream3_IRQHandler, Default_Handler
  .weak DMA2_Stream4_IRQHandler
  .thumb_set DMA2_Stream4_IRQHandler, Default_Handler
  .weak CAN2_TX_IRQHandler
  .thumb_set CAN2_TX_IRQHandler, Default_Handler
  .weak CAN2_RX0_IRQHandler
  .thumb_set CAN2_RX0_IRQHandler, Default_Handler
  .weak CAN2_RX1_IRQHandler
  .thumb_set CAN2_RX1_IRQHandler, Default_Handler
  .weak CAN2_SCE_IRQHandler
  .thumb_set CAN2_SCE_IRQHandler, Default_Handler
  .weak OTG_FS_IRQHandler
  .thumb_set OTG_FS_IRQHandler, Default_Handler
  .weak DMA2_Stream5_IRQHandler
  .thumb_set DMA2_Stream5_IRQHandler, Default_Handler
  .weak DMA2_Stream6_IRQHandler
  .thumb_set DMA2_Stream6_IRQHandler, Default_Handler
  .weak DMA2_Stream7_IRQHandler
  .thumb_set DMA2_Stream7_IRQHandler, Default_Handler
  .weak USART6_IRQHandler
  .thumb_set USART6_IRQHandler, Default_Handler
  .weak I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler, Default_Handler
  .weak I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler, Default_Handler
  .weak OTG_HS_EP1_OUT_IRQHandler
  .thumb_set OTG_HS_EP1_OUT_IRQHandler, Default_Handler
  .weak OTG_HS_EP1_IN_IRQHandler
  .thumb_set OTG_HS_EP1_IN_IRQHandler, Default_Handler
  .weak OTG_HS_WKUP_IRQHandler
  .thumb_set OTG_HS_WKUP_IRQHandler, Default_Handler
  .weak OTG_HS_IRQHandler
  .thumb_set OTG_HS_IRQHandler, Default_Handler
  .weak DCMI_IRQHandler
  .thumb_set DCMI_IRQHandler, Default_Handler
  .weak FPU_IRQHandler
  .thumb_set FPU_IRQHandler, Default_Handler
  .weak SPI4_IRQHandler
  .thumb_set SPI4_IRQHandler, Default_Handler
  .weak SAI1_IRQHandler
  .thumb_set SAI1_IRQHandler, Default_Handler
  .weak SAI2_IRQHandler
  .thumb_set SAI2_IRQHandler, Default_Handler
  .weak QUADSPI_IRQHandler
  .thumb_set QUADSPI_IRQHandler, Default_Handler
  .weak CEC_IRQHandler
  .thumb_set CEC_IRQHandler, Default_Handler
  .weak SPDIF_RX_IRQHandler
  .thumb_set SPDIF_RX_IRQHandler, Default_Handler
  .weak FMPI2C1_EV_IRQHandler
  .thumb_set FMPI2C1_EV_IRQHandler, Default_Handler
  .weak FMPI2C1_ER_IRQHandler
  .thumb_set FMPI2C1_ER_IRQHandler, Default_Handler
