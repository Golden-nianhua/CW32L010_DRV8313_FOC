.syntax unified
.cpu cortex-m0plus
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section.
defined in linker script */
.word _sidata
/* start address for the .data section. defined in linker script */
.word _sdata
/* end address for the .data section. defined in linker script */
.word _edata
/* start address for the .bss section. defined in linker script */
.word _sbss
/* end address for the .bss section. defined in linker script */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied main() routine is called.
 * @param  None
 * @retval None
*/

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Call the clock system initialization function.*/
  bl  SystemInit

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call static constructors */
  bl __libc_init_array
/* Call the application s entry point.*/
  bl main

LoopForever:
  b LoopForever

.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval None
*/
  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

/******************************************************************************
*
* The minimal vector table for a Cortex M0.  Note that the proper constructs
* must be placed on this to ensure that it ends up at physical address
* 0x0000.0000.
*
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object

g_pfnVectors:
  .word _estack                   /*  Top of Stack */
  .word Reset_Handler             /* < -15 Reset Vector, invoked on Power up and warm reset */
  .word NMI_Handler               /* < -14 Non maskable Interrupt, cannot be stopped or preempted */
  .word HardFault_Handler         /* < -13 Hard Fault, all classes of Fault */
  .word 0                         /* < -12 Reserved */
  .word 0                         /* < -11 Reserved */
  .word 0                         /* < -10 Reserved */
  .word 0                         /* < -9 Reserved */
  .word 0                         /* < -8 Reserved */
  .word 0                         /* < -7 Reserved */
  .word 0                         /* < -6 Reserved */
  .word SVC_Handler               /* < -5 System Service Call via SVC instruction */
  .word 0                         /* < -4 Reserved */
  .word 0                         /* < -3 Reserved */
  .word PendSV_Handler            /* < -2 Pendable request for system service */
  .word SysTick_Handler           /* < -1 System Tick Timer */

  .word WDT_IRQHandler            /* < 0 Watch Dog Timer Interrupt Handler */
  .word LVD_IRQHandler            /* < 1 Low Voltage Detect Interrupt Handler */
  .word RTC_IRQHandler            /* < 2 Real Time Clock Interrupt Handler */
  .word FLASHRAM_IRQHandler       /* < 3 Flash/RAM Interrupt Handler */
  .word SYSCTRL_IRQHandler        /* < 4 SysCtrl Interupt Handler */
  .word GPIOA_IRQHandler          /* < 5 GPIOA Interrupt Handler */
  .word GPIOB_IRQHandler          /* < 6 GPIOB Interrupt Handler */
  .word 0                         /* < 7 Reserved */
  .word 0                         /* < 8 Reserved */
  .word 0                         /* < 9 Reserved */
  .word 0                         /* < 10 Reserved */
  .word 0                         /* < 11 Reserved */
  .word ADC_IRQHandler            /* < 12 ADC Interrupt Handler */
  .word ATIM_IRQHandler           /* < 13 Advanced Timer Interrupt Handler */
  .word VC1_IRQHandler            /* < 14 Voltage Comparator 1 Interrupt Handler */
  .word VC2_IRQHandler            /* < 15 Voltage Comparator 2 Interrupt Handler */
  .word GTIM1_IRQHandler          /* < 16 General Timer1 Interrupt Handler */
  .word 0                         /* < 17 Reserved */
  .word 0                         /* < 18 Reserved */
  .word LPTIM_IRQHandler          /* < 19 LPTIM Interrupt Handle */
  .word BTIM1_IRQHandler          /* < 20 Base Timer1 Interrupt Handler */
  .word BTIM2_IRQHandler          /* < 21 Base Timer2 Interrupt Handler */
  .word BTIM3_IRQHandler          /* < 22 Base Timer3 Interrupt Handler */
  .word I2C1_IRQHandler           /* < 23 I2C1 Interrupt Handler */
  .word 0                         /* < 24 Reserved */
  .word SPI1_IRQHandler           /* < 25 SPI1 Interrupt Handler */
  .word 0                         /* < 26 Reserved */
  .word UART1_IRQHandler          /* < 27 UART1 Interrupt Handler */
  .word UART2_IRQHandler          /* < 28 UART2 Interrupt Handler */
  .word 0                         /* < 29 Reserved */
  .word 0                         /* < 30 Reserved */
  .word CLKFAULT_IRQHandler       /* < 31 CLKFAULT Interrupt Handler */

  .size g_pfnVectors, .-g_pfnVectors

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak      NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak      HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak      SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak      PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak      SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler


  .weak      WDT_IRQHandler
  .thumb_set WDT_IRQHandler,Default_Handler

  .weak      LVD_IRQHandler
  .thumb_set LVD_IRQHandler,Default_Handler

  .weak      RTC_IRQHandler
  .thumb_set RTC_IRQHandler,Default_Handler

  .weak      FLASHRAM_IRQHandler
  .thumb_set FLASHRAM_IRQHandler,Default_Handler

  .weak      SYSCTRL_IRQHandler
  .thumb_set SYSCTRL_IRQHandler,Default_Handler

  .weak      GPIOA_IRQHandler
  .thumb_set GPIOA_IRQHandler,Default_Handler

  .weak      GPIOB_IRQHandler
  .thumb_set GPIOB_IRQHandler,Default_Handler

  .weak      ADC_IRQHandler
  .thumb_set ADC_IRQHandler,Default_Handler

  .weak      ATIM_IRQHandler
  .thumb_set ATIM_IRQHandler,Default_Handler

  .weak      VC1_IRQHandler
  .thumb_set VC1_IRQHandler,Default_Handler

  .weak      VC2_IRQHandler
  .thumb_set VC2_IRQHandler,Default_Handler

  .weak      GTIM1_IRQHandler
  .thumb_set GTIM1_IRQHandler,Default_Handler

  .weak      LPTIM_IRQHandler
  .thumb_set LPTIM_IRQHandler,Default_Handler

  .weak      BTIM1_IRQHandler
  .thumb_set BTIM1_IRQHandler,Default_Handler

  .weak      BTIM2_IRQHandler
  .thumb_set BTIM2_IRQHandler,Default_Handler

  .weak      BTIM3_IRQHandler
  .thumb_set BTIM3_IRQHandler,Default_Handler

  .weak      I2C1_IRQHandler
  .thumb_set I2C1_IRQHandler,Default_Handler

  .weak      SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak      UART1_IRQHandler
  .thumb_set UART1_IRQHandler,Default_Handler

  .weak      UART2_IRQHandler
  .thumb_set UART2_IRQHandler,Default_Handler

  .weak      CLKFAULT_IRQHandler
  .thumb_set CLKFAULT_IRQHandler,Default_Handler