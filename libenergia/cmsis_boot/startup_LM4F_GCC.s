/* File: startup_ARMCM4.S
 * Purpose: startup file for Cortex-M4 devices. Should use with
 *   GCC for ARM Embedded Processors
 * Version: V1.4
 * Date: 20 Dezember 2012
 *
 */
/* Copyright (c) 2011 - 2012 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


    .syntax unified
    .arch armv7e-m

    .section .stack
    .align 3
#ifdef __STACK_SIZE
    .equ    Stack_Size, __STACK_SIZE
#else
    .equ    Stack_Size, 0x00000400
#endif
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

    .section .heap
    .align 3
#ifdef __HEAP_SIZE
    .equ    Heap_Size, __HEAP_SIZE
#else
    .equ    Heap_Size, 0x00000C00
#endif
    .globl    __HeapBase
    .globl    __HeapLimit
__HeapBase:
    .if    Heap_Size
    .space    Heap_Size
    .endif
    .size __HeapBase, . - __HeapBase
__HeapLimit:
    .size __HeapLimit, . - __HeapLimit

    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    .long    __StackTop            /* Top of Stack */
    .long    Reset_Handler         /* Reset Handler */
    .long    NMI_Handler           /* NMI Handler */
    .long    HardFault_Handler     /* Hard Fault Handler */
    .long    MemManage_Handler     /* MPU Fault Handler */
    .long    BusFault_Handler      /* Bus Fault Handler */
    .long    UsageFault_Handler    /* Usage Fault Handler */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    0                     /* Reserved */
    .long    SVC_Handler           /* SVCall Handler */
    .long    DebugMon_Handler      /* Debug Monitor Handler */
    .long    0                     /* Reserved */
    .long    PendSV_Handler        /* PendSV Handler */
    .long    SysTick_Handler       /* SysTick Handler */

    /* External interrupts */

    .long    GPIOA_Handler             /* GPIO Port A			 */
    .long    GPIOB_Handler             /* GPIO Port B			 */
    .long    GPIOC_Handler             /* GPIO Port C			 */
    .long    GPIOD_Handler             /* GPIO Port D			 */
    .long    GPIOE_Handler             /* GPIO Port E			 */
    .long    UART0_Handler             /* UART0 Rx and Tx		 */
    .long    UART1_Handler             /* UART1 Rx and Tx		 */
    .long    SSI0_Handler              /* SSI0 Rx and Tx		 */
    .long    I2C0_Handler              /* I2C0 Master and Slave		 */
    .long    PMW0_FAULT_Handler        /* PWM Fault			 */
    .long    PWM0_0_Handler            /* PWM Generator 0		 */
    .long    PWM0_1_Handler            /* PWM Generator 1		 */
    .long    PWM0_2_Handler            /* PWM Generator 2		 */
    .long    QEI0_Handler              /* Quadrature Encoder 0		 */
    .long    ADC0SS0_Handler           /* ADC Sequence 0		 */
    .long    ADC0SS1_Handler           /* ADC Sequence 1		 */
    .long    ADC0SS2_Handler           /* ADC Sequence 2		 */
    .long    ADC0SS3_Handler           /* ADC Sequence 3		 */
    .long    WDT0_Handler              /* Watchdog timer		 */
    .long    TIMER0A_Handler           /* Timer 0 subtimer A		 */
    .long    TIMER0B_Handler           /* Timer 0 subtimer B		 */
    .long    TIMER1A_Handler           /* Timer 1 subtimer A		 */
    .long    TIMER1B_Handler           /* Timer 1 subtimer B		 */
    .long    TIMER2A_Handler           /* Timer 2 subtimer A		 */
    .long    TIMER2B_Handler           /* Timer 2 subtimer B		 */
    .long    COMP0_Handler             /* Analog Comparator 0		 */
    .long    COMP1_Handler             /* Analog Comparator 1		 */
    .long    COMP2_Handler             /* Analog Comparator 2            */
    .long    SYSCTL_Handler            /* System Control (PLL, OSC, BO)  */
    .long    FLASH_Handler             /* FLASH Control			 */
    .long    GPIOF_Handler             /* GPIO Port F			 */
    .long    GPIOG_Handler             /* GPIO Port G			 */
    .long    GPIOH_Handler             /* GPIO Port H			 */
    .long    UART2_Handler             /* UART2 Rx and Tx		 */
    .long    SSI1_Handler              /* SSI1 Rx and Tx		 */
    .long    TIMER3A_Handler           /* Timer 3 subtimer A		 */
    .long    TIMER3B_Handler           /* Timer 3 subtimer B		 */
    .long    I2C1_Handler              /* I2C1 Master and Slave		 */
    .long    QEI1_Handler              /* Quadrature Encoder 1		 */
    .long    CAN0_Handler              /* CAN0				 */
    .long    CAN1_Handler              /* CAN1				 */
    .long    CAN2_Handler              /* CAN2				 */
    .long    ETH0_Handler              /* Ethernet			 */
    .long    HIB_Handler               /* Hibernate			 */
    .long    USB0_Handler              /* USB0				 */
    .long    PWM0_3_Handler            /* PWM Generator 3		 */
    .long    UDMA_Handler              /* uDMA Software Transfer	 */
    .long    UDMAERR_Handler           /* uDMA Error			 */
    .long    ADC1SS0_Handler           /* ADC1 Sequence 0		 */
    .long    ADC1SS1_Handler           /* ADC1 Sequence 1		 */
    .long    ADC1SS2_Handler           /* ADC1 Sequence 2		 */
    .long    ADC1SS3_Handler           /* ADC1 Sequence 3		 */
    .long    I2S0_Handler              /* I2S0				 */
    .long    EPI0_Handler              /* External Bus Interface 0	 */
    .long    GPIOJ_Handler             /* GPIO Port J			 */
									 
    .long    GPIOK_Handler             /* GPIO Port K			 */
    .long    GPIOL_Handler             /* GPIO Port L			 */
    .long    SSI2_Handler              /* SSI2 Rx and Tx		 */
    .long    SSI3_Handler              /* SSI3 Rx and Tx		 */
    .long    UART3_Handler             /* UART3 Rx and Tx		 */
    .long    UART4_Handler             /* UART4 Rx and Tx		 */
    .long    UART5_Handler             /* UART5 Rx and Tx		 */
    .long    UART6_Handler             /* UART6 Rx and Tx		 */
    .long    UART7_Handler             /* UART7 Rx and Tx		 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    I2C2_Handler              /* I2C2 Master and Slave		 */
    .long    I2C3_Handler              /* I2C3 Master and Slave		 */
    .long    TIMER4A_Handler           /* Timer 4 subtimer A		 */
    .long    TIMER4B_Handler           /* Timer 4 subtimer B		 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    0                         /* Reserved			 */
    .long    TIMER5A_Handler           /* Timer 5 subtimer A		 */
    .long    TIMER5B_Handler           /* Timer 5 subtimer B		 */
    .long    WTIMER0A_Handler          /* Wide Timer 0 subtimer A	 */
    .long    WTIMER0B_Handler          /* Wide Timer 0 subtimer B	 */
    .long    WTIMER1A_Handler          /* Wide Timer 1 subtimer A	 */
    .long    WTIMER1B_Handler          /* Wide Timer 1 subtimer B	 */
    .long    WTIMER2A_Handler          /* Wide Timer 2 subtimer A	 */
    .long    WTIMER2B_Handler          /* Wide Timer 2 subtimer B	 */
    .long    WTIMER3A_Handler          /* Wide Timer 3 subtimer A	 */
    .long    WTIMER3B_Handler          /* Wide Timer 3 subtimer B	 */
    .long    WTIMER4A_Handler          /* Wide Timer 4 subtimer A	 */
    .long    WTIMER4B_Handler          /* Wide Timer 4 subtimer B	 */
    .long    WTIMER5A_Handler          /* Wide Timer 5 subtimer A	 */
    .long    WTIMER5B_Handler          /* Wide Timer 5 subtimer B	 */
    .long    FPU_Handler               /* FPU				 */
    .long    PECI0_Handler             /* PECI 0			 */
    .long    LPC0_Handler              /* LPC 0				 */
    .long    I2C4_Handler              /* I2C4 Master and Slave		 */
    .long    I2C5_Handler              /* I2C5 Master and Slave		 */
    .long    GPIOM_Handler             /* GPIO Port M			 */
    .long    GPION_Handler             /* GPIO Port N			 */
    .long    QEI2_Handler              /* Quadrature Encoder 2		 */
    .long    FAN0_Handler              /* Fan 0				 */
    .long    0                         /* Reserved			 */
    .long    GPIOP0_Handler            /* GPIO Port P (Summary or P0)	 */
    .long    GPIOP1_Handler            /* GPIO Port P1			 */
    .long    GPIOP2_Handler            /* GPIO Port P2			 */
    .long    GPIOP3_Handler            /* GPIO Port P3			 */
    .long    GPIOP4_Handler            /* GPIO Port P4			 */
    .long    GPIOP5_Handler            /* GPIO Port P5			 */
    .long    GPIOP6_Handler            /* GPIO Port P6			 */
    .long    GPIOP7_Handler            /* GPIO Port P7			 */
    .long    GPIOQ0_Handler            /* GPIO Port Q (Summary or Q0)	 */
    .long    GPIOQ1_Handler            /* GPIO Port Q1			 */
    .long    GPIOQ2_Handler            /* GPIO Port Q2			 */
    .long    GPIOQ3_Handler            /* GPIO Port Q3			 */
    .long    GPIOQ4_Handler            /* GPIO Port Q4			 */
    .long    GPIOQ5_Handler            /* GPIO Port Q5			 */
    .long    GPIOQ6_Handler            /* GPIO Port Q6			 */
    .long    GPIOQ7_Handler            /* GPIO Port Q7			 */
    .long    GPIOR_Handler             /* GPIO Port R			 */
    .long    GPIOS_Handler             /* GPIO Port S			 */
    .long    PMW1_0_Handler            /* PWM 1 Generator 0		 */
    .long    PWM1_1_Handler            /* PWM 1 Generator 1		 */
    .long    PWM1_2_Handler            /* PWM 1 Generator 2		 */
    .long    PWM1_3_Handler            /* PWM 1 Generator 3		 */
    .long    PWM1_FAULT_Handler        /* PWM 1 Fault			 */

    .size    __isr_vector, . - __isr_vector

    .text
    .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

/* Here are two copies of loop implemenations. First one favors code size
 * and the second one favors performance. Default uses the first one.
 * Change to "#if 0" to use the second one */
#if 1
.LC0:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .LC0
#else
    subs    r3, r2
    ble    .LC1
.LC0:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .LC0
.LC1:
#endif

#ifdef __STARTUP_CLEAR_BSS
/*     This part of work usually is done in C library startup code. Otherwise,
 *     define this macro to enable it in this startup.
 *
 *     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      __bss_start__: start of BSS section. Must align to 4
 *      __bss_end__: end of BSS section. Must align to 4
 */
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__

    movs    r0, 0
.LC2:
    cmp     r1, r2
    itt    lt
    strlt   r0, [r1], #4
    blt    .LC2
#endif /* __STARTUP_CLEAR_BSS */

#ifndef __NO_SYSTEM_INIT
    bl    SystemInit
#endif

#ifndef __START
#define __START _start
#endif
    bl    __START
    .pool
    .size Reset_Handler, . - Reset_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_irq_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .type    \handler_name, %function
\handler_name :
    b    .
    .size    \handler_name, . - \handler_name
    .endm

    def_irq_handler    NMI_Handler
    def_irq_handler    HardFault_Handler
    def_irq_handler    MemManage_Handler
    def_irq_handler    BusFault_Handler
    def_irq_handler    UsageFault_Handler
    def_irq_handler    SVC_Handler
    def_irq_handler    DebugMon_Handler
    def_irq_handler    PendSV_Handler
    def_irq_handler    SysTick_Handler
    def_irq_handler    Default_Handler

    def_irq_handler    GPIOA_Handler
    def_irq_handler    GPIOB_Handler
    def_irq_handler    GPIOC_Handler
    def_irq_handler    GPIOD_Handler
    def_irq_handler    GPIOE_Handler
    def_irq_handler    UART0_Handler
    def_irq_handler    UART1_Handler
    def_irq_handler    SSI0_Handler
    def_irq_handler    I2C0_Handler
    def_irq_handler    PMW0_FAULT_Handler
    def_irq_handler    PWM0_0_Handler
    def_irq_handler    PWM0_1_Handler
    def_irq_handler    PWM0_2_Handler
    def_irq_handler    QEI0_Handler
    def_irq_handler    ADC0SS0_Handler
    def_irq_handler    ADC0SS1_Handler
    def_irq_handler    ADC0SS2_Handler
    def_irq_handler    ADC0SS3_Handler
    def_irq_handler    WDT0_Handler
    def_irq_handler    TIMER0A_Handler
    def_irq_handler    TIMER0B_Handler
    def_irq_handler    TIMER1A_Handler
    def_irq_handler    TIMER1B_Handler
    def_irq_handler    TIMER2A_Handler
    def_irq_handler    TIMER2B_Handler
    def_irq_handler    COMP0_Handler
    def_irq_handler    COMP1_Handler
    def_irq_handler    COMP2_Handler
    def_irq_handler    SYSCTL_Handler
    def_irq_handler    FLASH_Handler
    def_irq_handler    GPIOF_Handler
    def_irq_handler    GPIOG_Handler
    def_irq_handler    GPIOH_Handler
    def_irq_handler    UART2_Handler
    def_irq_handler    SSI1_Handler
    def_irq_handler    TIMER3A_Handler
    def_irq_handler    TIMER3B_Handler
    def_irq_handler    I2C1_Handler
    def_irq_handler    QEI1_Handler
    def_irq_handler    CAN0_Handler
    def_irq_handler    CAN1_Handler
    def_irq_handler    CAN2_Handler
    def_irq_handler    ETH0_Handler
    def_irq_handler    HIB_Handler
    def_irq_handler    USB0_Handler
    def_irq_handler    PWM0_3_Handler
    def_irq_handler    UDMA_Handler
    def_irq_handler    UDMAERR_Handler
    def_irq_handler    ADC1SS0_Handler
    def_irq_handler    ADC1SS1_Handler
    def_irq_handler    ADC1SS2_Handler
    def_irq_handler    ADC1SS3_Handler
    def_irq_handler    I2S0_Handler
    def_irq_handler    EPI0_Handler
    def_irq_handler    GPIOJ_Handler
							
    def_irq_handler    GPIOK_Handler
    def_irq_handler    GPIOL_Handler
    def_irq_handler    SSI2_Handler
    def_irq_handler    SSI3_Handler
    def_irq_handler    UART3_Handler
    def_irq_handler    UART4_Handler
    def_irq_handler    UART5_Handler
    def_irq_handler    UART6_Handler
    def_irq_handler    UART7_Handler
    def_irq_handler    I2C2_Handler
    def_irq_handler    I2C3_Handler
    def_irq_handler    TIMER4A_Handler
    def_irq_handler    TIMER4B_Handler
    def_irq_handler    TIMER5A_Handler
    def_irq_handler    TIMER5B_Handler
    def_irq_handler    WTIMER0A_Handler
    def_irq_handler    WTIMER0B_Handler
    def_irq_handler    WTIMER1A_Handler
    def_irq_handler    WTIMER1B_Handler
    def_irq_handler    WTIMER2A_Handler
    def_irq_handler    WTIMER2B_Handler
    def_irq_handler    WTIMER3A_Handler
    def_irq_handler    WTIMER3B_Handler
    def_irq_handler    WTIMER4A_Handler
    def_irq_handler    WTIMER4B_Handler
    def_irq_handler    WTIMER5A_Handler
    def_irq_handler    WTIMER5B_Handler
    def_irq_handler    FPU_Handler
    def_irq_handler    PECI0_Handler
    def_irq_handler    LPC0_Handler
    def_irq_handler    I2C4_Handler
    def_irq_handler    I2C5_Handler
    def_irq_handler    GPIOM_Handler
    def_irq_handler    GPION_Handler
    def_irq_handler    QEI2_Handler
    def_irq_handler    FAN0_Handler
    def_irq_handler    GPIOP0_Handler
    def_irq_handler    GPIOP1_Handler
    def_irq_handler    GPIOP2_Handler
    def_irq_handler    GPIOP3_Handler
    def_irq_handler    GPIOP4_Handler
    def_irq_handler    GPIOP5_Handler
    def_irq_handler    GPIOP6_Handler
    def_irq_handler    GPIOP7_Handler
    def_irq_handler    GPIOQ0_Handler
    def_irq_handler    GPIOQ1_Handler
    def_irq_handler    GPIOQ2_Handler
    def_irq_handler    GPIOQ3_Handler
    def_irq_handler    GPIOQ4_Handler
    def_irq_handler    GPIOQ5_Handler
    def_irq_handler    GPIOQ6_Handler
    def_irq_handler    GPIOQ7_Handler
    def_irq_handler    GPIOR_Handler
    def_irq_handler    GPIOS_Handler
    def_irq_handler    PMW1_0_Handler
    def_irq_handler    PWM1_1_Handler
    def_irq_handler    PWM1_2_Handler
    def_irq_handler    PWM1_3_Handler
    def_irq_handler    PWM1_FAULT_Handler

    .end
