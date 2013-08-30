
#ifndef _ENERGIA_CMSIS_PORT_H_
#define _ENERGIA_CMSIS_PORT_H_


#if defined(__CC_ARM) || defined(__CMSIS_RTOS)

	#define GPIOAIntHandler       GPIOA_Handler
	#define GPIOBIntHandler       GPIOB_Handler
	#define GPIOCIntHandler       GPIOC_Handler
	#define GPIODIntHandler       GPIOD_Handler
	#define GPIOEIntHandler       GPIOE_Handler
	#define GPIOFIntHandler       GPIOF_Handler

	#define UARTIntHandler        UART0_Handler
	#define UARTIntHandler1       UART1_Handler
	#define UARTIntHandler2       UART2_Handler
	#define UARTIntHandler3       UART3_Handler
	#define UARTIntHandler4       UART4_Handler
	#define UARTIntHandler5       UART5_Handler
	#define UARTIntHandler6       UART6_Handler
	#define UARTIntHandler7       UART7_Handler

	#define ToneIntHandler        TIMER4A_Handler

	#define WatchdogIntHandler    WDT0_Handler
	#define Timer5IntHandler      TIMER5A_Handler

#elif defined(sourcerygxx)

	#define GPIOAIntHandler       __cs3_isr_gpio_a
	#define GPIOBIntHandler       __cs3_isr_gpio_b
	#define GPIOCIntHandler       __cs3_isr_gpio_c
	#define GPIODIntHandler       __cs3_isr_gpio_d
	#define GPIOEIntHandler       __cs3_isr_gpio_e
	#define GPIOFIntHandler       __cs3_isr_gpio_f

	#define UARTIntHandler        __cs3_isr_uart0
	#define UARTIntHandler1       __cs3_isr_uart1
	#define UARTIntHandler2       __cs3_isr_uart2
	#define UARTIntHandler3       __cs3_isr_UART3
	#define UARTIntHandler4       __cs3_isr_UART4
	#define UARTIntHandler5       __cs3_isr_UART5
	#define UARTIntHandler6       __cs3_isr_UART6
	#define UARTIntHandler7       __cs3_isr_UART7

	#define ToneIntHandler        __cs3_isr_TIMER4A

	#define WatchdogIntHandler    __cs3_isr_watchdog
	#define Timer5IntHandler      __cs3_isr_TIMER5A
#else
#endif

#endif // _ENERGIA_CMSIS_PORT_H_
