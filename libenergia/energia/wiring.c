/*
 ************************************************************************
 *	wiring.c
 *
 *	Arduino core files for MSP430
 *		Copyright (c) 2012 Robert Wessels. All right reserved.
 *
 *
 ***********************************************************************
  Derived from:
  wiring.c - Partial implementation of the Wiring API for the ATmega8.
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
 */
#include "Energia.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include <limits.h>
#include <stdio.h>

void timerInit()
{
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / 1000); //0.125us
    ROM_SysTickEnable();

    //
    //Initialize WTimer4 to be used as time-tracker since beginning of time
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER4); //not tied to launchpad pin
    ROM_TimerConfigure(WTIMER4_BASE, TIMER_CFG_PERIODIC);
#if defined(__CC_ARM)
    ROM_TimerLoadSet64(WTIMER4_BASE, ULLONG_MAX); //start at 0 and count up
#else
    ROM_TimerLoadSet64(WTIMER4_BASE, ULONG_LONG_MAX); //start at 0 and count up
#endif
    ROM_TimerEnable(WTIMER4_BASE, TIMER_A);
}

unsigned long long micros()
{
#if defined(__CC_ARM)
    unsigned long long cycles = ULLONG_MAX - ROM_TimerValueGet64(WTIMER4_BASE);
#else
    unsigned long long cycles = ULONG_LONG_MAX - ROM_TimerValueGet64(WTIMER4_BASE);
#endif
    return (cycles / 80);
}

unsigned long long millis()
{
    return (micros() / 1000);
}

/* Delay for the given number of microseconds.  Assumes a 1, 8 or 16 MHz clock. */
void delayMicroseconds(unsigned int us)
{
    volatile unsigned long elapsedTime;
    unsigned long startTime = HWREG(0xE000E018);
    do {
        elapsedTime = startTime - (HWREG(0xE000E018) & 0x00FFFFFF);
    }
    while (elapsedTime <= us * 80);
}


/* (ab)use the WDT */
void delay(uint32_t milliseconds)
{
    unsigned long i;
    for (i = 0; i < milliseconds; i++) {
        delayMicroseconds(1000);
    }
}

/*
 * for unsinged long long (uint64_t) display...
 */
char* ulltoa(unsigned long long number)
{
#ifdef sourcerygxx
	static char numbuffer[20 + 1] = {0, };         // ulong_long_max = 20 characters + 1 ('\0')
    sprintf(numbuffer, "%llu", number);
    return numbuffer;
#else
    static char buf[8 * sizeof(unsigned long long) + 1]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';

    do {
        unsigned long m = number;
		char c;

        number /= 10;
        c = m - 10 * number;
        *--str = c < 10 ? c + '0' : c + 'A' - 10;
    }
    while (number);

    return str;
#endif

}
