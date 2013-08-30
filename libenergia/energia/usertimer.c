/*
 * usertimer.c
 *
 *  Created on: 2013. 8. 26.
 *      Author: monpetit
 */

#include "Energia.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "inc/hw_ints.h"
#include <limits.h>

#include "usertimer.h"
#include "energia_cmsis_port.h"

void init_user_timer(void);
// static unsigned long volatile ___ms = 0;

typedef void(*pf)(void);
typedef struct __timer_cbf {
    pf       func;
    uint32_t interval;
    uint32_t next_tick;
    uint8_t  status;
} timer_cbf;

#define __TIMER_CALLBACK_LIST_SIZE	5

timer_cbf timer_callbacks[__TIMER_CALLBACK_LIST_SIZE];


void init_timer_callbacks(void)
{
    int i;
    for (i = 0; i < __TIMER_CALLBACK_LIST_SIZE; i++)
        timer_callbacks[i].func = 0;
}


int find_empty_callback_slot(void)
{
    int i;
    for (i = 0; i < __TIMER_CALLBACK_LIST_SIZE; i++) {
        if (timer_callbacks[i].func != 0)
            return i;
    }
    return -1;
}


void register_timer_callback(int32_t slot, pf _func, uint32_t _interval)
{
    if ((slot < 0) || (__TIMER_CALLBACK_LIST_SIZE - 1 < slot))
        return;

    timer_callbacks[slot].func      = _func;
    timer_callbacks[slot].interval  = _interval;
    timer_callbacks[slot].next_tick = 0;
    timer_callbacks[slot].status    = 0;
}


void unregister_timer_callback(int32_t slot)
{
    if ((slot < 0) || (__TIMER_CALLBACK_LIST_SIZE - 1 < slot))
        return;

    timer_callbacks[slot].func      = 0;
}


void timer_callback_start(int32_t slot)
{
    if ((slot < 0) || (__TIMER_CALLBACK_LIST_SIZE - 1 < slot))
        return;
    if (timer_callbacks[slot].status == 1)
        return;

    timer_callbacks[slot].next_tick = millis() + timer_callbacks[slot].interval;
    timer_callbacks[slot].status    = 1;
}


void timer_callback_stop(int32_t slot)
{
    if ((slot < 0) || (__TIMER_CALLBACK_LIST_SIZE - 1 < slot))
        return;

    timer_callbacks[slot].next_tick = 0;
    timer_callbacks[slot].status    = 0;
}


uint8_t timer_callback_get_status(int32_t slot)
{
    return timer_callbacks[slot].status;
}


void init_user_timer(void)
{
    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Configure the two 32-bit periodic timers.
    //
    ROM_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    ROM_TimerLoadSet(TIMER5_BASE, TIMER_A, ROM_SysCtlClockGet() / 1000);

    //
    // Setup the interrupts for the timer timeouts.
    //
    ROM_IntEnable(INT_TIMER5A);
    ROM_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    ROM_TimerEnable(TIMER5_BASE, TIMER_A);

    //
    // Clear callback list.
    //
    init_timer_callbacks();
}


void Timer5IntHandler(void)
{
    int slot = 0;
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    // ___ms ++;

    //
    // Execute timer callback functions.
    //
    // ROM_IntMasterDisable();
    uint32_t ___ms = millis();
    for (slot = 0; slot < __TIMER_CALLBACK_LIST_SIZE; slot++) {
        if ((timer_callbacks[slot].func != 0) && (timer_callbacks[slot].status == 1)) {
            if (timer_callbacks[slot].next_tick <= ___ms) {
                timer_callbacks[slot].func();
                timer_callbacks[slot].next_tick = timer_callbacks[slot].next_tick + timer_callbacks[slot].interval;
            }
        }
    }
    // ROM_IntMasterEnable();
}
