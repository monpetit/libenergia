#include <stdint.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/watchdog.h"

typedef void(*wd_handler_t)(void);

#define WDOG_PERIOD_1S        (SysCtlClockGet())
#define WDOG_PERIOD_2S        (WDOG_PERIOD_1S * 2)
#define WDOG_PERIOD_3S        (WDOG_PERIOD_1S * 3)
#define WDOG_PERIOD_4S        (WDOG_PERIOD_1S * 4)
#define WDOG_PERIOD_8S        (WDOG_PERIOD_1S * 8)
#define WDOG_PERIOD_500MS     (WDOG_PERIOD_1S / 2)
#define WDOG_PERIOD_100MS     (WDOG_PERIOD_1S / 10)
#define WDOG_PERIOD_10MS      (WDOG_PERIOD_1S / 100)
#define WDOG_PERIOD_1MS       (WDOG_PERIOD_1S / 1000)


class WatchDog
{
public:
	WatchDog(uint32_t _period = WDOG_PERIOD_1S) {
		wd_handler = NULL;
		period = _period;
	}

	void start() {
		start(period);
	}

	void start(uint32_t _period) {
		period = _period;

		//
		// Enable the peripherals used by this example.
		//
		SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

		//
		// Enable processor interrupts.
		//
		IntMasterEnable();

		//
		// Enable the watchdog interrupt.
		//
		IntEnable(INT_WATCHDOG);

		//
		// Set the period of the watchdog timer.
		//
		WatchdogReloadSet(WATCHDOG0_BASE, _period);

		//
		// Enable reset generation from the watchdog timer.
		//
		WatchdogResetEnable(WATCHDOG0_BASE);

		//
		// Enable the watchdog timer.
		//
		WatchdogEnable(WATCHDOG0_BASE);
	}

	void stop(void) {
		WatchdogResetDisable(WATCHDOG0_BASE);
	    IntDisable(INT_WATCHDOG);
	}

	void reset(void) {
		//
        // Clear the watchdog interrupt.
        //
		WatchdogIntClear(WATCHDOG0_BASE);
	}

	void set_handler(wd_handler_t handler) {
		wd_handler = handler;
	}

	void reset_handler(void) {
		wd_handler = NULL;
	}

	uint32_t period;
	wd_handler_t wd_handler;
};



extern WatchDog wdog;
