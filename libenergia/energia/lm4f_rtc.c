#include "lm4f_rtc.h"

/*
 * Set Up Hibernation Module in RTC mode from 32.768kHz oscillator
 */
void rtc_init()
{
#if 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
    HibernateEnableExpClk(SysCtlClockGet());
    HibernateClockSelect(HIBERNATE_CLOCK_SEL_RAW);

    HibernateRTCTrimSet(0x7FFF); // must be called. if you don't set manually the trim register, the counter counts too fast or too slow have a look at page 462 (spsm294e.pdf)
    HibernateGPIORetentionEnable();
    SysCtlDelay(6400); //necessary, without this rtc won't work. don't really know why

    HibernateRTCEnable();
    HibernateIntEnable(HIBERNATE_INT_RTC_MATCH_0);
    HibernateBatCheckStart();
    HibernateWakeSet(HIBERNATE_WAKE_LOW_BAT);
    HibernateLowBatSet(HIBERNATE_LOW_BAT_ABORT | HIBERNATE_LOW_BAT_2_1V);
    // HibernateRTCSet(0);
    HibernateRTCMatch0Set(HibernateRTCGet() + 5);
#else

    SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
    HibernateEnableExpClk(SysCtlClockGet());
    HibernateClockConfig(HIBERNATE_OSC_DISABLE | HIBERNATE_OSC_HIGHDRIVE); //select the external crystal with 24pF filter capacitor (for launchpad lm4f120xl)
    HibernateClockSelect(HIBERNATE_CLOCK_SEL_RAW);

    HibernateRTCTrimSet(0x7FFF); // must be called. if you don't set manually the trim register, the counter counts too fast or too slow have a look at page 462 (spsm294e.pdf)
    HibernateGPIORetentionEnable();
    SysCtlDelay(6400); //necessary, without this rtc won't work. don't really know why
    HibernateWakeSet(HIBERNATE_WAKE_LOW_BAT);
    HibernateLowBatSet(HIBERNATE_LOW_BAT_ABORT | HIBERNATE_LOW_BAT_2_1V);

    HibernateRTCEnable();
    HibernateIntEnable(HIBERNATE_INT_RTC_MATCH_0);
    HibernateBatCheckStart();
    // HibernateRTCSet(0);
    HibernateRTCMatch0Set(HibernateRTCGet() + 5);
#endif
}
