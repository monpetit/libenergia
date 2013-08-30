#pragma once

#ifndef _LM4F_RTC_H
#define _LM4F_RTC_H

// #include <Arduino.h>
#include <driverlib/hibernate.h>
#include <driverlib/sysctl.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define rtc_settime HibernateRTCSet
#define rtc_now HibernateRTCGet

    void rtc_init(void);

#ifdef __cplusplus
}
#endif

#endif /* _LM4F_RTC_H */
