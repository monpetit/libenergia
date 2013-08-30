#include "wdog.h"
#include "energia_cmsis_port.h"


#ifdef __cplusplus
extern "C" {
#endif

    void WatchdogIntHandler(void)
    {
		if (wdog.wd_handler)
			wdog.wd_handler();
    }

#ifdef __cplusplus
}
#endif


WatchDog wdog;
