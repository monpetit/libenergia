/*
 * usertimer.h
 *
 *  Created on: 2013. 8. 26.
 *      Author: monpetit
 */

#ifndef USERTIMER_H_
#define USERTIMER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef void(*pf)(void);
    int find_empty_callback_slot(void);
    void register_timer_callback(int32_t slot, pf _func, uint32_t _interval);
    void unregister_timer_callback(int32_t slot);
    void timer_callback_start(int32_t slot);
    void timer_callback_stop(int32_t slot);
    void init_user_timer(void);
    uint8_t timer_callback_get_status(int32_t slot);

#ifdef __cplusplus
} // extern "C"
#endif

#endif /* USERTIMER_H_ */
