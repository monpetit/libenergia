#include "energia_cmsis_port.h"
#include <stdio.h>

/*
 * create some overridable default signal handlers
 */
__attribute__((weak)) void I2CIntHandler(void) {}


void I2C0_Handler(void)
{
    I2CIntHandler();
}

void I2C1_Handler(void)
{
    I2CIntHandler();
}

void I2C2_Handler(void)
{
    I2CIntHandler();
}

void I2C3_Handler(void)
{
    I2CIntHandler();
}


#if 0
/**
 * _sbrk - newlib memory allocation routine
 */
typedef char *caddr_t;

caddr_t _sbrk (int incr)
{
    double current_sp;
    extern char end asm ("end"); /* Defined by linker */
    static char * heap_end;
    char * prev_heap_end;

    if (heap_end == NULL) {
        heap_end = &end; /* first ram address after bss and data */
    }

    prev_heap_end = heap_end;

    // simplistic approach to prevent the heap from corrupting the stack
    // TBD: review for alternatives
    if ( heap_end + incr < (caddr_t)&current_sp ) {
        heap_end += incr;
        return (caddr_t) prev_heap_end;
    }
    else {
        return NULL;
    }
}

int _exit(int _code) 
{
    return _code;
}
#endif
