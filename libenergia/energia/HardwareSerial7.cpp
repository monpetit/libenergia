#include "HardwareSerial.h"

void
UARTIntHandler7(void)
{
    Serial7.UARTIntHandler();
}

HardwareSerial Serial7(7);
