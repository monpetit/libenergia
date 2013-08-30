#include "HardwareSerial.h"

void
UARTIntHandler6(void)
{
    Serial6.UARTIntHandler();
}

HardwareSerial Serial6(6);
