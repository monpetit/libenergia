#include "HardwareSerial.h"

void
UARTIntHandler1(void)
{
    Serial1.UARTIntHandler();
}

HardwareSerial Serial1(1);
