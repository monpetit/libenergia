#include "HardwareSerial.h"

void
UARTIntHandler5(void)
{
    Serial5.UARTIntHandler();
}

HardwareSerial Serial5(5);
