#include "HardwareSerial.h"

void
UARTIntHandler4(void)
{
    Serial4.UARTIntHandler();
}

HardwareSerial Serial4(4);
