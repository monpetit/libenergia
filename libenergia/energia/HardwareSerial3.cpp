#include "HardwareSerial.h"

void
UARTIntHandler3(void)
{
    Serial3.UARTIntHandler();
}

HardwareSerial Serial3(3);
