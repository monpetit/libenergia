#include "HardwareSerial.h"

void
UARTIntHandler2(void)
{
    Serial2.UARTIntHandler();
}

HardwareSerial Serial2(2);
