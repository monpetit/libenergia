#include <Energia.h>


int status = 0;
int count = 0;

void my_wdog_handler(void)
{
	// wdog.reset();

	status = 1 - status;
	digitalWrite(RED_LED, status);
	Serial.println("*");
	delay(20);
}

void setup()
{
	randomSeed(rtc_now());
    pinMode(RED_LED, OUTPUT);
    
    Serial.begin(9600);
    Serial.println("HELLO DOGMA!");
	wdog.set_handler(my_wdog_handler);
	wdog.start(WDOG_PERIOD_2S);
}


void loop()
{
    Serial.print("count = ");
    Serial.println(count++);
    Serial.print("sys tick = ");
    Serial.println(millis());

    wdog.reset();

    if (millis() >= 7000) {
        delay(random(1000, 2100));
    }
    else
        delay(1000 - (millis() % 1000));
}


