#include "Energia.h"

#define BOARD_LED_PIN   GREEN_LED

void test1(void)
{
    digitalWrite(BOARD_LED_PIN, HIGH);
    delay(20);
    digitalWrite(BOARD_LED_PIN, LOW);
}

void test2(void)
{
    Serial.println("TEST [2] TASK");
    Serial.print("now = ");
    Serial.println(millis());
}

void test3(void)
{
    Serial.println("\t\tTEST [3] TASK");
    Serial.print("\t\tnow = ");
    Serial.println(millis());
}


void setup()
{
    // initialize the digital pin as an output:
    pinMode(BOARD_LED_PIN, OUTPUT);

    init_user_timer();
    register_timer_callback(0, test1, 2000);	
    register_timer_callback(1, test2, 500);	
    register_timer_callback(2, test3, 1500);

    
    Serial.begin(9600);    

    timer_callback_start(0);
    delay(30);
    timer_callback_start(1);
    delay(30);
    timer_callback_start(2);
}

void loop()
{
#if 0
	char __buffer[1024] = {0, };
	sprintf(__buffer, "hello vlaidmir monpetit hamasky\r\n");
	//Serial.println("hello vlaidmir monpetit hamasky\r\n");
	Serial.println(__buffer);
#endif
	delay(1000);
    if ((millis() > 30000) && (millis() <= 60000)) {
        for (int i = 1; i < 3; i++) 
            if (timer_callback_get_status(i) == 1) {
                timer_callback_stop(i);
            }
    }
    if ((millis() > 60000) && (millis() <= 90000)) {
        for (int i = 1; i < 3; i++) 
            if (timer_callback_get_status(i) == 0) {
                timer_callback_start(i);
                delay(30);
            }
    }
#if 0
    if (millis() > 90000) {
        for (int i = 1; i < 3; i++)
            if (timer_callback_get_status(i) == 1) {
                timer_callback_stop(i);
            }
    }
#endif
}


