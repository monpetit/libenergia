#include "EnergiaFreeRTOS.h"
#include <limits.h>

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
mutex type semaphore that is used to ensure mutual exclusive access to stdout. */
xSemaphoreHandle xMutex;


static unsigned int led_num;

static void taskA (void* pdata)
{
    pinMode(RED_LED, OUTPUT);

    for (;;) {
        digitalWrite(RED_LED, HIGH);   // set the LED on
        delay(10);              // wait for a second
        digitalWrite(RED_LED, LOW);    // set the LED off

        xSemaphoreTake( xMutex, portMAX_DELAY );
        Serial.print("[A] led_num = ");
        Serial.println(led_num++);
        Serial.print("uint64_t max = ");
        Serial.println(ULONG_LONG_MAX);
        Serial.print("uint32_t max = ");
        Serial.println(ULONG_MAX);
        xSemaphoreGive( xMutex );

        vTaskDelay(2500 - (millis() % 2500));
    }
}

static void taskB (void* pdata)
{
    for (;;) {
        xSemaphoreTake( xMutex, portMAX_DELAY );
        Serial.print("[B] led_num = ");
        Serial.println(led_num++);
        xSemaphoreGive( xMutex );

        vTaskDelay(1000 - (millis() % 1000));
    }
}


static void taskC (void* pdata)
{
    char buffer[] = "HELLO VLADIMIR MONPETIT HAMASKY!";

    for (;;) {
        xSemaphoreTake( xMutex, portMAX_DELAY );
        Serial.print("[C] milliseconds = ");
        Serial.println(millis());

        Serial.println(buffer);
        Serial.println(strlen(buffer));

        xSemaphoreGive( xMutex );

        vTaskDelay(1000 - (millis() % 1000));
    }
}


static void taskD (void* pdata)
{
    for (;;) {
        char* msg = (char*)pvPortMalloc(128);
        if (msg) {
            strcpy(msg, "****** pvPortMalloc() test... ****** \r\n");
            xSemaphoreTake( xMutex, portMAX_DELAY );
            Serial.println(msg);
            Serial.print("xPortGetFreeHeapSize = ");
            Serial.println(xPortGetFreeHeapSize());
            xSemaphoreGive( xMutex );
            vPortFree(msg);
        }
        vTaskDelay(1000 - (millis() % 1000));
    }
}



void setup()
{
    /* Before a semaphore is used it must be explicitly created.  In this example
    a mutex type semaphore is created. */
    xMutex = xSemaphoreCreateMutex();

    Serial.begin(9600);

    /*!< Create four tasks	*/
    xTaskCreate(taskA, (signed char*)"taskA", 200, NULL, 1, NULL );
    xTaskCreate(taskB, (signed char*)"taskB", 200, NULL, 2, NULL );
    xTaskCreate(taskC, (signed char*)"taskC", 200, NULL, 3, NULL );
    xTaskCreate(taskD, (signed char*)"taskD", 200, NULL, 3, NULL );

    /* Start the scheduler so the created tasks start executing. */
    vTaskStartScheduler();

    while (1) {
    }

}

void loop()
{
}
