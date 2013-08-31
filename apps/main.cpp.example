#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "EnergiaFreeRTOS.h"

#define BOARD_LED_PIN   GREEN_LED
#define INPUT_ANALOG    A0
#define BUFFER_SIZE	256


char buffer[BUFFER_SIZE];
xSemaphoreHandle xMutex;


int cnv_str_to_int(int & ri, char* src, char sep)
{
    int lindex = 0;
    char numbuf[5];

    while (src[ri] != sep)
        numbuf[lindex++] = src[ri++];
    numbuf[lindex] = 0;
    ri++;

    return atoi(numbuf);
}


void setRtcDateTime(char* dtstr)
{
    int year, month, day, hour, minute, second;

    xSemaphoreTake(xMutex, portMAX_DELAY);
    Serial.print("DATE STRING = [");
    Serial.print(dtstr);
    Serial.println("]");
    xSemaphoreGive(xMutex);

    int rindex = 0;
    year = cnv_str_to_int(rindex, dtstr, '-');
    month = cnv_str_to_int(rindex, dtstr, '-');
    day = cnv_str_to_int(rindex, dtstr, ' ');
    hour = cnv_str_to_int(rindex, dtstr, ':');
    minute = cnv_str_to_int(rindex, dtstr, ':');
    second = cnv_str_to_int(rindex, dtstr, ':');

    struct tm newtime;
    newtime.tm_year = year - 1900;
    newtime.tm_mon = month - 1;
    newtime.tm_mday = day;
    newtime.tm_hour = hour;
    newtime.tm_min = minute;
    newtime.tm_sec = second;
    rtc_settime(mktime(&newtime));
}


static void vLEDFlashTask(void *pvParameters)
{
    for (;;) {
        digitalWrite(BOARD_LED_PIN, HIGH);
        vTaskDelay(20);
        digitalWrite(BOARD_LED_PIN, LOW);
        vTaskDelay(2000 - (/*xTaskGetTickCount*/millis() % 2000));
    }
}

static void vRecvTask(void *pvParameters)
{
    int buffer_index = 0;
    bool taking = false;

    while (true) {
        if (Serial.available()) {
            xSemaphoreTake(xMutex, portMAX_DELAY);
            char cmd = Serial.read();
            xSemaphoreGive(xMutex);

            if (cmd == '{') {
                buffer_index = 0;
                taking = true;
            }
            else if (cmd == '}') {
                buffer[buffer_index] = 0;
                taking = false;
                xSemaphoreTake(xMutex, portMAX_DELAY);
                Serial.print("input buffer = [");
                Serial.print(buffer);
                Serial.println("]");
                xSemaphoreGive(xMutex);
                setRtcDateTime(buffer);
            }

            if (taking)
                if ((cmd != '{') && (cmd != '}') && (cmd != '\r') && (cmd != '\n'))
                    buffer[buffer_index++] = cmd;
        }

    }
}

static void vSerialTask(void *pvParameters)
{
    unsigned long int count = 0;
    int analog_pin = 15;
    int val = 0;
    pinMode(analog_pin, INPUT_ANALOG);

    bool sensor_error = false;

    int count_at_error = 0;
    int val_at_error = 0;

    for (;;) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        Serial.print("count = ");
        Serial.println(count++);
        xSemaphoreGive(xMutex);
        val = analogRead(analog_pin);
        xSemaphoreTake(xMutex, portMAX_DELAY);
        Serial.print("analog value = ");
        Serial.println(val);
        xSemaphoreGive(xMutex);

        if (((val < 100) || (3000 < val)) && !sensor_error)
            sensor_error = true;

        if (sensor_error) {
            if (count_at_error == 0) {
                count_at_error = count;
                val_at_error = val;
            }
            xSemaphoreTake(xMutex, portMAX_DELAY);
            Serial.print("SENSOR ERROR! COUNT = ");
            Serial.print(count_at_error);
            Serial.print(" value = ");
            Serial.println(val_at_error);
            xSemaphoreGive(xMutex);
        }
        else {
            xSemaphoreTake(xMutex, portMAX_DELAY);
            Serial.println("SENSOR OK!");
            xSemaphoreGive(xMutex);
        }

        time_t _t = rtc_now();
        struct tm* t = gmtime(&_t);
        Serial.print("SECS = ");
        char tmbuf[128] = {
            0,
        };
        sprintf(tmbuf, "%04d-%02d-%02d %02d:%02d:%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
        Serial.println(tmbuf);

        Serial.print("system tick = ");
        Serial.println(/*xTaskGetTickCount*/millis());
        Serial.println();
        vTaskDelay(2000 - (/*xTaskGetTickCount*/millis() % 2000));
    }
}


void setup()
{
    Serial.begin(38400);

    // initialize the digital pin as an output:
    pinMode(BOARD_LED_PIN, OUTPUT);

    xMutex = xSemaphoreCreateMutex();

    Serial.println("HELLO VLADIMIR!");

    xTaskCreate(vLEDFlashTask,
                (signed portCHAR *)"Task1",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);


    xTaskCreate(vSerialTask,
                (signed portCHAR *)"Task2",
                512,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);


    xTaskCreate(vRecvTask,
                (signed portCHAR *)"Task3",
                512,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    vTaskStartScheduler();
}


void loop()
{
    // Insert background code here
}
