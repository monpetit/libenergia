/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    RTX_EX1.C
 *      Purpose: Your First RTX example program
 *----------------------------------------------------------------------------
 *
 * Copyright (c) 1999-2009 KEIL, 2009-2013 ARM Germany GmbH
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of ARM  nor the names of its contributors may be used 
 *    to endorse or promote products derived from this software without 
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *---------------------------------------------------------------------------*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <limits.h>

#include <Energia.h>
#include <cmsis_os.h>

#define BOARD_LED_PIN   GREEN_LED
#define INPUT_ANALOG    A0
#define BUFFER_SIZE	256


char buffer[BUFFER_SIZE];
	
osMutexId mutex;
osMutexDef(mutex);	

/* Forward reference */
void vLEDFlashTask(void const *pvParameters);
void vRecvTask(void const *pvParameters);
void vSerialTask(void const *pvParameters);

/* Thread IDs */
osThreadId ledflash_id;
osThreadId recvtask_id;
osThreadId serialtask_id;

/* Thread definitions */
osThreadDef(vLEDFlashTask, osPriorityNormal, 1, 0);
osThreadDef(vRecvTask, osPriorityNormal, 1, 0);
osThreadDef(vSerialTask, osPriorityRealtime, 1, 0);


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

	osMutexWait(mutex, osWaitForever);    
    Serial.print("DATE STRING = [");
    Serial.print(dtstr);
    Serial.println("]");
	osMutexRelease(mutex);    

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
    // rtc.settime(year, month, day, hour, minute, second);
    // SetTimeDate(day, month, year - 2000, hour, minute, second);
}


void vLEDFlashTask(void const *pvParameters)
{
    for (;;) {
        digitalWrite(BOARD_LED_PIN, HIGH);
		osThreadYield();
        osDelay(20);		
        digitalWrite(BOARD_LED_PIN, LOW);
#if 0		
		osMutexWait(mutex, osWaitForever);
		Serial.print("UNSIGNED LONG LONG MAX = ");
		Serial.println(ULONG_LONG_MAX);
		osMutexRelease(mutex);
#endif
		osThreadYield();
        osDelay(1500 - (millis() % 1500));
    }
}

void vRecvTask(void const *pvParameters)
{
    int buffer_index = 0;
    bool taking = false;    

    while (true) {
        if (Serial.available()) {
            osMutexWait(mutex, osWaitForever);
            char cmd = Serial.read();
			osMutexRelease(mutex);

            if (cmd == '{') {
                buffer_index = 0;
                taking = true;                    
            }
            else if (cmd == '}') {
                buffer[buffer_index] = 0;
                taking = false;
				osMutexWait(mutex, osWaitForever);
                Serial.print("input buffer = [");
                Serial.print(buffer);
                Serial.println("]");
				osMutexRelease(mutex);
                setRtcDateTime(buffer);
            }

            if (taking)
                if ((cmd != '{') && (cmd != '}') && (cmd != '\r') && (cmd != '\n'))
                    buffer[buffer_index++] = cmd;
        }
		osThreadYield();
    }
}

void vSerialTask(void const *pvParameters)
{
    unsigned long int count = 0;
    int analog_pin = 15;
    int val = 0;
    pinMode(analog_pin, INPUT_ANALOG);

    bool sensor_error = false;

    int count_at_error = 0;
    int val_at_error = 0;

    for (;;) {
		osMutexWait(mutex, osWaitForever);        
        Serial.print("count = ");
        Serial.println(count++);
		osMutexRelease(mutex);
        val = analogRead(analog_pin);
		osMutexWait(mutex, osWaitForever);        
        Serial.print("analog value = ");
        Serial.println(val);
		osMutexRelease(mutex);

        if (((val < 100) || (3000 < val)) && !sensor_error)
            sensor_error = true;

        if (sensor_error) {
            if (count_at_error == 0) {
                count_at_error = count;
                val_at_error = val;
            }
			osMutexWait(mutex, osWaitForever);                    
            Serial.print("SENSOR ERROR! COUNT = ");
            Serial.print(count_at_error);
            Serial.print(" value = ");
            Serial.println(val_at_error);
			osMutexRelease(mutex);            
        }
        else {
			osMutexWait(mutex, osWaitForever);                    
            Serial.println("SENSOR OK!");
			osMutexRelease(mutex);            
        }

        time_t _t = rtc_now();
        struct tm* t = localtime(&_t);
        Serial.print("SECS = ");
        char tmbuf[128] = {
            0,         };
        sprintf(tmbuf, "%04d-%02d-%02d %02d:%02d:%02d", t->tm_year +1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
        Serial.println(tmbuf);

        Serial.print("system tick = ");
        Serial.println(millis());
        Serial.println();
		osThreadYield();
        osDelay(2000 - (millis() % 2000));
    }
}


void setup()
{
    Serial.begin(38400);    
    
    // initialize the digital pin as an output:
    pinMode(BOARD_LED_PIN, OUTPUT);

	mutex = osMutexCreate(osMutex(mutex));
    
    Serial.println("HELLO VLADIMIR!");

	ledflash_id   = osThreadCreate(osThread(vLEDFlashTask), NULL);
	recvtask_id   = osThreadCreate(osThread(vRecvTask), NULL);
	serialtask_id = osThreadCreate(osThread(vSerialTask), NULL);

	// while (1) ;
}


void loop()
{
    // Insert background code here
}


