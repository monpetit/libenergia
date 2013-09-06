#include <Energia.h>
#include <coocox.h>


/*---------------------------- Symbol Define -------------------------------*/
#define STACK_SIZE_TASKA 128              /*!< Define "taskA" task size */
#define STACK_SIZE_TASKB 128              /*!< Define "taskA" task size */
#define STACK_SIZE_TASKC 128              /*!< Define "taskA" task size */

/*---------------------------- Variable Define -------------------------------*/
OS_STK     taskA_stk[STACK_SIZE_TASKA];	  /*!< Define "taskA" task stack */
OS_STK     taskB_stk[STACK_SIZE_TASKB];	  /*!< Define "taskB" task stack */
OS_STK     taskC_stk[STACK_SIZE_TASKC];	  /*!< Define "taskC" task stack */

OS_MutexID mutex;


void taskA (void* pdata)
{
    U32 count = 0;
    char* buffer;

    for (;;) {
        buffer = (char*)CoKmalloc(128 * sizeof(char));
        sprintf(buffer, "HELLO VLADIMIR HAMASKY...");
        CoEnterMutexSection(mutex); /* Enter critical region */
        Serial.print("TASK A\t ");
        Serial.print(buffer);
        Serial.print("\t");
        Serial.println(count++);
        CoLeaveMutexSection(mutex); /* Exit critical region */
        CoKfree(buffer);
        CoTimeDelay(0, 0, 0, 500);
    }
}


void taskB (void* pdata)
{
    pinMode(RED_LED, OUTPUT);

    for (;;) {
        digitalWrite(RED_LED, HIGH);
        CoTimeDelay(0, 0, 0, 20);
        digitalWrite(RED_LED, LOW);
        CoTimeDelay(0, 0, 1, 980);
    }
}


void taskC (void* pdata)
{
    volatile unsigned int dms;
    volatile unsigned int ms;

    for (;;) {
        CoEnterMutexSection(mutex); /* Enter critical region */
        Serial.print("TASK C\t systick = ");
        ms = millis();
        dms = 1000 - (ms % 1000);
        Serial.println(ms);
        CoLeaveMutexSection(mutex); /* Exit critical region */
        CoTimeDelay(0, 0, 0, dms);
    }
}


void setup()
{
    CoInitOS ();				 /*!< Initial CooCox CoOS          */

    /* Create a mutex */
    mutex = CoCreateMutex();

    Serial.begin(9600);

    /*!< Create three tasks	*/
    CoCreateTask (taskA, 0, 0, &taskA_stk[STACK_SIZE_TASKA - 1], STACK_SIZE_TASKA);
    CoCreateTask (taskB, 0, 1, &taskB_stk[STACK_SIZE_TASKB - 1], STACK_SIZE_TASKB);
    CoCreateTask (taskC, 0, 2, &taskC_stk[STACK_SIZE_TASKC - 1], STACK_SIZE_TASKC);
    CoStartOS ();			    /*!< Start multitask	           */

}

void loop() {}

