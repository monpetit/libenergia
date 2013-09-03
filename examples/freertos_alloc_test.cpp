/* ------------------------------------------------
 *  FREERTOS memory allocation test
 * ------------------------------------------------ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include <string>


#include <Energia.h>
#include "EnergiaFreeRTOS.h"

#define pc Serial
#define BOARD_LED_PIN   GREEN_LED

class Person
{
public:
	Person() {
		pc.println("***** PERSON ALLOCATED! *****");
	}

	~Person() {
		pc.println("----- PERSON DEALLOCATED! -----");
	}

	void set_name(const std::string &_name) {
		name = _name;
	}

	std::string & get_name(void) {
		return name;
	}

	std::string name;
};




inline void* _CALLOC(size_t size)
{
	void* p = (char*)pvPortMalloc(size);
	if (p)
		memset(p, 0x00, size);
	return p;
}
#define _FREE(P)	vPortFree(P)
#define _MALLOC(S)	pvPortMalloc(S)

#define _NEW_INSTANCE(P, _CLASS) \
	_CLASS * P = (_CLASS *)_MALLOC(sizeof(_CLASS)); \
	P = new (P) _CLASS();

#define _DEL_INSTANCE(P, _CLASS) \
	P->~_CLASS(); \
	_FREE(P);


char* buffer = NULL;
uint32_t count = 0;

void setup(void)
{
	pc.begin(115200);
	pc.println(">>>>> START <<<<<");
	delay(1000);
}

void loop(void)
{
	buffer = (char*)_CALLOC(256 * sizeof(char));
	if (buffer) {
		sprintf(buffer, "count = %lu", count++);
		pc.println(buffer);
		_FREE(buffer);
	}

	pc.print("[BEFORE ALLOC] xPortGetFreeHeapSize = ");
	pc.println(xPortGetFreeHeapSize());

	// Person *p = (Person*)_MALLOC(sizeof(Person));
	// p = new (p) Person();
	_NEW_INSTANCE(p, Person);

	pc.print("[AFTER ALLOC] xPortGetFreeHeapSize = ");
	pc.println(xPortGetFreeHeapSize());

	p->set_name("Thanks. It seemed counterintuitive to use 'new' when the whole purpose of using TBB's scalable_allocator was to replace new. But I guess it's different because it's placement new that's being used. – Nav Feb 10 '11 at 11:16\r\n"
		"The parameter to allocate() is the number of objects, not the size in bytes. You then call the allocator's construct() function to construct the object."
		"scalable_allocator<SomeClass> sa;\r\n"
		"SomeClass* s = sa.allocate(1);\r\n"
		"sa.construct(s, SomeClass());\r\n"
		"// ...\r\n"
		"하늘이 푸릅니다\r\n"
		"sa.destroy(s);\r\n"
		"sa.deallocate(s);\r\n"
		"If want to use it with a standard library container or other std allocator aware type, simply give it the allocator type.");

	pc.println(p->get_name().data());
	_DEL_INSTANCE(p, Person);

	pc.print("[AFTER FREE] xPortGetFreeHeapSize = ");
	pc.println(xPortGetFreeHeapSize());
	pc.println();

	delay(1000 - (millis() % 1000));
}

