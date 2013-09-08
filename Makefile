CPREF = arm-none-eabi
CC = ${CPREF}-gcc
CXX = ${CPREF}-g++
#AS = ${CPREF}-as
AS= ${CC} -x assembler-with-cpp
AR = ${CPREF}-ar
LD = ${CPREF}-ld
NM = ${CPREF}-nm
OBJCOPY = ${CPREF}-objcopy
OBJDUMP = ${CPREF}-objdump
READELF = ${CPREF}-readelf
SIZE    = ${CPREF}-size

TARGET = example.elf

LIBENERGIA = libenergia.a

ENERGIA_ROOT = ./libenergia
CMSIS = ${ENERGIA_ROOT}/cmsis
CMSIS_BOOT = ${ENERGIA_ROOT}/cmsis_boot
ENERGIA_LIBDIR = ${ENERGIA_ROOT}/energia

LIBLARIES_ROOT = ./libraries
CMSIS_OS = ${LIBLARIES_ROOT}/cmsis_os
FREERTOS = ${LIBLARIES_ROOT}/FreeRTOS
COOS = ${LIBLARIES_ROOT}/CoOS
CHIBIOS = ${LIBLARIES_ROOT}/ChibiOS

LIBSRCS = \
	${CMSIS_BOOT}/system_LM4F.c \
	${CMSIS_BOOT}/startup_LM4F_GCC.s \
	$(shell ls ${ENERGIA_LIBDIR}/driverlib/*.c) \
	${ENERGIA_LIBDIR}/debug.c \
	${ENERGIA_LIBDIR}/energia_cmsis_port.c \
	${ENERGIA_LIBDIR}/itoa.c \
	${ENERGIA_LIBDIR}/lm4f_rtc.c \
	${ENERGIA_LIBDIR}/random.c \
	${ENERGIA_LIBDIR}/startup_gcc.c \
	${ENERGIA_LIBDIR}/usertimer.c \
	${ENERGIA_LIBDIR}/WInterrupts.c \
	${ENERGIA_LIBDIR}/wiring.c \
	${ENERGIA_LIBDIR}/wiring_analog.c \
	${ENERGIA_LIBDIR}/wiring_digital.c \
	${ENERGIA_LIBDIR}/wiring_pulse.c \
	${ENERGIA_LIBDIR}/wiring_shift.c \
	${ENERGIA_LIBDIR}/energia_main.cpp \
	${ENERGIA_LIBDIR}/HardwareSerial.cpp \
	${ENERGIA_LIBDIR}/HardwareSerial1.cpp \
	${ENERGIA_LIBDIR}/new.cpp \
	${ENERGIA_LIBDIR}/Print.cpp \
	${ENERGIA_LIBDIR}/Stream.cpp \
	${ENERGIA_LIBDIR}/Tone.cpp \
	${ENERGIA_LIBDIR}/wdog.cpp \
	${ENERGIA_LIBDIR}/WMath.cpp \
	${ENERGIA_LIBDIR}/WString.cpp

#	${ENERGIA_LIBDIR}/HardwareSerial2.cpp \
#	${ENERGIA_LIBDIR}/HardwareSerial3.cpp \
#	${ENERGIA_LIBDIR}/HardwareSerial4.cpp \
#	${ENERGIA_LIBDIR}/HardwareSerial5.cpp \
#	${ENERGIA_LIBDIR}/HardwareSerial6.cpp \
#	${ENERGIA_LIBDIR}/HardwareSerial7.cpp \

__LIBSRCS = ${LIBSRCS:.s=.o}
_LIBSRCS = ${__LIBSRCS:.c=.o}
LIBOBJS = ${_LIBSRCS:.cpp=.o}
LIBDEPS = ${LIBOBJS:.o=.d}


#
# KEIL RTX
#
LIBRTX = librtx.a
RTX_SRCS = \
	${CMSIS_OS}/HAL_CM4.s \
	${CMSIS_OS}/SVC_Table.s \
	${CMSIS_OS}/HAL_CM.c \
	${CMSIS_OS}/RTX_Conf_CM.c \
	${CMSIS_OS}/rt_CMSIS.c \
	${CMSIS_OS}/rt_Event.c \
	${CMSIS_OS}/rt_List.c \
	${CMSIS_OS}/rt_Mailbox.c \
	${CMSIS_OS}/rt_MemBox.c \
	${CMSIS_OS}/rt_Memory.c \
	${CMSIS_OS}/rt_Mutex.c \
	${CMSIS_OS}/rt_Robin.c \
	${CMSIS_OS}/rt_Semaphore.c \
	${CMSIS_OS}/rt_System.c \
	${CMSIS_OS}/rt_Task.c \
	${CMSIS_OS}/rt_Time.c \
	${CMSIS_OS}/rt_Timer.c

_RTX_OBJS = ${RTX_SRCS:.s=.o}
RTX_OBJS = ${_RTX_OBJS:.c=.o}
RTX_DEPS = ${RTX_OBJS:.o=.d}


#
# FREERTOS
#
LIBFREERTOS = libfreertos.a
FR_SRCS = \
	${FREERTOS}/EnergiaFreeRTOS.cpp \
	${FREERTOS}/utility/croutine.c \
	${FREERTOS}/utility/list.c \
	${FREERTOS}/utility/queue.c \
	${FREERTOS}/utility/tasks.c \
	${FREERTOS}/utility/timers.c \
	${FREERTOS}/utility/portable/GCC/ARM_CM4F/port.c \
	${FREERTOS}/utility/portable/MemMang/heap_2.c

_FR_SRCS = ${FR_SRCS:.c=.o}
FR_OBJS = ${_FR_SRCS:.cpp=.o}
FR_DEPS = ${FR_OBJS:.o=.d}


#
# COOS
#
LIBCOOS = libcoos.a
CO_SRCS = $(shell ls ${COOS}/kernel/*.c) \
	${COOS}/portable/arch.c \
	${COOS}/portable/GCC/port.c

CO_OBJS = ${CO_SRCS:.c=.o}
CO_DEPS = ${CO_OBJS:.o=.d}


#
# CHIBIOS
#
#LIBSRCS += $(shell ls ${CHIBIOS}/utility/*.c) \
#	${CHIBIOS}/ChibiOS_ARM.c




SRCS = \
	apps/main.cpp

__SRCS = ${SRCS:.s=.o}
_SRCS = ${__SRCS:.c=.o}
OBJS = ${_SRCS:.cpp=.o}
DEPS = ${OBJS:.o=.d}



LDSCRIPT = ${CMSIS_BOOT}/gcc_arm.ld


INCLUDEPATH = -I./apps -I${ENERGIA_LIBDIR} -I${CMSIS} -I${CMSIS_BOOT} -I${CMSIS_OS} \
	-I${FREERTOS} -I${FREERTOS}/utility/include \
	-I${COOS}/kernel -I${COOS}/portable \
#	-I${CHIBIOS} -I${CHIBIOS}/utility

DEFS = -DPART_LM4F120H5QR \
       -DF_CPU=80000000L \
       -DARDUINO=101 \
       -DENERGIA=9 \
       -DTARGET_IS_BLIZZARD_RA2 \
       -D__CMSIS_RTOS \
       -D__CORTEX_M4F \
       -D__FPU_PRESENT=1 \
       -D__VFP_FP__

CFLAGS = -c \
	 -Os \
	 -Wall \
	 -fmessage-length=0 \
	 -ffunction-sections \
	 -fdata-sections \
	 -fsingle-precision-constant \
	 -fno-common \
	 -MMD \
	 -MP \
	 -mcpu=cortex-m4 \
	 -march=armv7e-m \
	 -mthumb \
	 -mfloat-abi=hard \
	 -mfpu=fpv4-sp-d16

ifdef DEBUG
CFLAGS+=-g3 -DDEBUG
CXXFLAGS+=-g3 -DDEBUG
else
CFLAGS+=-DNDEBUG
CXXFLAGS+=-DNDEBUG
endif

CXXFLAGS = ${CFLAGS} \
	   -fno-rtti \
	   -fno-exceptions

ASFLAGS = -c \
	-mcpu=cortex-m4 \
	-march=armv7e-m \
	-mthumb \
	-mfloat-abi=hard \
	-mfpu=fpv4-sp-d16

%.o: %.c
	${CC} ${CFLAGS} ${DEFS} ${INCLUDEPATH} -o "$@" "$<"

%.o: %.cpp
	${CXX} ${CXXFLAGS} ${DEFS} ${INCLUDEPATH} -o "$@" "$<"

%.o: %.s
	${AS} ${ASFLAGS} ${DEFS} ${INCLUDEPATH} -o "$@" "$<"


LIBS = -lenergia -lrtx -lfreertos -lcoos
#-larm_cortexM4lf_math -lgcc -lc -lm

LIBPATH = -L.
LDFLAGS = -T ${LDSCRIPT} \
	  -Wl,--gc-sections \
	  -Xlinker -Map=${TARGET}.map \
	  -mcpu=cortex-m4 \
	  -march=armv7e-m \
	  -mthumb \
	  -mfloat-abi=hard \
	  -mfpu=fpv4-sp-d16



all: ${TARGET}.bin ${LIBENERGIA} ${LIBRTX} ${LIBFREERTOS} ${LIBCOOS}

${TARGET}.bin: ${OBJS} ${LIBENERGIA} ${LIBRTX} ${LIBFREERTOS} ${LIBCOOS}
	${CXX} -o ${TARGET} ${OBJS} ${LIBS} ${LIBPATH} ${LDFLAGS}
	${OBJCOPY} -O binary ${TARGET} "$@"
	${OBJDUMP} -S ${TARGET} > ${TARGET}.lst
	@find . -name "*.o" | xargs ${SIZE} -t
	${SIZE} ${TARGET}


${LIBENERGIA}: ${LIBOBJS}
	$(foreach lobj,${LIBOBJS},${AR} rcs "$@" ${lobj};)

${LIBRTX}: ${RTX_OBJS}
	$(foreach lobj,${RTX_OBJS},${AR} rcs "$@" ${lobj};)

${LIBFREERTOS}: ${FR_OBJS}
	$(foreach lobj,${FR_OBJS},${AR} rcs "$@" ${lobj};)

${LIBCOOS}: ${CO_OBJS}
	$(foreach lobj,${CO_OBJS},${AR} rcs "$@" ${lobj};)



clean:
	rm -f ${TARGET} ${TARGET}.bin ${TARGET}.map ${OBJS} ${DEPS} ${TARGET}.lst
	rm -f ${RTX_OBJS} ${RTX_DEPS}
	rm -f ${FR_OBJS} ${CO_DEPS}
	rm -f ${CO_OBJS} ${CO_DEPS}
	rm -f *.a

install: ${TARGET}.bin
	lm4flash "$<"

