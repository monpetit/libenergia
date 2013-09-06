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

LIBSRCS = \
	${CMSIS_BOOT}/system_LM4F.c \
	${CMSIS_BOOT}/startup_LM4F_GCC.s \
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
	${CMSIS_OS}/rt_Timer.c \
	${ENERGIA_LIBDIR}/driverlib/adc.c \
	${ENERGIA_LIBDIR}/driverlib/can.c \
	${ENERGIA_LIBDIR}/driverlib/comp.c \
	${ENERGIA_LIBDIR}/driverlib/cpu.c \
	${ENERGIA_LIBDIR}/driverlib/eeprom.c \
	${ENERGIA_LIBDIR}/driverlib/epi.c \
	${ENERGIA_LIBDIR}/driverlib/ethernet.c \
	${ENERGIA_LIBDIR}/driverlib/fan.c \
	${ENERGIA_LIBDIR}/driverlib/flash.c \
	${ENERGIA_LIBDIR}/driverlib/fpu.c \
	${ENERGIA_LIBDIR}/driverlib/gpio.c \
	${ENERGIA_LIBDIR}/driverlib/hibernate.c \
	${ENERGIA_LIBDIR}/driverlib/i2c.c \
	${ENERGIA_LIBDIR}/driverlib/i2s.c \
	${ENERGIA_LIBDIR}/driverlib/interrupt.c \
	${ENERGIA_LIBDIR}/driverlib/lpc.c \
	${ENERGIA_LIBDIR}/driverlib/mpu.c \
	${ENERGIA_LIBDIR}/driverlib/peci.c \
	${ENERGIA_LIBDIR}/driverlib/pwm.c \
	${ENERGIA_LIBDIR}/driverlib/qei.c \
	${ENERGIA_LIBDIR}/driverlib/ssi.c \
	${ENERGIA_LIBDIR}/driverlib/sysctl.c \
	${ENERGIA_LIBDIR}/driverlib/sysexc.c \
	${ENERGIA_LIBDIR}/driverlib/systick.c \
	${ENERGIA_LIBDIR}/driverlib/timer.c \
	${ENERGIA_LIBDIR}/driverlib/uart.c \
	${ENERGIA_LIBDIR}/driverlib/udma.c \
	${ENERGIA_LIBDIR}/driverlib/usb.c \
	${ENERGIA_LIBDIR}/driverlib/watchdog.c \
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

#
# FREERTOS
#
LIBSRCS += \
	${FREERTOS}/EnergiaFreeRTOS.cpp \
	${FREERTOS}/utility/croutine.c \
	${FREERTOS}/utility/list.c \
	${FREERTOS}/utility/queue.c \
	${FREERTOS}/utility/tasks.c \
	${FREERTOS}/utility/timers.c \
	${FREERTOS}/utility/portable/GCC/ARM_CM4F/port.c \
	${FREERTOS}/utility/portable/MemMang/heap_2.c

#
# COOS
#
LIBSRCS += $(shell ls ${COOS}/kernel/*.c) \
	${COOS}/portable/arch.c \
	${COOS}/portable/GCC/port.c


__LIBSRCS = ${LIBSRCS:.s=.o}
_LIBSRCS = ${__LIBSRCS:.c=.o}
LIBOBJS = ${_LIBSRCS:.cpp=.o}
LIBDEPS = ${LIBOBJS:.o=.d}	


SRCS = \
	apps/main.cpp

__SRCS = ${SRCS:.s=.o}
_SRCS = ${__SRCS:.c=.o}
OBJS = ${_SRCS:.cpp=.o}
DEPS = ${OBJS:.o=.d}	


LDSCRIPT = ${CMSIS_BOOT}/gcc_arm.ld


INCLUDEPATH = -I./apps -I${ENERGIA_LIBDIR} -I${CMSIS} -I${CMSIS_BOOT} -I${CMSIS_OS} \
	-I${FREERTOS} -I${FREERTOS}/utility/include \
	-I${COOS}/kernel -I${COOS}/portable

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
else
CFLAGS+=-DNDEBUG
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


LIBS = -lenergia
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



all: ${TARGET}.bin ${LIBENERGIA}

${TARGET}.bin: ${OBJS} ${LIBENERGIA}
	${CXX} -o ${TARGET} ${OBJS} ${LIBS} ${LIBPATH} ${LDFLAGS}
	${OBJCOPY} -O binary ${TARGET} "$@"
	${OBJDUMP} -S ${TARGET} > ${TARGET}.lst
	@find . -name "*.o" | xargs ${SIZE} -t
	${SIZE} ${TARGET}


${LIBENERGIA}: ${LIBOBJS}
	$(foreach lobj,${LIBOBJS},${AR} rcs "$@" ${lobj};)

clean:
	rm -f ${TARGET} ${TARGET}.bin ${TARGET}.map ${OBJS} ${DEPS} ${TARGET}.lst ${LIBOBJS} ${LIBDEPS} ${LIBENERGIA}

install: ${TARGET}.bin
	lm4flash "$<"

