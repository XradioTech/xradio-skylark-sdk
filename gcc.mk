#
# Common rules for GCC Makefile
#

# ----------------------------------------------------------------------------
# cross compiler
# ----------------------------------------------------------------------------
CC_DIR := ~/tools/gcc-arm-none-eabi-4_9-2015q2/bin
CC_PREFIX := $(CC_DIR)/arm-none-eabi-

AS      := $(CC_PREFIX)as
CC      := $(CC_PREFIX)gcc
CPP     := $(CC_PREFIX)g++
LD      := $(CC_PREFIX)ld
NM      := $(CC_PREFIX)nm
AR      := $(CC_PREFIX)ar
OBJCOPY := $(CC_PREFIX)objcopy
OBJDUMP := $(CC_PREFIX)objdump
SIZE    := $(CC_PREFIX)size
STRIP   := $(CC_PREFIX)strip

# ----------------------------------------------------------------------------
# tools
# ----------------------------------------------------------------------------
CP := cp

# $(MKIMAGE) is a tool for creating image
ifeq ($(shell uname -o), Cygwin)
  MKIMAGE := mkimage.exe
else
  MKIMAGE := mkimage
endif

# ----------------------------------------------------------------------------
# global configuration
# ----------------------------------------------------------------------------
include $(ROOT_PATH)/config.mk

# ----------------------------------------------------------------------------
# options
# ----------------------------------------------------------------------------
QUIET ?= n
OPTIMIZE := y
MDK_DBG_EN := y
HARDFP := n

# building display
ifeq ($(QUIET), y)
  Q := @
  S := -s
endif

ifeq ($(OPTIMIZE), y)
  OPTIMIZE_FLAG := -Os -DNDEBUG
else
  OPTIMIZE_FLAG := -O0 -DDEBUG
endif

ifeq ($(MDK_DBG_EN), y)
  DBG_FLAG := -gdwarf-2
else
  DBG_FLAG := -g
endif

ifeq ($(HARDFP), y)
  FLOAT_ABI := hard
else
  FLOAT_ABI := softfp
endif

# ----------------------------------------------------------------------------
# flags for compiler and linker
# ----------------------------------------------------------------------------
# CPU/FPU options
ifeq ($(__CONFIG_CPU_CM4F), y)
  CPU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=$(FLOAT_ABI)
else
  CPU := -mcpu=cortex-m3 -mthumb
endif

CC_FLAGS = $(CPU) -c $(DBG_FLAG) -fno-common -fmessage-length=0 \
	-fno-exceptions -ffunction-sections -fdata-sections -fomit-frame-pointer \
	-Wall -Werror -Wpointer-arith -Wno-error=unused-function \
	-MMD -MP $(OPTIMIZE_FLAG)

LD_FLAGS = $(CPU) -Wl,--gc-sections --specs=nano.specs \
	-Wl,-Map=$(basename $@).map,--cref

# config symbols
CC_SYMBOLS = $(CONFIG_SYMBOLS)
AS_SYMBOLS = $(CONFIG_SYMBOLS)

ifeq ($(__CONFIG_LIBC_PRINTF_FLOAT), y)
  LD_FLAGS += -u _printf_float
endif

ifeq ($(__CONFIG_LIBC_SCANF_FLOAT), y)
  LD_FLAGS += -u _scanf_float
endif

LD_FLAGS += -Wl,--wrap,main
LD_FLAGS += -Wl,--wrap,exit

ifneq ($(__CONFIG_MALLOC_USE_STDLIB), y)
LD_FLAGS += -Wl,--wrap,malloc
LD_FLAGS += -Wl,--wrap,realloc
LD_FLAGS += -Wl,--wrap,free
else
ifeq ($(__CONFIG_MIX_HEAP_MANAGE), y)
LD_FLAGS += -Wl,--wrap,malloc
LD_FLAGS += -Wl,--wrap,realloc
LD_FLAGS += -Wl,--wrap,calloc
LD_FLAGS += -Wl,--wrap,free
endif
endif

LD_FLAGS += -Wl,--wrap,_malloc_r
LD_FLAGS += -Wl,--wrap,_realloc_r
LD_FLAGS += -Wl,--wrap,_free_r

LD_FLAGS += -Wl,--wrap,gettimeofday
LD_FLAGS += -Wl,--wrap,settimeofday
LD_FLAGS += -Wl,--wrap,time

ifeq ($(__CONFIG_LIBC_WRAP_STDIO), y)
LD_FLAGS += -Wl,--wrap,printf
LD_FLAGS += -Wl,--wrap,vprintf
LD_FLAGS += -Wl,--wrap,puts
LD_FLAGS += -Wl,--wrap,fprintf
LD_FLAGS += -Wl,--wrap,vfprintf
LD_FLAGS += -Wl,--wrap,fputs
LD_FLAGS += -Wl,--wrap,putchar
LD_FLAGS += -Wl,--wrap,putc
LD_FLAGS += -Wl,--wrap,fputc
LD_FLAGS += -Wl,--wrap,fflush
endif

LD_FLAGS += -Wl,--wrap,memcpy
LD_FLAGS += -Wl,--wrap,memset
LD_FLAGS += -Wl,--wrap,memmove

# standard libraries
LD_SYS_LIBS := -lstdc++ -lsupc++ -lm -lc -lgcc

# include path
INCLUDE_ROOT_PATH := $(ROOT_PATH)/include
INCLUDE_PATHS = -I$(INCLUDE_ROOT_PATH)
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/libc
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/driver/cmsis

ifeq ($(__CONFIG_OS_FREERTOS), y)
  ifeq ($(__CONFIG_OS_FREERTOS_VER), 80203)
    INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/kernel/FreeRTOS/FreeRTOSv8.2.3
    ifeq ($(__CONFIG_CPU_CM4F), y)
      INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/kernel/FreeRTOS/FreeRTOSv8.2.3/portable/GCC/ARM_CM4F
    else
      INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/kernel/FreeRTOS/FreeRTOSv8.2.3/portable/GCC/ARM_CM3
    endif
  else ifeq ($(__CONFIG_OS_FREERTOS_VER), 100201)
    INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/kernel/FreeRTOS/FreeRTOSv10.2.1
    ifeq ($(__CONFIG_CPU_CM4F), y)
      INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/kernel/FreeRTOS/FreeRTOSv10.2.1/portable/GCC/ARM_CM4F
    else
      INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/kernel/FreeRTOS/FreeRTOSv10.2.1/portable/GCC/ARM_CM3
    endif
  endif
endif

ifeq ($(__CONFIG_LWIP_V1), y)
  LWIP_DIR := lwip-1.4.1
else
  LWIP_DIR := lwip-2.0.3
endif

ifeq ($(__CONFIG_MBEDTLS_VER), 0x02020000)
  MBEDTLS_DIR := mbedtls-2.2.0
else ifeq ($(__CONFIG_MBEDTLS_VER), 0x02100000)
  MBEDTLS_DIR := mbedtls-2.16.0
endif

INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net/$(LWIP_DIR)
INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net/$(MBEDTLS_DIR)
ifeq ($(__CONFIG_LWIP_V1), y)
  INCLUDE_PATHS += -I$(INCLUDE_ROOT_PATH)/net/$(LWIP_DIR)/ipv4
endif

# platform relative path
PLATFORM_RELATIVE_PATH := xradio_v$(__CONFIG_CHIP_ARCH_VER)

ifeq ($(__CONFIG_PLATFORM_FPGA), y)
PLATFORM_RELATIVE_PATH := $(PLATFORM_RELATIVE_PATH)/fpga
endif

ifeq ($(__CONFIG_ETF), y)
PLATFORM_RELATIVE_PATH := $(PLATFORM_RELATIVE_PATH)/etf
endif

# ----------------------------------------------------------------------------
# common makefile for library and project
# ----------------------------------------------------------------------------
LIB_MAKE_RULES := $(ROOT_PATH)/src/lib.mk
PRJ_MAKE_RULES := $(ROOT_PATH)/project/project.mk

# ----------------------------------------------------------------------------
# common rules of compiling objects
# ----------------------------------------------------------------------------
%.o: %.asm
	$(Q)$(CC) $(CPU) $(AS_SYMBOLS) -c -x assembler-with-cpp -o $@ $<

%.o: %.s
	$(Q)$(CC) $(CPU) $(AS_SYMBOLS) -c -x assembler-with-cpp -o $@ $<

%.o: %.S
	$(Q)$(CC) $(CPU) $(AS_SYMBOLS) -c -x assembler-with-cpp -o $@ $<

%.o: %.c
	$(Q)$(CC) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99 $(INCLUDE_PATHS) -o $@ $<

%.o: %.cpp
	$(Q)$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu++98 -fno-rtti $(INCLUDE_PATHS) -o $@ $<
