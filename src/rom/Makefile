#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ../..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# library and objects
# ----------------------------------------------------------------------------
LIBS := librom.a

ifeq ($(__CONFIG_ROM), y)
  DIRS := .
else
  ROM_DIRS_IGNORE :=
  ifneq ($(__CONFIG_ROM_FREERTOS), y)
    ROM_DIRS_IGNORE += $(shell find ./rom_bin/src/kernel/FreeRTOS -type d)
  endif
  ifneq ($(__CONFIG_ROM_XZ), y)
    ROM_DIRS_IGNORE += ./rom_bin/src/xz
  endif
  ROM_DIRS_ALL := $(shell find ./rom_bin -type d)
  ROM_DIRS := $(filter-out $(ROM_DIRS_IGNORE),$(ROM_DIRS_ALL))
  DIRS := . $(ROM_DIRS)
  INCLUDE_PATHS += -I$(ROOT_PATH)/src/rom/rom_bin/include
endif

SRCS := $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS])))

OBJS := $(addsuffix .o,$(SRCS))

# library make rules
include $(LIB_MAKE_RULES)
