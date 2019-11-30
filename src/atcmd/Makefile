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
LIBS := libatcmd.a

DIRS := .

SRCS := $(sort $(basename $(foreach dir,$(DIRS),$(wildcard $(dir)/*.[csS]))))

OBJS := $(addsuffix .o,$(SRCS))

INCLUDE_PATHS += -I../../project/

# library make rules
include $(LIB_MAKE_RULES)
