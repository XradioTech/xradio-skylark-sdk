#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common targets and building rules
# ----------------------------------------------------------------------------
INSTALL_PATH ?= $(ROOT_PATH)/lib

.PHONY: all install size clean install_clean

all: $(LIBS)

$(LIBS): $(OBJS)
	$(Q)$(AR) -crs $@ $^

install: $(LIBS)
	$(Q)$(CP) -t $(INSTALL_PATH) $^

size:
	$(Q)$(SIZE) -t $(LIBS)

objdump: $(LIBS)
	$(Q)$(OBJDUMP) -Sdh $< > $(basename $(LIBS)).objdump

clean:
	$(Q)-rm -f $(LIBS) $(OBJS) $(DEPS) *.objdump

install_clean:
	$(Q)-rm -f $(INSTALL_PATH)/$(LIBS)

# ----------------------------------------------------------------------------
# dependent rules
# ----------------------------------------------------------------------------
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
