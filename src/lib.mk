#
# Rules for building library
#

# ----------------------------------------------------------------------------
# common targets and building rules
# ----------------------------------------------------------------------------
INSTALL_PATH ?= $(ROOT_PATH)/lib

.PHONY: all install size clean install_clean config config_clean

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

config:
	@$(Q)cd $(ROOT_PATH) && ./configure.sh

config_clean:
	$(Q)-rm -f $(ROOT_PATH)/.config

# ----------------------------------------------------------------------------
# dependent rules
# ----------------------------------------------------------------------------
DEPS = $(OBJS:.o=.d)
-include $(DEPS)
