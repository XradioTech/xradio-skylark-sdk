#
# Rules for building sub-modules
#

# ----------------------------------------------------------------------------
# common rules
# ----------------------------------------------------------------------------
ROOT_PATH := ..

include $(ROOT_PATH)/gcc.mk

# ----------------------------------------------------------------------------
# sub-modules
# ----------------------------------------------------------------------------
SUBDIRS := driver/chip
SUBDIRS += libc
SUBDIRS += image
SUBDIRS += rom

ifeq ($(__CONFIG_OS_FREERTOS), y)
ifneq ($(__CONFIG_ROM_FREERTOS), y)
SUBDIRS += kernel/FreeRTOS
endif
SUBDIRS += kernel/os/FreeRTOS
endif

ifeq ($(__CONFIG_BIN_COMPRESS), y)
ifneq ($(__CONFIG_ROM_XZ), y)
SUBDIRS += xz
endif
endif

NET_SUBDIRS := net/ethernetif
NET_SUBDIRS += net/$(LWIP_DIR)
NET_SUBDIRS += net/ping
NET_SUBDIRS += net/HTTPClient
NET_SUBDIRS += net/$(MBEDTLS_DIR)
NET_SUBDIRS += net/nopoll
NET_SUBDIRS += net/libwebsockets/lib
NET_SUBDIRS += net/mqtt
NET_SUBDIRS += net/shttpd-1.42
NET_SUBDIRS += net/sntp
NET_SUBDIRS += net/udhcp-0.9.8
NET_SUBDIRS += net/cloud/aliyun
NET_SUBDIRS += smartlink
NET_SUBDIRS += sys

AT_SUBDIRS := atcmd

ifeq ($(__CONFIG_PM), y)
SUBDIRS += pm
endif

SUBDIRS += efpg
ifneq ($(__CONFIG_BOOTLOADER), y)
SUBDIRS += driver/component
SUBDIRS += ota
SUBDIRS += console
SUBDIRS += fs/fatfs
SUBDIRS += audio/pcm
SUBDIRS += audio/manager
SUBDIRS += $(NET_SUBDIRS)
SUBDIRS += $(AT_SUBDIRS)
SUBDIRS += cjson
SUBDIRS += util
SUBDIRS += jpeg
endif

# ----------------------------------------------------------------------------
# building rules
# ----------------------------------------------------------------------------
.PHONY: all install clean install_clean $(SUBDIRS) _all _install _clean _install_clean

all:
	$(Q)$(MAKE) _all TARGET=all

install:
	$(Q)$(MAKE) _install TARGET=install

clean:
	$(Q)$(MAKE) _clean TARGET=clean

install_clean:
	$(Q)$(MAKE) _install TARGET=install_clean

$(SUBDIRS):
	$(Q)$(MAKE) $(S) -C $@ $(TARGET)

_all: $(SUBDIRS)

_install: $(SUBDIRS)

_clean: $(SUBDIRS)

_install_clean: $(SUBDIRS)
