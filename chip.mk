#
# chip definition
#

# ----------------------------------------------------------------------------
# chip type
# ----------------------------------------------------------------------------
-include $(ROOT_PATH)/.config

# ----------------------------------------------------------------------------
# chips of arch version 2
# ----------------------------------------------------------------------------
ifeq ($(__CONFIG_CHIP_TYPE), xr872)
  __CONFIG_CHIP_ARCH_VER := 2
  __CONFIG_CHIP_XR872 := y
  CONFIG_SYMBOLS += -D__CONFIG_CHIP_XR872
endif

ifeq ($(__CONFIG_CHIP_TYPE), xr808)
  __CONFIG_CHIP_ARCH_VER := 2
  __CONFIG_CHIP_XR808 := y
  CONFIG_SYMBOLS += -D__CONFIG_CHIP_XR808
endif

CONFIG_SYMBOLS += -D__CONFIG_CHIP_ARCH_VER=$(__CONFIG_CHIP_ARCH_VER)

# ----------------------------------------------------------------------------
# arch and core
# ----------------------------------------------------------------------------
__CONFIG_ARCH_DUAL_CORE := n
ifeq ($(__CONFIG_ARCH_DUAL_CORE), y)
  CONFIG_SYMBOLS += -D__CONFIG_ARCH_DUAL_CORE
endif

__CONFIG_ARCH_APP_CORE := y
ifeq ($(__CONFIG_ARCH_APP_CORE), y)
  CONFIG_SYMBOLS += -D__CONFIG_ARCH_APP_CORE
endif

__CONFIG_ARCH_NET_CORE := n
ifeq ($(__CONFIG_ARCH_NET_CORE), y)
  CONFIG_SYMBOLS += -D__CONFIG_ARCH_NET_CORE
endif

# ----------------------------------------------------------------------------
# cpu
# ----------------------------------------------------------------------------
__CONFIG_CPU_CM4F ?= y
ifeq ($(__CONFIG_CPU_CM4F), y)
  CONFIG_SYMBOLS += -D__CONFIG_CPU_CM4F
endif

# ----------------------------------------------------------------------------
# chip configuration check
# ----------------------------------------------------------------------------
ifneq ($(MAKECMDGOALS), config)
ifneq ($(MAKECMDGOALS), config_clean)

  ifndef __CONFIG_CHIP_TYPE
    __nullstring :=
    $(info ERROR:)
    $(info $(__nullstring)  Chip is not defined!)
    $(info $(__nullstring)  Please run `make config` in your project.)
    $(info $(__nullstring)  Or run `./configure.sh` in the root directory.)
    $(error )
  endif

  ifndef __CONFIG_CHIP_ARCH_VER
    $(error Invalid chip configuration!)
  endif

endif
endif
