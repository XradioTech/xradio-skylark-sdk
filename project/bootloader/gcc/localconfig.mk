#
# project local config options, override the global config options
#

# ----------------------------------------------------------------------------
# override global config options
# ----------------------------------------------------------------------------
# set y to enable bootloader and disable some features, for bootloader only
export __CONFIG_BOOTLOADER := y

# set n to disable printf float variables
export __CONFIG_LIBC_PRINTF_FLOAT := n

# set n to disable scanf float variables
export __CONFIG_LIBC_SCANF_FLOAT := n

# set n to disable power manager
export __CONFIG_PM := n

# set y to support bin compression
export __CONFIG_BIN_COMPRESS := y

# enable/disable XIP, default to y
export __CONFIG_XIP := n

# set y to support secure boot
export __CONFIG_SECURE_BOOT := n
