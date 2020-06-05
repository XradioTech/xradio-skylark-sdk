#
# project local config options, override the global config options
#

# ----------------------------------------------------------------------------
# override global config options
# ----------------------------------------------------------------------------
# enable/disable wlan, default to y
export __CONFIG_WLAN := n

# enable/disable xplayer, default to n
export __CONFIG_XPLAYER := y

# enable/disable XIP, default to y
export __CONFIG_XIP := y

export __CONFIG_PSRAM := y
export __CONFIG_PSRAM_CHIP_OPI32 := y

export __CONFIG_MALLOC_TRACE := y
