#
# project local config options, override the global config options
#

# ----------------------------------------------------------------------------
# override global config options
# ----------------------------------------------------------------------------
# enable/disable xplayer, default to n
export __CONFIG_XPLAYER := n

# enable/disable XIP, default to y
export __CONFIG_XIP := y

# enable/disable PSRAM, default to n
export __CONFIG_PSRAM := y
export __CONFIG_PSRAM_CHIP_OPI32 := y

# enable/disable OTA, default to n
export __CONFIG_OTA := n

# enable/disable JPEG, default to n
export __CONFIG_JPEG := y
export __CONFIG_JPEG_SHAR_SRAM_64K := n
