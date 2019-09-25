#
# project local config options, override the global config options
#

# ----------------------------------------------------------------------------
# override global config options
# ----------------------------------------------------------------------------
# enable/disable wlan station mode, default to y
export __CONFIG_WLAN_STA := y

# enable/disable wps for wlan station mode, default to n
export __CONFIG_WLAN_STA_WPS := n

# enable/disable wlan hostap mode, default to y
export __CONFIG_WLAN_AP := y

# enable/disable XIP, default to y
export __CONFIG_XIP := y

# enable/disable OTA, default to n
export __CONFIG_OTA := y
