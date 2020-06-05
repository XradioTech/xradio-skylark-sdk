#
# global config options
#

# ----------------------------------------------------------------------------
# config options
# ----------------------------------------------------------------------------
include $(ROOT_PATH)/chip.mk

# redefine int32_t to signed int, but not signed long
__CONFIG_LIBC_REDEFINE_GCC_INT32_TYPE ?= y

# support printf float variables
__CONFIG_LIBC_PRINTF_FLOAT ?= y

# support scanf float variables
__CONFIG_LIBC_SCANF_FLOAT ?= y

# wrap standard input/output/error functions
__CONFIG_LIBC_WRAP_STDIO ?= y

# heap managed by stdlib
__CONFIG_MALLOC_USE_STDLIB ?= y

# trace heap memory usage and error when using malloc, free, etc.
__CONFIG_MALLOC_TRACE ?= n

# trace psram heap memory usage and error when using psram_malloc, psram_free, etc.
__CONFIG_PSRAM_MALLOC_TRACE ?= n

# os
__CONFIG_OS_FREERTOS ?= y

ifeq ($(__CONFIG_OS_FREERTOS), y)
#   - 80203: FreeRTOS 8.2.3
#   - 100201: FreeRTOS 10.2.1
__CONFIG_OS_FREERTOS_VER ?= 80203
endif

# lwIP
#   - y: lwIP 1.4.1, support IPv4 stack only
#   - n: lwIP 2.x.x, support dual IPv4/IPv6 stack
__CONFIG_LWIP_V1 ?= y

# mbed TLS
#   - 0x02020000: mbed TLS 2.2.0
#   - 0x02100000: mbed TLS 2.16.0
__CONFIG_MBEDTLS_VER ?= 0x02100000

# mbuf implementation mode
#   - mode 0: continuous memory allocated from heap
#   - mode 1: continuous memory allocated from lwip pbuf
__CONFIG_MBUF_IMPL_MODE ?= 0

# wlan
__CONFIG_WLAN ?= y

# wlan station mode
__CONFIG_WLAN_STA ?= y

# wlan hostap mode
__CONFIG_WLAN_AP ?= y

# wlan monitor mode
__CONFIG_WLAN_MONITOR ?= y

# wps of wlan station mode
__CONFIG_WLAN_STA_WPS ?= n

# wlan ETF test
__CONFIG_ETF ?= n

# wlan ETF test using ETF command line interface
#   - y: using ETF command line interface
#   - n: using ETF GUI exec on PC
__CONFIG_ETF_CLI ?= n

# Wi-Fi Certification of WFA
__CONFIG_WIFI_CERTIFIED ?= y

# XIP
__CONFIG_XIP ?= y

# psram
__CONFIG_PSRAM ?= n
ifeq ($(__CONFIG_PSRAM), y)
  __CONFIG_PSRAM_CHIP_OPI32 ?= y
else
  __CONFIG_PSRAM_CHIP_OPI32 ?= n
endif
__CONFIG_PSRAM_CHIP_OPI64 ?= n
__CONFIG_PSRAM_CHIP_SQPI ?= n

# enable section attribute macros "__xip_xxx", eg. __xip_text
ifeq ($(__CONFIG_XIP), y)
  __CONFIG_SECTION_ATTRIBUTE_XIP ?= y
else
  __CONFIG_SECTION_ATTRIBUTE_XIP ?= n
endif

# enable section attribute macros "__nonxip_xxx", eg. __nonxip_text
ifeq ($(__CONFIG_XIP), y)
  __CONFIG_SECTION_ATTRIBUTE_NONXIP ?= y
else
  __CONFIG_SECTION_ATTRIBUTE_NONXIP ?= n
endif

# enable section attribute macros "__sram_xxx", eg. __sram_text
__CONFIG_SECTION_ATTRIBUTE_SRAM ?= y

# enable section attribute macros "__psram_xxx", eg. __psram_text
ifeq ($(__CONFIG_PSRAM), y)
  __CONFIG_SECTION_ATTRIBUTE_PSRAM ?= y
else
  __CONFIG_SECTION_ATTRIBUTE_PSRAM ?= n
endif

# rom
ifeq ($(__CONFIG_CHIP_ARCH_VER), 1)
  __CONFIG_ROM ?= n
else
  __CONFIG_ROM ?= y
endif

# rom of FreeRTOS
ifeq ($(__CONFIG_ROM), y)
  ifeq ($(__CONFIG_OS_FREERTOS_VER), 80203)
    __CONFIG_ROM_FREERTOS ?= y
  else
    __CONFIG_ROM_FREERTOS ?= n
  endif
else
  __CONFIG_ROM_FREERTOS ?= n
endif

# rom of xz
ifeq ($(__CONFIG_ROM), y)
  __CONFIG_ROM_XZ ?= y
else
  __CONFIG_ROM_XZ ?= n
endif

# enable/disable bootloader, y to enable bootloader and disable some features
__CONFIG_BOOTLOADER ?= n

# secure boot
__CONFIG_SECURE_BOOT ?= n

# power manager
__CONFIG_PM ?= y

# OTA
__CONFIG_OTA ?= n

# ota policy, choose ota mode
#   - 0x00: ping-pong mode
#   - 0x01: image compression mode
__CONFIG_OTA_POLICY ?= 0x00

# bin compression
__CONFIG_BIN_COMPRESS ?= n
__CONFIG_BIN_COMPRESS_APP ?= y
__CONFIG_BIN_COMPRESS_APP_PSRAM ?= y

# xplayer
__CONFIG_XPLAYER ?= n

# JPEG
__CONFIG_JPEG ?= n
__CONFIG_JPEG_SHARE_64K ?= n

# mix sram/psram heap manager
ifeq ($(__CONFIG_MALLOC_USE_STDLIB), y)
  ifeq ($(__CONFIG_PSRAM), y)
    ifneq ($(__CONFIG_MALLOC_TRACE), y)
      __CONFIG_MIX_HEAP_MANAGE ?= n
    endif
  endif
endif
__CONFIG_MIX_HEAP_MANAGE ?= n

# config dma_malloc using psram
# unit KB, 0 is disable
ifeq ($(__CONFIG_PSRAM), y)
ifeq ($(__CONFIG_JPEG), y)
__CONFIG_DMAHEAP_PSRAM_SIZE ?= 768
else
__CONFIG_DMAHEAP_PSRAM_SIZE ?= 256
endif
else
__CONFIG_DMAHEAP_PSRAM_SIZE := 0
endif

# icache and dcache configure, sort by size sum of icache+dcache
#   - 0x00: icache  0 KB, dcache  0 KB
#   - 0x01: icache  0 KB, dcache  8 KB
#   - 0x10: icache  8 KB, dcache  0 KB
#   - 0x02: icache  0 KB, dcache 16 KB
#   - 0x11: icache  8 KB, dcache  8 KB
#   - 0x20: icache 16 KB, dcache  0 KB
#   - 0x12: icache  8 KB, dcache 16 KB
#   - 0x21: icache 16 KB, dcache  8 KB
#   - 0x04: icache  0 KB, dcache 32 KB
#   - 0x22: icache 16 KB, dcache 16 KB
#   - 0x40: icache 32 KB, dcache  0 KB
#   - 0x14: icache  8 KB, dcache 32 KB
#   - 0x41: icache 32 KB, dcache  8 KB
ifeq ($(__CONFIG_CHIP_TYPE), xr872)
ifeq ($(__CONFIG_PSRAM), y)
  __CONFIG_CACHE_POLICY ?= 0x41
else
  __CONFIG_CACHE_POLICY ?= 0x02
endif
else ifeq ($(__CONFIG_CHIP_TYPE), xr808)
  __CONFIG_CACHE_POLICY ?= 0x04
endif

# heap usage mode for different module
#   - mode 0: heap from sram
#   - mode 1: heap from psram
ifeq ($(__CONFIG_PSRAM), y)
__CONFIG_MBUF_HEAP_MODE ?= 0
__CONFIG_MBEDTLS_HEAP_MODE ?= 1
__CONFIG_HTTPC_HEAP_MODE ?= 1
__CONFIG_MQTT_HEAP_MODE ?= 1
__CONFIG_NOPOLL_HEAP_MODE ?= 1
__CONFIG_WPA_HEAP_MODE ?= 1
__CONFIG_UMAC_HEAP_MODE ?= 0
__CONFIG_LMAC_HEAP_MODE ?= 0
__CONFIG_CEDARX_HEAP_MODE ?= 1
__CONFIG_AUDIO_HEAP_MODE ?= 1
__CONFIG_CODEC_HEAP_MODE ?= 1
else
__CONFIG_MBUF_HEAP_MODE := 0
__CONFIG_MBEDTLS_HEAP_MODE := 0
__CONFIG_HTTPC_HEAP_MODE := 0
__CONFIG_MQTT_HEAP_MODE := 0
__CONFIG_NOPOLL_HEAP_MODE := 0
__CONFIG_WPA_HEAP_MODE := 0
__CONFIG_UMAC_HEAP_MODE := 0
__CONFIG_LMAC_HEAP_MODE := 0
__CONFIG_CEDARX_HEAP_MODE := 0
__CONFIG_AUDIO_HEAP_MODE := 0
__CONFIG_CODEC_HEAP_MODE := 0
endif

ifeq ($(__CONFIG_PSRAM), y)
__CONFIG_PSRAM_ALL_CACHEABLE ?= y
endif

# support setting cpu clock to 349MHz
__CONFIG_CPU_SUPPORT_349MHZ ?= n

# ----------------------------------------------------------------------------
# config symbols
# ----------------------------------------------------------------------------
CONFIG_SYMBOLS += -D__CONFIG_HOSC_TYPE=$(__CONFIG_HOSC_TYPE)

ifeq ($(__CONFIG_LIBC_REDEFINE_GCC_INT32_TYPE), y)
  CONFIG_SYMBOLS += -D__CONFIG_LIBC_REDEFINE_GCC_INT32_TYPE
endif

ifeq ($(__CONFIG_LIBC_PRINTF_FLOAT), y)
  CONFIG_SYMBOLS += -D__CONFIG_LIBC_PRINTF_FLOAT
endif

ifeq ($(__CONFIG_LIBC_SCANF_FLOAT), y)
  CONFIG_SYMBOLS += -D__CONFIG_LIBC_SCANF_FLOAT
endif

ifeq ($(__CONFIG_LIBC_WRAP_STDIO), y)
  CONFIG_SYMBOLS += -D__CONFIG_LIBC_WRAP_STDIO
endif

ifeq ($(__CONFIG_MALLOC_USE_STDLIB), y)
  CONFIG_SYMBOLS += -D__CONFIG_MALLOC_USE_STDLIB
endif

ifeq ($(__CONFIG_MALLOC_TRACE), y)
  CONFIG_SYMBOLS += -D__CONFIG_MALLOC_TRACE
endif

ifeq ($(__CONFIG_PSRAM_MALLOC_TRACE), y)
  CONFIG_SYMBOLS += -D__CONFIG_PSRAM_MALLOC_TRACE
endif

ifeq ($(__CONFIG_OS_FREERTOS), y)
  CONFIG_SYMBOLS += -D__CONFIG_OS_FREERTOS
  CONFIG_SYMBOLS += -D__CONFIG_OS_FREERTOS_VER=$(__CONFIG_OS_FREERTOS_VER)
endif

ifeq ($(__CONFIG_LWIP_V1), y)
  CONFIG_SYMBOLS += -D__CONFIG_LWIP_V1
endif

CONFIG_SYMBOLS += -D__CONFIG_MBEDTLS_VER=$(__CONFIG_MBEDTLS_VER)

CONFIG_SYMBOLS += -D__CONFIG_MBUF_IMPL_MODE=$(__CONFIG_MBUF_IMPL_MODE)

ifeq ($(__CONFIG_WLAN), y)
  CONFIG_SYMBOLS += -D__CONFIG_WLAN
else
  __CONFIG_WLAN_STA := n
  __CONFIG_WLAN_AP := n
  __CONFIG_WLAN_MONITOR := n
  __CONFIG_WLAN_STA_WPS := n
endif

ifeq ($(__CONFIG_WLAN_STA), y)
  CONFIG_SYMBOLS += -D__CONFIG_WLAN_STA
endif

ifeq ($(__CONFIG_WLAN_AP), y)
  CONFIG_SYMBOLS += -D__CONFIG_WLAN_AP
endif

ifeq ($(__CONFIG_WLAN_MONITOR), y)
  CONFIG_SYMBOLS += -D__CONFIG_WLAN_MONITOR
endif

ifeq ($(__CONFIG_WLAN_STA_WPS), y)
  CONFIG_SYMBOLS += -D__CONFIG_WLAN_STA_WPS
endif

ifeq ($(__CONFIG_ETF), y)
  CONFIG_SYMBOLS += -D__CONFIG_ETF
endif

ifeq ($(__CONFIG_ETF_CLI), y)
  CONFIG_SYMBOLS += -D__CONFIG_ETF_CLI
endif

ifeq ($(__CONFIG_WIFI_CERTIFIED), y)
  CONFIG_SYMBOLS += -D__CONFIG_WIFI_CERTIFIED
endif

ifeq ($(__CONFIG_XIP), y)
  CONFIG_SYMBOLS += -D__CONFIG_XIP
endif

ifeq ($(__CONFIG_PSRAM), y)
  CONFIG_SYMBOLS += -D__CONFIG_PSRAM
endif

ifeq ($(__CONFIG_PSRAM_CHIP_SQPI), y)
  CONFIG_SYMBOLS += -D__CONFIG_PSRAM_CHIP_SQPI
endif

ifeq ($(__CONFIG_PSRAM_CHIP_OPI32), y)
  CONFIG_SYMBOLS += -D__CONFIG_PSRAM_CHIP_OPI32
endif

ifeq ($(__CONFIG_PSRAM_CHIP_OPI64), y)
  CONFIG_SYMBOLS += -D__CONFIG_PSRAM_CHIP_OPI64
endif

ifeq ($(__CONFIG_SECTION_ATTRIBUTE_XIP), y)
  CONFIG_SYMBOLS += -D__CONFIG_SECTION_ATTRIBUTE_XIP
endif

ifeq ($(__CONFIG_SECTION_ATTRIBUTE_NONXIP), y)
  CONFIG_SYMBOLS += -D__CONFIG_SECTION_ATTRIBUTE_NONXIP
endif

ifeq ($(__CONFIG_SECTION_ATTRIBUTE_SRAM), y)
  CONFIG_SYMBOLS += -D__CONFIG_SECTION_ATTRIBUTE_SRAM
endif

ifeq ($(__CONFIG_SECTION_ATTRIBUTE_PSRAM), y)
  CONFIG_SYMBOLS += -D__CONFIG_SECTION_ATTRIBUTE_PSRAM
endif

ifeq ($(__CONFIG_ROM), y)
  CONFIG_SYMBOLS += -D__CONFIG_ROM
endif

ifeq ($(__CONFIG_ROM_FREERTOS), y)
  CONFIG_SYMBOLS += -D__CONFIG_ROM_FREERTOS
endif

ifeq ($(__CONFIG_ROM_XZ), y)
  CONFIG_SYMBOLS += -D__CONFIG_ROM_XZ
endif

ifeq ($(__CONFIG_BOOTLOADER), y)
  CONFIG_SYMBOLS += -D__CONFIG_BOOTLOADER
endif

ifeq ($(__CONFIG_SECURE_BOOT), y)
  CONFIG_SYMBOLS += -D__CONFIG_SECURE_BOOT
endif

ifeq ($(__CONFIG_PM), y)
  CONFIG_SYMBOLS += -D__CONFIG_PM
endif

ifeq ($(__CONFIG_OTA), y)
  CONFIG_SYMBOLS += -D__CONFIG_OTA
endif

CONFIG_SYMBOLS += -D__CONFIG_OTA_POLICY=$(__CONFIG_OTA_POLICY)

ifeq ($(__CONFIG_BIN_COMPRESS), y)
  CONFIG_SYMBOLS += -D__CONFIG_BIN_COMPRESS

ifeq ($(__CONFIG_BIN_COMPRESS_APP), y)
  CONFIG_SYMBOLS += -D__CONFIG_BIN_COMPRESS_APP
endif

ifeq ($(__CONFIG_BIN_COMPRESS_APP_PSRAM), y)
  CONFIG_SYMBOLS += -D__CONFIG_BIN_COMPRESS_APP_PSRAM
endif

endif # __CONFIG_BIN_COMPRESS

ifeq ($(__CONFIG_XPLAYER), y)
  CONFIG_SYMBOLS += -D__CONFIG_XPLAYER
endif

ifeq ($(__CONFIG_JPEG), y)
  CONFIG_SYMBOLS += -D__CONFIG_JPEG
endif

ifeq ($(__CONFIG_JPEG_SHARE_64K), y)
  CONFIG_SYMBOLS += -D__CONFIG_JPEG_SHARE_64K
endif

ifeq ($(__CONFIG_MIX_HEAP_MANAGE), y)
  CONFIG_SYMBOLS += -D__CONFIG_MIX_HEAP_MANAGE
endif

CONFIG_SYMBOLS += -D__CONFIG_CACHE_POLICY=$(__CONFIG_CACHE_POLICY)
CONFIG_SYMBOLS += -D__CONFIG_DMAHEAP_PSRAM_SIZE=$(__CONFIG_DMAHEAP_PSRAM_SIZE)

CONFIG_SYMBOLS += -D__CONFIG_MBUF_HEAP_MODE=$(__CONFIG_MBUF_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_MBEDTLS_HEAP_MODE=$(__CONFIG_MBEDTLS_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_HTTPC_HEAP_MODE=$(__CONFIG_HTTPC_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_MQTT_HEAP_MODE=$(__CONFIG_MQTT_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_NOPOLL_HEAP_MODE=$(__CONFIG_NOPOLL_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_WPA_HEAP_MODE=$(__CONFIG_WPA_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_UMAC_HEAP_MODE=$(__CONFIG_UMAC_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_LMAC_HEAP_MODE=$(__CONFIG_LMAC_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_CEDARX_HEAP_MODE=$(__CONFIG_CEDARX_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_AUDIO_HEAP_MODE=$(__CONFIG_AUDIO_HEAP_MODE)
CONFIG_SYMBOLS += -D__CONFIG_CODEC_HEAP_MODE=$(__CONFIG_CODEC_HEAP_MODE)

ifeq ($(__CONFIG_PSRAM_ALL_CACHEABLE), y)
  CONFIG_SYMBOLS += -D__CONFIG_PSRAM_ALL_CACHEABLE
endif

ifeq ($(__CONFIG_CPU_SUPPORT_349MHZ), y)
  CONFIG_SYMBOLS += -D__CONFIG_CPU_SUPPORT_349MHZ
endif

