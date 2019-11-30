# ChangeLog for XRADIO Skylark SDK

## xradio_skylark_sdk-1.0.0 (20191129)
  * Driver
    - CACHE: fix several bugs of cache operations
    - SPI: support slave mode
    - CAMERA: fix a bug about CSI exception
    - Audio Codec: support delay after setting PA on

  * WLAN
    - Optimize EVM
    - Add an API to set rx queue size

  * Misc
    - Fix a bug of setting sram retention
    - Support decoding QR code


## xradio_skylark_sdk-1.0.0-rc3 (20191115)
  * WLAN
    - Improve the RF performance
    - Fix a bug about dpd calculation exception
    - Increase limitation for 802.11b tx power
    - Fix bugs of smart config to avoid memory writing out of bounds
    - Add two new APIs for sta mode and ap mode

  * Driver
    - CACHE: adjust some h/w operations according to spec

  * System
    - Support PSRAM heap trace function
    - adjust board config names


## xradio_skylark_sdk-1.0.0-rc2 (20191104)
  * Driver
    - PSRAM: Improve the performance and stability
    - IR: support 40M HOSC
    - CSI/JPEG: improve the working flow and fix several small issues
    - DMA: fix data inconsistent issue when using cache
    - FLASH: add support for P25Q64H

  * WLAN
    - Improve working efficiency and power
    - Fix a bug about memory leakage in some cases
    - Fix a bug about failed to connect the open AP after setting WEP keys
    - Add APIs to get the number of scan results

  * Cedarx
    - Fix a bug of blocking when stop playing in some cases
    - Fix some issues about playing AAC stream and HTTP stream
    - Improve the speed of parsing MP3 stream
    - Add an API to set buffer size for playing HTTP stream

  * System
    - Add heap usage mode (SRAM or PSRAM) configurations for some modules
    - Add some policies to improve the system's stability

  * Misc
    - FOTA: support image compression policy
    - Bootloader: fix a bug of decompressing bin files failed
    - Add some project examples


## xradio_skylark_sdk-1.0.0-rc1 (20190925)
  * First release candidate for XRADIO Skylark SDK
