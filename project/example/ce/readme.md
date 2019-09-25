

# 硬件加解密模块(CE)示例工程

> 硬件加解密模块示例工程展示了XRadio SDK中CE模块中常用的加解密接口使用方法。
>
> 本工程中提供了以下几种加解密示例：
>
> 1. AES、CRC、MD5、伪随机数获取。

------

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
>
> 1. 底板：XR872MD_EVB
> 2. 模组：XR872AT_MD01

> 本工程在基于XR872ET的“XR872MD_EVB+XR872AT_MD01”评估板上测试通过。
>
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> localconfig.mk：
>
> - N/A
>
> Makefile：
>
> - N/A
>
> board_config.h：
>
> - N/A
>
> board_config.c：
>
> - N/A
>
> prj_config.h：
>
> - PRJCONF_CE_EN: 可选项，PRJCONF_CE_EN为1时，系统初始化时会自动初始化CE模块，应用层要用CE模块时，可直接用，不必初始化；为0时，系统初始化时不会自动初始化CE模块，应用层要用CE模块时，则需要先初始化CE模块。

## 模块依赖

> 必选项
>
> 无

> 可选项
>
> 无

## 工程说明

> 本工程为CE模块的使用用例，使用CE模块前，需要对常用的加解密算法或哈希算法有一定了解，比如AES，DES，3DES、CRC、SHA、MD5等。
>
> CE模块支持常用的AES、DES、3DES加解密，支持的模式有ECB、CBC。支持的哈希算法有：CRC、MD5、SHA1、SHA256。支持的随机数发生器有：伪随机发生器、真随机发生器。
>
> 本工程只演示了AES加解密、CRC、MD5、伪随机发生器的用法，其他算法的接口用法可参考头文件，也可参考`project/common/cmd/cmd_ce.c`中的代码。
>

### 操作说明

> 1. 编译工程，烧录镜像，重启即可
> 2. 打开串口调试工具，并连接串口
> 3. 系统启动后，会自动打印加密结果

### 代码结构

```
#本工程
.
├── gcc
│   ├── localconfig.mk     # 本工程的配置选项，如board_config.h等文件指定，主要用于覆盖默认全局配置
│   └── Makefile           # 本工程的编译规则，如ld文件、image.cfg等文件指定，可覆盖默认配置
├── main.c                 # 本工程的入口，工程示例
├── prj_config.h           # 本工程的配置选项，主要用于功能的选择。
└── readme.md              # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_evb             #本工程在localconfig.mk中指定使用xradio_evb的板级配置
                ├── board_config.h     #本工程的板级配置
                └── board_config.c     #本工程的板级pin mux的配置。
```

## 性能资源

> 各加解密算法加解密数据长度及所需时间数据如下：
>
> 在AES、DES和3DES中，在ce启动后，每增加一个block，加解密增加的时间是固定的。

| 加密模式及秘钥长度 | 16 bytes | 32 bytes | 48 bytes | 64 bytes | 80 bytes |
| :----------------: | :------: | :------: | :------: | :------: | :------: |
|    AES_ECB_128     |  4.1us   |  5.1us   |  6.2us   |  7.4us   |  8.5us   |
|    AES_ECB_192     |  4.2us   |  5.6us   |  6.8us   |  8.0us   |  9.2us   |
|    AES_ECB_256     |  4.8us   |  6.1us   |  7.4us   |  8.7us   |  10.1us  |
|    AES_CBC_128     |  4.9us   |  6.0us   |  7.1us   |  8.2us   |  9.3us   |
|    AES_CBC_192     |  5.5us   |  6.5us   |  7.7us   |  8.9us   |  10.1us  |
|    AES_CBC_256     |  5.6us   |  6.9us   |  8.2us   |  9.6us   |  10.9us  |

| 加密模式 | 8 bytes | 16 bytes | 24 bytes | 32 bytes | 40 bytes |
| :------: | :-----: | :------: | :------: | :------: | :------: |
| DES_ECB  |  3.2us  |  3.6us   |  4.2us   |  4.7us   |  5.2us   |
| DES_CBC  |  3.5us  |  4.0us   |  4.5us   |  5.0us   |  5.8us   |
| 3DES_ECB |  4.1us  |  4.8us   |  5.2us   |  6.5us   |  7.3us   |
| 3DES_CBC |  4.5us  |  5.3us   |  6.1us   |  7.0us   |  7.8us   |

| 加密模式 | 50 bytes | 100 bytes | 150 bytes | 200 bytes | 250 bytes |
| :------: | :------: | :-------: | :-------: | :-------: | :-------: |
|   SHA1   |  5.6us   |   6.4us   |   7.3us   |   8.1us   |   9.1us   |
|  SHA256  |  5.5us   |   6.3us   |   7.2us   |   8.0us   |   8.9us   |
|   MD5    |  5.4us   |   6.2us   |   7.1us   |   7.9us   |   8.7us   |

## 注意事项

> AES、DES、3DES的填充方式为zeropadding，如果需要其他填充方式，需要自行修改ce驱动代码。

## 常见问题

> 无
>

## 参考文档

> 无