# JPEG 示例工程

> JPEG示例工程展示了camera模块获取JPEG图像的使用方法。
>
> 本工程中提供以下模块接口使用的示例：
>
> 1. 获取一帧JPEG图像，然后保存在SD卡中去

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR872MD_IO、XR872MD_EVB
> 2. 模组：XR872AT_MD01

> 本工程在基于XR872ET的“XR872AT_VER_V1_0”板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

---

## 工程配置

> localconfig.mk：
>
> - __CONFIG_JPEG：必选项， 配置JPEG模块使能，需使能。
> - __CONFIG_JPEG_SHAR_SRAM_64K：可选项，配置要切出给JPEG使用的内存大小。当图像的高大于576或宽大于720时需要使能该项。
> - __CONFIG_PSRAM：可选项，配置工程PSRAM使能，当工程使用PSRAM功能时需使能该项。
> - __CONFIG_PSRAM_CHIP_OPI32：可选项，配置PSRAM的类型。
>
> Makefile：
>
> - N/A
>
> board_config.h
>
> - N/A
>
> board_config.c
>
> - N/A
>
> prj_config.h
>
> - PRJCONF_MMC_EN: 必选项， 配置使用SD卡功能，需使能。
>
> -  PRJCONF_CSI_SDC_EN：必选项，配置CSI、SDC功能PIN脚的选择，需使能。

## 模块依赖

> N/A

## 工程说明

> 本工程对camera模块捕获一帧JPEG图像，然后保存在SD卡中的使用进行说明（JPEG工作方式是在线模式），获取到的图像保存在SD卡根目录下的“test.jpg”。
>
> 如果想同时获取JEPG和YUV图像，需要配置JPEG模块工作模式为离线模式（将工程的JPEG_ONLINE_EN配置为0），同时需要使用PSRAM功能（将工程的JPEG_PSRAM_EN配置为1），否则图像数据过大，内存不够。获取到的YUV图像保存在SD卡根目录下的test.YUV文件。

### 操作说明：

> 1. 用跳线将PB16/PB17/PB18分别连接到SD卡槽的SD_CMD/SD_DATA0/SD_CLK信号脚上去，然后插入SD卡。
> 2. 编译工程，烧录镜像，启动。
> 3. 系统启动后，可以看到JPEG图像大小的打印信息，并提示获取图像成功。

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 代码结构
```
.
├── command.c          			# 本工程的控制台命令入口和命令定义
├── command.h
├── gcc
│   ├── localconfig.mk          # 本工程的配置规则，用于覆盖默认配置
│   └── Makefile                # 本工程的编译选项，可覆盖默认配置
├── image
│   └── xr872
│       └── image.cfg           # 本工程的镜像分区配置
├── main.c                      # 本工程的入口，进行JPEG图像捕捉到的示例说明
├── prj_config.h                # 本工程的配置规则
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_evb           #本工程在Makefile中指定使用xradio_evb的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口： 执行捕获一帧JPEG图像的操作示例。
> 

---

## 常见问题

> N/A

## 参考文档

> N/A
