# QR  示例工程

> QR 示例工程展示了QR模块识别解码二维码使用方法。
>
> 本工程中提供以下模块接口使用的示例：
>
> 1. 摄像头拍摄一帧QR图像，然后进行解码并打印识别的结果

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT

> 本工程适用以下评估板类型：
> 1. 底板：XR872_EVB_AI
> 2. 模组：XR872AT_MD01

> 本工程在基于"XR872AT_MD01"的“XR872_EVB_AI”板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://docs.xradiotech.com

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
> - PRJCONF_MMC_EN：可选项， 配置使用SD卡功能。
>

## 模块依赖

> - libzbar.a：条形码解码库

## 工程说明

> 本工程对QR模块拍摄QR图像，然后识别解码的使用进行说明（JPEG工作方式是离线模式，因为需要获取YUV图像），解码后的结果输出到串口中去。
>
> 工程拍摄的图像视野是160x120，因此摄像头尽可能对准QR图像的中心，且图像大小不能过大（例如工程测试的图像大小为 80x60）。实测效果来看，摄像头与待拍摄的QR图像距离为7.1cm左右时，基本都能识别出QR码。

### 操作说明：

> 1. 编译工程，烧录镜像，启动。
>
> 2. 系统启动后，适当调整摄像头与QR码的位置，可以在串口中看到识别后的信息。

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://docs.xradiotech.com

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
├── main.c                      # 本工程的入口，进行QR图像解码的示例说明
├── prj_config.h                # 本工程的配置规则
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xr872_evb_ai           #本工程在Makefile中指定使用xr872_evb_ai的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口：拍摄一帧QR码并打印识别后结果的操作示例。

>		更详细的开发指南请参考《XRADIO_QR_Developr_Guide-CN.doc》

---

## 常见问题

> N/A

## 参考文档

> 文档资源
>
1. 《XRADIO_QR_Developr_Guide-CN.doc》
