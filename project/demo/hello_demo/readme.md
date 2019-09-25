# hello_demo演示工程

> hello_demo演示工程展示了XRadio SDK常用工程配置和系统初始化流程。
>
> 本工程中提供以下常用工程配置和系统初始化流程的演示：
> 1. 常用工程配置：启用XIP、OTA、网络功能
> 2. 支持常用的控制台命令，可用于常用网络功能演示、OTA升级、系统调试等
> 3. 在main()函数入口完成平台初始化后，每隔10秒打印“Hello world! @ xxx sec”

---

## 适用平台

> 本工程适用以下芯片类型：
> 1. XR872系列芯片：XR872AT、XR872ET
> 2. XR808系列芯片：XR808ST、XR808CT

> 本工程适用以下评估板类型：
> 1. 底板：XR872MD_EVB、XR872MD_IO、XR808MD_EVB_IO
> 2. 模组：XR872AT_MD01、XR808ST_MD01、XR808CT_MD02、XR808CT_MD01

> 本工程在基于XR872ET的“XR872MD_EVB+XR872AT_MD01”评估板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> localconfig.mk：
> * __CONFIG_XIP: 可选项，配置是否开启XIP功能
> * __CONFIG_OTA: 可选项，配置是否开启OTA功能
>
> Makefile：
> * PRJ_BOARD：必选项，选择板子的板级配置路径
>
> board_config.h
> * N/A
>
> board_config.c
> * N/A
>
> prj_config.h
>
> * PRJCONF_NET_EN: 可选项，配置是否开启网络功能

## 模块依赖

> 可选项
> 1. liblwip.a： 开启网络功能需要依赖的库
> 2. wlan模块： 开启网络功能需要依赖的库

---

## 工程说明

> 本工程完成系统初始化后，每隔10秒打印“Hello world! @ xxx sec”。

### 操作说明：
> 1. 使用串口线连接UART0连接至PC
> 2. 编译工程，烧录镜像，复位启动
> 3. 系统启动10秒后，每隔10秒打印“Hello world! @ xxx sec”


> XRadio SDK的编译、烧写等操作方式的说明可参考《XRADIO_Quick_Start_Guide-CN》

### 控制命令

> 1. 连网操作（可选操作）
```
$ net sta config <your_ssid> [your_password]
$ net sta enable
```

### 代码结构
```
#本工程
.
├── command.c           # 本工程的控制台命令入口和命令定义
├── command.h
├── gcc
│   ├── localconfig.mk  # 本工程的配置选项，可覆盖全局默认配置
│   └── Makefile        # 本工程的Makefile，可指定src、board config、ld、image cfg
├── main.c              # 本工程的入口，完成平台初始化后，每隔10秒打印“Hello world! @ xxx sec”
├── prj_config.h        # 本工程的功能选项配置，主要用于功能的选择
└── readme.md           # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_evb             # 本工程在Makefile中指定使用xradio_evb的板级配置
                ├── board_config.h     # 本工程的板级配置
                └── board_config.c     # 本工程的板级pin mux的配置
```
### 代码流程

> 1. main()入口：
>     A) 调用platform_init()完成平台初始化
>     B) 执行while循环，每隔10秒打印“Hello world! @ xxx sec”
---

## 常见问题

> * N/A

## 参考文档

> 《XRADIO_Quick_Start_Guide-CN》
