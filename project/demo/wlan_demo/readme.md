# wlan_demo 演示工程

> wlan_demo演示工程展示了XRadio SDK Wi-Fi模块相关的使用方法。

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片
> 2. XR808系列芯片

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://docs.xradiotech.com

## 工程配置

> localconfig.mk：
> * __CONFIG_WLAN_STA：使用STA模式时将此项置为y
> * __CONFIG_WLAN_STA_WPS：使用STA模式并且使用WPS功能时，将此项置为y，注意不能与__CONFIG_WLAN_STA同时置为y
> * __CONFIG_WLAN_AP：使用AP模式时将此项置为y
>
> Makefile：
> * IMAGE_CFG：选择工程的flash布局文件。如无，则使用默认配置
> * LINKER_SCRIPT：选择工程的ld文件。如无，则使用默认ld文件
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
> * PRJCONF_CONSOLE_EN：必选项，配置使用控制台
> * PRJCONF_NET_EN：必选项，配置使用网络

## 模块依赖

> libwireless.a：网络底层驱动
> libwlan.a：网络框架层

---

## 工程说明

> 本工程为wlan演示工程。

### 操作说明

> 1.编译工程，烧写固件，启动
>
> 2.输入命令进行STA模式连接AP，或者进入AP模式等
>
> 3.输入命令进行ping、iperf等网络应用测试
> 
> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://docs.xradiotech.com

### 控制命令

> N/A

### 代码结构
```
#本工程
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile                # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
├── image
│   └── xr872
│       └── image.cfg           # 本工程的镜像布局配置
├── main.c                      # 本工程的入口，完成平台初始化
├── command.c                   # 本工程的控制台命令入口
├── command.h
├── prj_config.h                # 本工程的配置选项，主要用于功能的选择。
└── readme.md                   # 本工程的说明文档

#本工程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xr872_evb_ai           #本工程在Makefile中指定使用xr872_evb_ai的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口：初始化平台。
> 2. 等待控制台输入联网等操作。

---


## 常见问题

> N/A
