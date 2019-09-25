# dhcpd  示例工程

> dpcpd 示例工程展示了XRadio SDK中dhcpd模块接口的使用方法。
>

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR808系列芯片：XR808CT、XR808ST
> 2. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR808MD_EVB_IO、XR872MD_IO、XR872MD_EVB
> 2. 模组：XR808CT_MD01、XR808CT_MD02、XR808ST_MD01、XR872AT_MD01

> 本工程在基于XR872AT的“XR872AT_VER_V1_0”板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

## 工程配置

> localconfig.mk：
> * N/A
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
> * PRJCONF_NET_EN: 必选项，配置使能网络功能

## 模块依赖

> 必选项
>
> 1. wlan模块： 网络连接需要依赖的库

---

## 工程说明

> 本工程启动后，配置开发板为AP模式，然后开启dhcpd的运行。工程中dhcpd默认的租约时间为60min，最大租约数量为5，IP池地址范围为192.168.51.100 ~ 192.168.51.150。

### 操作说明

> 1. 编译工程，烧录镜像，启动
> 2. 在系统提示进行连接AP时，可以使用手机/平板等作为客户端连接AP
> 3. 连接成功后，查看客户端分配到的IP地址是否在IP池范围内

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 控制命令

> N/A

### 代码结构
```
#本工程
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置选项，主要用于覆盖默认全局配置
│   └── Makefile                # 本工程的编译规则，如ld文件、image.cfg、等文件指定，可覆盖默认配置
├── main.c                      # 本工程的代码实现
├── prj_config.h                # 本工程的配置选项，主要用于功能的选择。
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_audio           #本工程在Makefile中指定使用xradio_evb的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口： 调用dhcpd_demo()进行dhcpd模块运行。
> 3. dhcpd_demo()函数流程：
>   A）配置系统为AP模式
>   B）重启dhcpd
>   C）等待客户端连接
>   

---

## 常见问题

无

## 参考文档

无