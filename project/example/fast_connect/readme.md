# fast con示例工程

> fast con示例工程展示了wlan模块的快速连接相关接口的使用方法。
>
> 本工程中提供以下模块接口使用的示例：
> 1. 第一次普通连接AP的过程
> 2. 后续的快速连接AP的过程

---

## 适用平台

> 本工程适用以下芯片类型：
> 1. XR808系列芯片：XR808ST、XR872CT
> 2. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR872MD_EVB、XR872MD_IO、XR808MD_EVB_IO
> 2. 模组：XR872AT_MD01、XR808ST_MD01、XR808CT_MD02、XR808CT_MD01

> 本工程在基于XR872ET的“XR808ST_MD01”评估板上测试通过。
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
> * N/A

## 模块依赖

> 必选项
> 1.libxrwireless.a：
> 2.libwlan.a

---

## 工程说明

> 本工程的实现为第一次正常连接AP后，将对应的BSS信息保存到flash的指定位置，后续的启动都会使用快速启动连接模式

### 操作说明：

> 1. 修改工程中的 sta_ssid 和 sta_psk 字符串为实际需要连接的AP的字符串
> 2. 编译工程，烧录镜像，启动即可
> 3. 系统启动后，可以通过串口软件看到示例的打印信息
> 4. 首次启动为普通连接，在连上AP之后会保存BSS信息到flash
> 5. 后续启动为快速连接

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 控制命令

> * N/A

### 代码结构
```
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置规则，用于覆盖默认配置
│   └── Makefile                # 本工程的编译规则，可指定src、lib、ld、image.cfg、board_config等文件
├── image
│   └── xr872
│       └── image.cfg           # 本工程的镜像分区配置
├── main.c                      # 本工程的入口，运行fast con的示例代码
├── prj_config.h                # 本工程的配置选项，主要用于功能的选择。
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_evb             #本工程在Makefile中指定使用xradio_evb的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口：
> A）初始化系统
> B）执行fast_con_example()
> 2. 执行fast_con_example()函数流程：
> A）尝试从flash指定位置读取BSS信息
> B）如果没有读取到数据，执行普通连接流程
> C）如果读取到了数据，执行快速连接流程
> 3. 执行普通连接流程：
> A）设置SSID和PSK，然后使能STA功能，连接AP
> B）等待连接上AP之后，将IP地址等信息保存到flash
> C）获取当前连接的BSS信息，将其保存到flash
> 4. 执行快速连接流程：
> A）设置flash中获取到的BSS信息到网络端
> B）设置flash中获取到的SSID和PSK
> C）使能STA功能，连接AP
> D）等待连接上AP

---

## 常见问题

> * N/A

## 参考文档

> * N/A
