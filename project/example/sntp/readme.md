# sntp  示例工程

> sntp 示例工程展示了XRadio SDK中进行网络实现时间同步的实现方法。
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
> * PRJCONF_NET_EN: 必选项，配置使能网络功能

## 模块依赖

> 必选项
>
> 1. wlan模块： 网络连接需要依赖的库

---

## 工程说明

> 本工程启动后，在接入网络后会每隔5s左右请求远程NTP/SNTP服务器时间，并将打印获取到的具体实时时间，若请求失败则退出系统运行。

### 操作说明

> 1. 编译工程，烧录镜像，启动
> 2. 往串口输入配网命令进行网络连接
> 3. 成功联网后，可以看到每隔5s左右串口会打印实时时间

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 控制命令

> 1. 网络连接
>
>    $ net sta config <ssid> <passphrase>
>
>    $ net sta enable

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

> 1. main()入口： 调用sntp_test()进行远程NTP/SNTP服务器时间获取测试。
> 3. sntp_test()函数流程：
>   A）等待网络连接
>   B）网络连接成功后，请求远程NTP/SNTP服务器时间
>   C）请求成功后，则打印获取到的当下实时时间，否则则退出该函数
>   D）休眠5s左右，继续进行上述操作
>   

---

## 常见问题

无

## 参考文档

无