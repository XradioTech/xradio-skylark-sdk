# voice print 示例工程

> voice print 示例工程展示了XRadio SDK中进行声波配网的代码实现方法。
>

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR872MD_EVB
> 2. 模组：XR872AT_MD01

> 本工程在基于XR872ET的“XR872MD_EVB+XR872AT_MD01”评估板上测试通过。
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
> * PRJCONF_INTERNAL_SOUNDCARD_EN: 必选项，配置使用内置声卡
> * PRJCONF_NET_EN: 必选项，配置使能网络功能

## 模块依赖

> 必选项
>
> 1. wlan模块： 网络连接需要依赖的库

---

## 工程说明

> 本工程会启动声波配网，并进行wifi连接。

### 操作说明

> 1. 编译工程，烧录镜像，启动
> 2. 手机端启动声波配网工具或微信公众号，开始配网

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
│   └── Makefile                # 本工程的编译规则，如ld文件、image.cfg、board_config.h等文件指定，可覆盖默认配置
├── main.c                      # 本工程的代码实现
├── prj_config.h                # 本工程的配置选项，主要用于功能的选择。
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_audio           #本工程在Makefile中指定使用xradio_audio的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口： 调用voice_print_example()启动声波配网。
> 3. voice_print_example()函数流程：
>   A）初始化配网助手
>   B）启动声波配网
>   C）等待配网成功或超时，这里超时时间设置为120s
>   D）配网成功的话，则获取配网结果
>   E）解析配网结果，获取wifi的ssid和password
>   F）配网助手进行联网
>

---

## 常见问题

无

## 参考文档

无