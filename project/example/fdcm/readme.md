# fdcm 示例工程

> fdcm示例工程展示了XRadio SDK中使用fdcm保存数据的代码实现方法。
>

---

## 适用平台

> 本工程适用以下芯片类型：
>
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
> * N/A

## 模块依赖

> N/A
>

---

## 工程说明

> 本工程会在flash里创建一个fdcm区域，用来存放信息。示例里会多次进行fdcm区域的写和读操作。通过串口打印，可以看出写进fdcm的信息和读取出来的信息是一致的

### 操作说明

> 1. 编译工程，烧录镜像，启动即可

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 控制命令

> 无

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
            └── xradio_evb           #本工程在Makefile中指定使用xradio_evb的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。
```
### 代码流程

> 1. main()入口： 调用fdcm_example()进行fdcm操作。
> 2. fdcm_example()函数流程：
>   A）打开/创建fdcm模块
>   B）往fdcm区域写入一个wifi ssid和password信息
>   C）从fdcm区域读取信息
>   D）打印写入的信息和读取到的信息。通过打印，可看出两者是一致的
>   E）重复进行B、C、D操作
>   F）关闭/销毁fdcm模块
>

---

## 常见问题

无

## 参考文档

无