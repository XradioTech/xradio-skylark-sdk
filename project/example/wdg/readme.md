# wdg示例工程

> wdg示例工程展示了wdg模块接口使用方法。
>
> 本工程中提供以下模块接口使用的示例：
> 1. wdg的系统复位模式使用
> 2. wdg的触发中断模式使用
> 3. wdg的CPU复位模式使用

---

## 适用平台

> 本工程适用以下芯片类型：
> 1. XR808系列芯片：XR808ST、XR872CT
> 2. XR872系列芯片：XR872AT、XR872ET

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
> * N/A

## 模块依赖

> * N/A

---

## 工程说明

> 本工程通过串口UART0打印信息演示wdg模块系统复位模式、触发中断模式、CPU复位模式的使用, 通过使能宏定义选择wdg工作模式。

### 操作说明：

> 1. 使用串口线连接UART0接口
> 2. 编译工程，烧录镜像，启动即可
> 3. 系统启动后，可以通过串口软件看到示例的打印信息

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
├── main.c                      # 本工程的入口，进行wdg三种工作模式的选择和执行
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
> A）根据条件执行对应的wdg模式初始化操作
> B）执行WdgShow()
> C）执行DeInit操作
> 2. WdgShow()函数流程：
> A）启动wdg
> B）通过WdgFeedDog()触发wdg
> C）停止wdg

---

## 常见问题

> * N/A

## 参考文档

> * N/A
