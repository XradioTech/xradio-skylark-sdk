# uart示例工程

> uart示例工程展示了uart模块接口使用方法。
>
> 本工程中提供以下模块接口使用的示例：
> 1. DMA收发模式使用
> 2. 中断收发模式使用
> 3. 轮询收发模式使用

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

> * N/A

---

## 工程说明

> 本工程通过串口UART1收发数据演示uart模块DMA收发模式、中断收发模式、轮询收发模式的使用, 通过使能宏定义选择wdg工作模式。

### 操作说明：

> 1. 使用串口线连接UART0、UART1接口
> 2. 编译工程，烧录镜像，复位启动
> 3. 系统启动后，可以通过串口软件看到示例的打印信息（UART0），UART1也会打印一行提示信息
> 4. 可以通过串口软件给UART1发送数据，系统会将接受到的数据通过UART1发送回去

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
├── main.c                      # 本工程的入口，进行uart三种收发模式的选择和执行
├── prj_config.h                # 本工程的配置规则
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
> A）UART初始化
> B）根据条件执行对应的uart收发模式
> 2. uart_dma_mode()函数流程：
> A）使能DMA收发
> B）通过UART1发送一行提示
> C）等待接受来自UART1的数据，收到数据后发送回去
> 3. uart_it_mode()函数流程：
> A）通过UART1发送一行提示
> B）等待接受来自UART1的数据，收到数据后发送回去
> 4. uart_poll_mode()函数流程：
> A）通过UART1发送一行提示
> B）等待接受来自UART1的数据，收到数据后发送回去
---

## 常见问题

> * N/A

## 参考文档

> * N/A
