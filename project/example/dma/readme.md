# dma示例工程

> dma示例工程展示了dma模块接口使用方法。
>
> 本工程中提供以下模块接口使用的示例：
> 1. DMA SRAM复制数据模式使用
> 2. DMA UART0收发数据模式使用

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

> 本工程通过串口打印演示dma模块 DMA SRAM复制数据模式、DMA UART0收发数据模式使用使用, 通过使能宏定义选择dma工作模式。

### 操作说明：

> 1. 使用串口线连接UART0接口
> 2. 编译工程，烧录镜像，复位启动
> 3. 系统启动后，可以通过串口软件看到示例的打印信息
> 4. 如果通过使能宏定义选择DMA UART0收发数据模式（uart0 在平台初始化时已配置，无需再配置），在dma发送数据后需要使用者手动通过串口软件发送相应的数据，cpu通过dma拿到使用者手动发送的数据后会与原来的数据做比对，并打印比对结果

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

> 1. main()入口：调用Dma_Show()函数
> 2. Dma_Show()函数流程：
> A）初始化dma write、read通道
> B）调用Dma_Write_Read()函数, 执行dma写、读操作
> C）dma通道 Deinit操作
> 3. Dma_Write_Read()函数流程：
> A）如果选择DMA SRAM复制数据模式，启动DMA写数据，等待完成，启动DMA读数据，等待完成，比对数据，打印比对结果
> B）如果选择DMA UART0收发数据模式，启动DMA发数据，等待完成，启动DMA读数据，等待使用者通过串口Uart0发送对应长度数据，比对数据，打印比对结果
---

## 常见问题

> * N/A

## 参考文档

> * N/A
