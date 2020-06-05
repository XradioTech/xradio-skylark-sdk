# IR 示例工程

> IR示例工程展示了红外模块发送、接收的使用方法。
>
> 本工程中提供以下模块接口使用的示例：
>
> 1. IRRX、IRTX直连通信
> 2. IRTX载波调制发送、IRRX接收

---

## 适用平台

> 本工程适用以下芯片类型：
>
> 1. XR808系列芯片：XR808CT
> 2. XR872系列芯片：XR872AT

> 本工程适用以下评估板类型：
> 1. 底板：XR872_EVB_IO、XR872_EVB_AI
> 2. 模组：XR872AT_MD01

> 本工程未在以下评估板底板上测试：XR872_EVB_AI、XR872_EVB_IO，代码仅供参考。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://docs.xradiotech.com

## 工程配置

> localconfig.mk：
>
> - N/A
>
> Makefile：
>
> - N/A
>
> board_config.h
>
> - N/A
>
> board_config.c
>
> - N/A
>
> prj_config.h
>
> - N/A

## 模块依赖

> N/A

## 工程说明

> 本工程对IR模块使用进行介绍，宏IR_IMS_TEST_EN表示通信是否启动内部载波调制功能。
>
> IR_IMS_TEST_EN为0时，工程执行的IRRX、IRTX直连测试的功能，即IR模块没有外挂红外发射管及接收头，此时需要用跳线将PA12和PA16相连接。
>
> IR_IMS_TEST_EN为1时，工程执行的IRRX、IRTX载波调制的功能，IR模块需要外挂红外发射管及接收头。
>
> 注：使用的通信协议是NEC。

### 操作说明：

> 1. 编译工程，烧录镜像，启动。
> 3. 系统启动后，可以看到打印信息“treceived ir code addr:0x01 key:0x02”，则说明功能正常。
> 3. 系统启动后10s内，用红外遥控器往IR接收头发送信号，IRTX亦能收到信号并打印在串口上。

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://docs.xradiotech.com

### 代码结构
```
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置规则，用于覆盖默认配置
│   └── Makefile                # 本工程的编译选项，可覆盖默认配置
├── image
│   └── xr872
│       └── image.cfg           # 本工程的镜像分区配置
├── main.c                      # 本工程的入口，进行IR通信示例说明
├── prj_config.h                # 本工程的配置规则
└── readme.md                   # 本工程的说明文档

#本程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xr872_evb_ai           #本工程在Makefile中指定使用xr872_evb_ai的板级配置
                ├── board_config.h     #本工程的板级配置
                └── board_config.c     #本工程的板级pin mux的配置
```
### 代码流程

> 1. main()入口：执行IR通信的操作示例。
> 

---

## 常见问题

> N/A

## 参考文档

> N/A
