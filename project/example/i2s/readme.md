# I2S示例工程

> I2S示例工程展示了I2S模块接口使用方法。
>
> 本工程中提供以下模块接口使用的示例：
>
> 1. I2S发送和接收LOOP BACK的使用

---

## 适用平台

> 本工程适用以下芯片类型：
> 1. XR872系列芯片：XR872AT、XR872ET

> 本工程适用以下评估板类型：
> 1. 底板：XR872MD_EVB
> 2. 模组：XR872AT_MD01

> 本工程在基于XR872AT的“XR872AT_VER_V1_0”数字板上测试通过。
> 若需要在其他适用芯片和评估板上运行本工程，请根据快速指南《XRadio_Quick_Start_Guide-CN》的提示进行相关配置修改。

> XRadio Wireless MCU芯片和评估板的更多信息可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

---

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

---

## 模块依赖
>N/A

---

## 工程说明

> 本工程对I2S模块发送和接收LOOP BACK的使用进行介绍。

### 操作说明：

> 1. 打开src\driver\chip\hal_i2s.c中的LOOP_EN宏定义
> 2. 编译工程，烧录镜像，启动即可
> 3. 系统启动后，可以看到测试的打印信息

> XRadio SDK的编译、烧写等操作方式的说明可在以下地址获取：
> https://github.com/XradioTech/xradiotech-wiki

### 代码结构
```
.
├── gcc
│   ├── localconfig.mk          # 本工程的配置规则，用于覆盖默认配置
│   └── Makefile                # 本工程的编译选项，可覆盖默认配置
├── image
│   └── xr872
│       └── image.cfg           # 本工程的镜像分区配置
├── main.c                       # 本工程的入口，进行I2S LOOPBACK测试的示例说明
├── prj_config.h                 # 本工程的配置规则
└── readme.md                    # 本工程的说明文档

# 本工程用到XRadio SDK的其他配置文件
.
└── project
    └── common
        └── board
            └── xradio_evb    #本工程在Makefile中指定使用xradio_evb的板级配置
                ├── board_config.h     #本工程的板级配置，
                └── board_config.c     #本工程的板级pin mux的配置。

```
### 代码流程

> 1. main()入口： 执行I2S发送和接收LOOP BACK的操作示例。

---

## 常见问题

无

## 参考文档

无
